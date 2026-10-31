// pipeline/pipeline.cpp
#include "pipeline/pipeline.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "pipeline/solver.h"
#include "vision/angles.h"
#include "vision/utils.h"

namespace {

// 开火窗口：目标和炮口方向的角度差在这个范围内才开火。
// 太小会打不出去（云台在追的时候几乎永远差一点），太大会送弹药。
// 参考量级：装甲板半高在 5 m 处约 0.68°，所以窗口必须明显小于它。
constexpr double kFireWindowDeg = 0.6;

// 观测超过这么久没更新，就停控停火。
// 必须小于仿真的指令看门狗（250 ms），否则"丢失目标后继续开火"在真机上是事故。
constexpr double kObservationTimeout = 0.20;

// 开火还要求跟踪已经稳了：至少这么多帧、且拟合残差不大（目标没在急转）。
constexpr std::size_t kFireMinSamples = 4;

// 残差门限必须**按距离归一**，不能是绝对值。
//
// 拟合残差主要来自 PnP 的位置噪声，而那个噪声随距离增长（深度误差 ~ d²）：
// 实测 3.3 m 处残差中位 0.047 m，5.0 m 处 0.119 m。如果门限写死 0.06，
// 结果就是"远处的靶永远不许开火"——far 场景因此只打出 2 发（跟踪误差其实
// 只有 0.16°）。用比值：三处实测分别是 0.0145 / 0.0094 / 0.0239。
constexpr double kFireMaxResidualRatio = 0.03;

// 距离的合法区间。解出这个范围外的值基本是把两根不相关的灯条配对了。
constexpr double kMinDistance = 0.5;
constexpr double kMaxDistance = 20.0;

// PnP 重投影门限。超了说明这组点不可能是一块真的装甲板。
constexpr double kMaxReprojectionPx = 6.0;

// 侧看到什么程度就不要了。
//
// 矩形侧看会退化成一根线，深度就解不准了：translate 场景里那块侧板
// （入射角 60~80°）解出来的距离在 4.9~6.7 m 之间乱跳，而速度是距离的导数，
// 于是速度被估成 2 m/s 以上，提前量把云台甩出视野。
//
// 但门限不能卡太紧：自转靶上本来就该"板转到侧面就换下一块"，
// 卡在 0.4（66°）会让它每 0.3 s 就重启一次跟踪，速度永远估不出来。
// cos(76°) ≈ 0.25：只丢掉真正退化的，剩下的靠"优先锁最正对的那块"来解决。
constexpr double kMinCosIncidence = 0.25;

// 目标速度的物理上限。四块板装在半径 0.25 m 的车上、自转 4 rad/s，
// 切向也就 1 m/s；再加上平移，3.5 m/s 已经很宽了。
// 超过它说明跟踪已经坏了（多半是关联跳板或 PnP 崩了），这时宁可不要提前量。
constexpr double kMaxTargetSpeed = 6.0;

// 提前量迭代次数。飞行时间与目标位置互相依赖，三次足够收敛到毫米级。
constexpr int kLeadIterations = 3;

}  // namespace

Pipeline::Pipeline(PipelineConfig config)
    : config_(config), detector_(config.detector) {}

void Pipeline::reset() {
    tracker_.reset();
    last_stamp_ = -1.0;
    latency_ = -1.0;
    reference_distance_ = -1.0;
}

Detection Pipeline::detect(const cv::Mat& frame) const {
    return detector_.detect(frame);
}

TargetEstimate Pipeline::estimate(const Observation& observation,
                                  const Detection& detection) {
    TargetEstimate estimate;
    estimate.stamp = observation.sim_time;

    // 帧间隔 = 流水线延迟的代理量。用 EMA 平滑，因为单帧的间隔会抖。
    if (last_stamp_ >= 0.0) {
        const double interval = observation.sim_time - last_stamp_;
        if (interval > 0.0 && interval < 0.2) {
            latency_ = (latency_ < 0.0) ? interval : latency_ * 0.8 + interval * 0.2;
        }
    }
    last_stamp_ = observation.sim_time;
    const double latency = latency_ > 0.0 ? latency_ : 0.0;

    // 1) 每个候选都解成三维位姿（顺带拿到距离）。解不出来的直接扔掉，
    //    所以"候选下标"和"位姿下标"是两张表。
    std::vector<solver::ArmorPose> poses;
    std::vector<cv::Point3d> positions;
    std::vector<int> armor_index;
    for (std::size_t i = 0; i < detection.armors.size(); ++i) {
        const auto pose = solver::solveArmor(detection.armors[i].left(),
                                             detection.armors[i].right(),
                                             observation, reference_distance_);
        if (!pose) {
            continue;
        }
        if (pose->reprojection_px > kMaxReprojectionPx ||
            pose->distance < kMinDistance || pose->distance > kMaxDistance ||
            pose->cos_incidence < kMinCosIncidence) {
            continue;
        }
        poses.push_back(*pose);
        positions.push_back(pose->position_world);
        armor_index.push_back(static_cast<int>(i));
    }

    // 按"正对程度"排序：越正对的板 PnP 越准（深度越可观）。
    // tracker 在首次锁定和换板时取第 0 个，所以这样等于"优先锁最好的那块"——
    // 自转靶上就是"锁最正对的那块"，而不是随机锁一块然后被角度噪声拖死。
    std::vector<std::size_t> order(poses.size());
    for (std::size_t i = 0; i < order.size(); ++i) {
        order[i] = i;
    }
    std::stable_sort(order.begin(), order.end(),
                     [&poses](std::size_t a, std::size_t b) {
                         return poses[a].cos_incidence > poses[b].cos_incidence;
                     });

    const cv::Point3d origin(observation.gimbal_origin.x,
                             observation.gimbal_origin.y,
                             observation.gimbal_origin.z);

    // 2) 交给跟踪器做关联 + 拟合。没有候选时它也会被调用，
    //    这样它才能知道"这一帧什么都没看到"并自己计时。
    std::vector<cv::Point3d> sorted_positions;
    sorted_positions.reserve(order.size());
    for (std::size_t index : order) {
        sorted_positions.push_back(positions[index]);
    }
    const PlateTracker::Result track =
        tracker_.update(observation.sim_time, sorted_positions, origin);
    if (!track.valid) {
        return estimate;  // valid=false，下游据此停控
    }

    // 3) 速度的物理合理性。超过上限就当它没估出来：位置还能用，提前量不能用。
    cv::Point3d velocity = track.velocity;
    const double target_speed = cv::norm(velocity);
    const bool velocity_ok = std::isfinite(target_speed) && target_speed < kMaxTargetSpeed;
    if (!velocity_ok) {
        velocity = cv::Point3d(0.0, 0.0, 0.0);
    }

    // 4) 提前量：预测"弹丸到达时"目标在哪。
    //
    //    飞行时间依赖目标位置（距离），目标位置依赖飞行时间——解一个不动点。
    //    还要加上发射延迟：扣扳机到出膛这段时间云台早就定了，但目标是会动的。
    const std::size_t pose_index = order[static_cast<std::size_t>(track.index)];
    const solver::ArmorPose& measured = poses[pose_index];

    solver::AimSolution aim;
    cv::Point3d target_at = track.position;
    double flight = measured.distance / std::max(observation.bullet_speed, 1.0);
    for (int i = 0; i < kLeadIterations; ++i) {
        target_at = track.position +
                    velocity * (flight + observation.launch_delay + latency);
        const solver::AimSolution candidate = solver::solveAim(target_at, origin, observation);
        if (!candidate.ok) {
            // 够不着（disc<0）：退回直线角，至少方向是对的。
            aim = candidate;
            aim.yaw = std::atan2(target_at.y - origin.y, target_at.x - origin.x);
            aim.pitch = -std::atan2(target_at.z - origin.z,
                                    std::hypot(target_at.x - origin.x,
                                               target_at.y - origin.y));
            aim.flight_time = flight;
            break;
        }
        aim = candidate;
        flight = candidate.flight_time;
    }

    // 4) 目标角速度：把"位置速度"转成"角度速度"，给云台做前馈。
    //    云台伺服是 a = w²e + 2w(goal_v − v)，不填 goal_v 就会有一个
    //    稳态滞后 2·θ̇/ω，在 4 rad/s 的自转靶上约 0.8°——比开火窗口还大。
    const cv::Point3d delta = target_at - origin;
    const double horizontal = std::hypot(delta.x, delta.y);
    const double range2 = delta.x * delta.x + delta.y * delta.y;
    double yaw_rate = 0.0;
    double pitch_rate = 0.0;
    if (range2 > 1e-6) {
        yaw_rate = (delta.x * velocity.y - delta.y * velocity.x) / range2;
        const double horizontal_rate =
            (delta.x * velocity.x + delta.y * velocity.y) / std::max(horizontal, 1e-6);
        const double denominator = horizontal * horizontal + delta.z * delta.z;
        pitch_rate = -(velocity.z * horizontal - delta.z * horizontal_rate) /
                     std::max(denominator, 1e-6);
    }

    estimate.valid = true;
    estimate.armor_index = armor_index[pose_index];
    estimate.pixel = detection.armors[static_cast<std::size_t>(estimate.armor_index)].center();
    estimate.distance = measured.distance;
    estimate.yaw = aim.yaw;
    estimate.pitch = aim.pitch;
    estimate.yaw_rate = yaw_rate;
    estimate.pitch_rate = pitch_rate;
    estimate.flight_time = aim.flight_time;
    estimate.residual = track.residual;
    estimate.samples = track.samples;
    estimate.velocity_ok = velocity_ok;

    // 记下这一帧的距离，下一帧用它给"小/大装甲"做时间一致性约束。
    reference_distance_ = measured.distance;
    return estimate;
}

Command Pipeline::command(const Observation& observation,
                          const TargetEstimate& estimate) {
    Command command;
    if (!estimate.valid) {
        return command;  // control=false / fire=false
    }

    // 观测过期就别再控制了。注意这里用的是"最后一次成功观测"，不是
    // "最后一次算过"，否则丢帧时会一直按旧目标发指令。
    const double age = observation.sim_time - tracker_.last_measure_time();
    if (!(age >= 0.0) || age > kObservationTimeout) {
        return command;
    }

    command.control = true;
    command.yaw = estimate.yaw;
    command.pitch = estimate.pitch;
    command.yaw_vel = estimate.yaw_rate;
    command.pitch_vel = estimate.pitch_rate;

    // 开火判据，四条同时成立：
    //   1) 目标方向已经进窗口；
    //   2) 观测是新鲜的（不是几帧之前的位置）；
    //   3) 跟踪已经攒够帧数（速度估出来了，提前量才可信）；
    //   4) 拟合残差不大（目标没在急转，速度模型还成立）。
    const double error_yaw =
        std::abs(utils::wrapPi(estimate.yaw - observation.gimbal_yaw));
    const double error_pitch = std::abs(estimate.pitch - observation.gimbal_pitch);
    const double window = kFireWindowDeg * CV_PI / 180.0;
    const bool on_target = error_yaw < window && error_pitch < window;
    const bool fresh = age < 0.08;
    const double residual_limit = kFireMaxResidualRatio * std::max(estimate.distance, 1.0);
    const bool stable = estimate.residual < residual_limit &&
                        estimate.samples >= static_cast<int>(kFireMinSamples) &&
                        estimate.velocity_ok;
    command.fire = config_.allow_fire && on_target && fresh && stable;
    return command;
}
