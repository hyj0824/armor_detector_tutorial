// reference/pipeline_ref.cpp
#include "reference/pipeline_ref.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>

namespace {

// 关联门限：候选和预测位置在三维空间里差多少以内才认为"还是这一块板"。
// 四块板之间最近也有 0.2 m 以上，所以 0.22 m 足够区分。
constexpr double kAssociationGate = 0.22;

// 滑窗长度：用最近若干帧做最小二乘直线拟合，同时拿到"当前时刻的位置"和速度。
// 为什么要滑动拟合而不是简单的 alpha-beta：目标有加速度时，一阶滤波的位置和
// 速度会各带一份滞后，而提前量对速度误差极其敏感。
const std::size_t kWindowSize =
    static_cast<std::size_t>(std::getenv("ARMOR_REF_WIN") ? std::atoi(std::getenv("ARMOR_REF_WIN")) : 8);
const double kWindowSeconds =
    std::getenv("ARMOR_REF_WINSEC") ? std::atof(std::getenv("ARMOR_REF_WINSEC")) : 0.28;

// 超过这么久没有新观测，就认为目标丢了。
constexpr double kTrackTimeout = 0.25;

constexpr double kFireWindowDeg = 0.8;
constexpr double kFireMaxAge = 0.12;
constexpr std::size_t kFireMinSamples = 4;
constexpr double kFireMaxResidual = 0.06;  // 拟合残差超过这个值说明目标在急转弯

constexpr double kBulletSpeedFallback = 23.0;
constexpr double kGravityFallback = 9.7833;

}  // namespace

ReferencePipeline::ReferencePipeline(PipelineConfig config)
    : config_(config), detector_(config.detector) {}

void ReferencePipeline::reset() { history_.clear(); }

Detection ReferencePipeline::detect(const cv::Mat& frame) const {
    return detector_.detect(frame);
}

bool ReferencePipeline::fit(double now, cv::Point3d& position,
                           cv::Point3d& velocity, double& residual) const {
    if (history_.size() < 2) {
        return false;
    }
    double mean_t = 0.0;
    cv::Point3d mean_p(0.0, 0.0, 0.0);
    for (const auto& sample : history_) {
        mean_t += sample.time;
        mean_p += sample.position;
    }
    const double n = static_cast<double>(history_.size());
    mean_t /= n;
    mean_p *= 1.0 / n;

    double var_t = 0.0;
    cv::Point3d cov(0.0, 0.0, 0.0);
    for (const auto& sample : history_) {
        const double dt = sample.time - mean_t;
        var_t += dt * dt;
        cov += (sample.position - mean_p) * dt;
    }
    if (var_t < 1e-9) {
        return false;
    }
    velocity = cov * (1.0 / var_t);
    position = mean_p + velocity * (now - mean_t);

    residual = 0.0;
    for (const auto& sample : history_) {
        const cv::Point3d predicted = position + velocity * (sample.time - now);
        residual += cv::norm(sample.position - predicted);
    }
    residual /= n;
    return true;
}

TargetEstimate ReferencePipeline::estimate(const Observation& observation,
                                           const Detection& detection) {
    TargetEstimate estimate;
    estimate.stamp = observation.sim_time;

    // 1) 每块候选都解出三维位姿（顺带拿到距离）。
    std::vector<ref::ArmorPose> poses;
    std::vector<int> armor_index;
    for (std::size_t i = 0; i < detection.armors.size(); ++i) {
        const auto pose = ref::solveArmor(detection.armors[i].left(),
                                          detection.armors[i].right(), observation);
        if (!pose) {
            continue;
        }
        // 重投影超过 6 px 基本是把两个不相关的灯条配对了，直接扔掉。
        if (pose->reprojection_px > 6.0 || pose->distance < 0.3 || pose->distance > 30.0) {
            continue;
        }
        poses.push_back(*pose);
        armor_index.push_back(static_cast<int>(i));
    }
    if (poses.empty()) {
        return estimate;
    }

    // 2) 用历史拟合出的速度预测"现在应该在哪"，再用它做关联。
    cv::Point3d predicted(0.0, 0.0, 0.0);
    cv::Point3d velocity(0.0, 0.0, 0.0);
    double residual = 0.0;
    const bool has_fit = fit(observation.sim_time, predicted, velocity, residual);
    if (!has_fit && !history_.empty()) {
        predicted = history_.back().position;
    }

    int best = -1;
    double best_distance = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < poses.size(); ++i) {
        const double distance = cv::norm(poses[i].position_world - predicted);
        const double gate = has_fit ? kAssociationGate : std::numeric_limits<double>::max();
        if (distance < best_distance && distance <= gate) {
            best_distance = distance;
            best = static_cast<int>(i);
        }
    }

    if (best < 0) {
        // 这一帧没找到"同一块板"，保留历史继续等它转回来；
        // 太久没更新就清空，否则会在错误的位置上一直预测。
        if (!history_.empty() &&
            observation.sim_time - history_.back().time > kTrackTimeout) {
            history_.clear();
        }
        return estimate;
    }

    // 3) 把这一帧观测推进滑窗（滑窗只保留最近 kWindowSize 帧 / kWindowSeconds 秒）。
    const ref::ArmorPose& measured = poses[static_cast<std::size_t>(best)];
    const bool jumped = !history_.empty() &&
                        cv::norm(measured.position_world - history_.back().position) > 0.5;
    if (jumped) {
        history_.clear();  // 换了一块板，历史作废
    }
    history_.push_back({observation.sim_time, measured.position_world});
    while (history_.size() > kWindowSize) {
        history_.pop_front();
    }
    while (!history_.empty() &&
           history_.front().time < observation.sim_time - kWindowSeconds) {
        history_.pop_front();
    }
    last_measure_stamp_ = observation.sim_time;

    // 4) 提前量：预测"弹丸到达时"的位置，飞行时间和抬升角互相迭代。
    const cv::Point3d origin(observation.gimbal_origin.x, observation.gimbal_origin.y,
                             observation.gimbal_origin.z);
    cv::Point3d position_now = measured.position_world;
    cv::Point3d v(0.0, 0.0, 0.0);
    double fit_residual = 0.0;
    const bool fitted =
        fit(observation.sim_time, position_now, v, fit_residual) || history_.size() >= 2;
    if (!fitted) {
        position_now = measured.position_world;
        v = cv::Point3d(0.0, 0.0, 0.0);
    }

    double flight = 0.15;
    ref::AimSolution aim;
    cv::Point3d target_at = position_now;
    for (int i = 0; i < 3; ++i) {
        target_at = position_now + v * flight;
        aim = ref::solveAim(target_at, origin, kBulletSpeedFallback, kGravityFallback);
        flight = aim.flight_time;
    }

    // 5) 目标角速度：对 atan2 求导，给云台做前馈。
    const cv::Point3d delta = target_at - origin;
    const double horizontal = std::hypot(delta.x, delta.y);
    const double range2 = delta.x * delta.x + delta.y * delta.y;
    double yaw_rate = 0.0;
    double pitch_rate = 0.0;
    if (range2 > 1e-6) {
        yaw_rate = (delta.x * v.y - delta.y * v.x) / range2;
        const double horizontal_rate =
            (delta.x * v.x + delta.y * v.y) / std::max(horizontal, 1e-6);
        const double denom = horizontal * horizontal + delta.z * delta.z;
        pitch_rate = -(v.z * horizontal - delta.z * horizontal_rate) / std::max(denom, 1e-6);
    }

    estimate.valid = true;
    estimate.armor_index = armor_index[static_cast<std::size_t>(best)];
    estimate.pixel = detection.armors[static_cast<std::size_t>(estimate.armor_index)].center();
    estimate.distance = measured.distance;
    estimate.yaw = aim.yaw;
    estimate.pitch = aim.pitch;
    estimate.yaw_rate = yaw_rate;
    estimate.pitch_rate = pitch_rate;

    // 开火门控要用的"这次观测可不可信"，顺手存下来给 command() 用。
    stable_ = history_.size() >= kFireMinSamples &&
              (!fitted || fit_residual < kFireMaxResidual);
    return estimate;
}

Command ReferencePipeline::command(const Observation& observation,
                                   const TargetEstimate& estimate) const {
    Command command;
    if (!estimate.valid || last_measure_stamp_ < 0.0) {
        return command;
    }
    if (observation.sim_time - last_measure_stamp_ > kTrackTimeout) {
        return command;  // 目标过期，停控停火
    }

    command.control = true;
    command.yaw = estimate.yaw;
    command.pitch = estimate.pitch;
    command.yaw_vel = estimate.yaw_rate;
    command.pitch_vel = estimate.pitch_rate;

    const double error_yaw = std::abs(ref::wrapPi(estimate.yaw - observation.gimbal_yaw));
    const double error_pitch = std::abs(estimate.pitch - observation.gimbal_pitch);
    const double window = kFireWindowDeg * CV_PI / 180.0;
    command.fire = config_.allow_fire && stable_ && error_yaw < window &&
                   error_pitch < window &&
                   (observation.sim_time - last_measure_stamp_) < kFireMaxAge;
    return command;
}
