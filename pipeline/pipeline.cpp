// pipeline/pipeline.cpp
#include "pipeline/pipeline.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "vision/angles.h"
#include "vision/utils.h"

namespace {

// 固定的抬升角补偿重力下坠。
//
// 这是个【占位实现】，不是正确答案：
//   弹丸 23 m/s、重力 9.78 m/s^2，在 4 m 处飞行约 0.17 s，下坠约 0.15 m，
//   对应的抬升角约 2.1 度。所以这个常数只在靶距 2~5 m 附近有效。
//   靶距变成 7 m 时下坠约 0.45 m，需要约 3.6 度——用 2.1 度就会打到靶子下方。
//
// 真正的做法是先知道距离（PnP）、再算飞行时间、再算抬升角，
// 而且飞行时间又依赖抬升角，需要迭代。那件事留给你。
constexpr double kFixedElevationDeg = 2.10;

// 开火窗口：目标和炮口方向的角度差在这个范围内才开火。
constexpr double kFireWindowDeg = 0.6;

}  // namespace

Pipeline::Pipeline(PipelineConfig config)
    : config_(config), detector_(config.detector) {}

Detection Pipeline::detect(const cv::Mat& frame) const {
    return detector_.detect(frame);
}

TargetEstimate Pipeline::estimate(const Observation& observation,
                                  const Detection& detection) const {
    TargetEstimate estimate;
    estimate.stamp = observation.sim_time;

    if (detection.armors.empty()) {
        return estimate;
    }

    // v0 的目标关联：每帧独立地选"离画面中心最近的那块板"。
    // 没有历史、没有速度、没有换板惩罚——所以旋转靶会在四块板之间跳。
    const cv::Point2f image_center(static_cast<float>(observation.cx),
                                   static_cast<float>(observation.cy));
    double best_distance = std::numeric_limits<double>::max();
    int best_index = -1;
    for (std::size_t i = 0; i < detection.armors.size(); ++i) {
        const double distance =
            cv::norm(detection.armors[i].center() - image_center);
        if (distance < best_distance) {
            best_distance = distance;
            best_index = static_cast<int>(i);
        }
    }

    const Armor& armor = detection.armors[static_cast<std::size_t>(best_index)];
    const RelativeAngle relative =
        relativeAngleFromPixel(armor.center(), observation.fx, observation.fy,
                               observation.cx, observation.cy);

    estimate.valid = true;
    estimate.armor_index = best_index;
    estimate.pixel = armor.center();
    estimate.distance = 0.0;  // 不知道距离 —— 这是 v0 最大的缺口
    estimate.yaw = utils::wrapPi(observation.gimbal_yaw + relative.yaw);
    estimate.pitch = observation.gimbal_pitch + relative.pitch -
                     kFixedElevationDeg * CV_PI / 180.0;
    estimate.yaw_rate = 0.0;
    estimate.pitch_rate = 0.0;
    return estimate;
}

Command Pipeline::command(const Observation& observation,
                          const TargetEstimate& estimate) const {
    Command command;
    if (!estimate.valid) {
        return command;  // control=false / fire=false，云台会按限加速度停下
    }

    command.control = true;
    command.yaw = estimate.yaw;
    command.pitch = estimate.pitch;

    // 开火判据：目标方向已经进入窗口。没有延迟补偿、没有连续帧确认、
    // 也没有考虑目标速度——所以移动靶基本打不中，这是预期现象。
    const double error_yaw =
        std::abs(utils::wrapPi(estimate.yaw - observation.gimbal_yaw));
    const double error_pitch = std::abs(estimate.pitch - observation.gimbal_pitch);
    const double window = kFireWindowDeg * CV_PI / 180.0;
    command.fire = config_.allow_fire && error_yaw < window &&
                   error_pitch < window;
    return command;
}
