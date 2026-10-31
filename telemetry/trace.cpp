// telemetry/trace.cpp
#include "telemetry/trace.h"

#include <stdexcept>

#include <nlohmann/json.hpp>

#include "vision/utils.h"

Trace::Trace(const std::string& path) {
    if (path.empty()) {
        return;
    }
    out_.open(path, std::ios::out | std::ios::trunc);
    if (!out_) {
        throw std::runtime_error("无法写 trace 文件: " + path);
    }
}

Trace::~Trace() = default;

void Trace::frame(const Observation& observation, const Detection& detection,
                  const TargetEstimate& estimate, const Command& command) {
    if (!enabled()) {
        return;
    }

    nlohmann::json line;
    line["t"] = observation.sim_time;

    // 检测阶段：能看到"灯条几个、候选几个"就已经能定位一半问题。
    line["det"] = {{"bars", detection.light_bars.size()},
                   {"armors", detection.armors.size()},
                   {"ms", detection.detect_ms}};

    // 估计阶段：v0 里 distance / rate 恒为 0，正好是留给你的两个空位。
    line["est"] = {{"valid", estimate.valid},
                   {"idx", estimate.armor_index},
                   {"px", estimate.pixel.x},
                   {"py", estimate.pixel.y},
                   {"yaw", estimate.yaw},
                   {"pitch", estimate.pitch},
                   {"dist", estimate.distance},
                   {"yaw_rate", estimate.yaw_rate},
                   {"pitch_rate", estimate.pitch_rate}};

    // 控制阶段：指令角、开火位，以及【指令 - 当帧云台实际角】的偏差。
    line["cmd"] = {{"control", command.control},
                   {"fire", command.fire},
                   {"yaw", command.yaw},
                   {"pitch", command.pitch}};

    line["gim"] = {{"yaw", observation.gimbal_yaw},
                   {"pitch", observation.gimbal_pitch}};

    const double error_yaw =
        utils::wrapPi(estimate.yaw - observation.gimbal_yaw);
    const double error_pitch = estimate.pitch - observation.gimbal_pitch;
    line["err"] = {{"yaw", error_yaw}, {"pitch", error_pitch}};

    out_ << line.dump() << '\n';
}
