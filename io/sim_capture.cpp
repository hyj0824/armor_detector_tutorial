// io/sim_capture.cpp
#include "io/sim_capture.h"

#include <cstddef>
#include <stdexcept>
#include <vector>

namespace {

// frame.json 里的 camera.K 是 3x3 行优先的 9 个数，这里直接拆成标量。
// 不做矩阵抽象，因为 v0 只用到 fx/fy/cx/cy；等你需要完整外参时再升级。
void fillIntrinsics(const nlohmann::json& camera, Observation& observation) {
    const std::vector<double> k = camera.at("K").get<std::vector<double>>();
    if (k.size() != 9) {
        throw std::runtime_error("camera.K 不是 3x3");
    }
    observation.fx = k[0];
    observation.cx = k[2];
    observation.fy = k[4];
    observation.cy = k[5];
}

// weapon 段在 state.json 里，不在 frame.json 里，所以从 client 的最新状态取。
// 取不到就保留 Observation 里的默认值（和 configs/default.cfg 一致）。
void fillWeapon(const nlohmann::json& state, Observation& observation) {
    if (!state.is_object() || !state.contains("weapon")) {
        return;
    }
    const auto& weapon = state.at("weapon");
    observation.bullet_speed = weapon.value("speed", observation.bullet_speed);
    observation.gravity = weapon.value("gravity", observation.gravity);
    observation.launch_delay = weapon.value("delay", observation.launch_delay);
    observation.barrel_length = weapon.value("barrel_length", observation.barrel_length);
}

// camera 段里有相机相对云台的安装偏移。丢掉它就会有一个恒定的角度偏差：
// 相机比旋转中心高 4 cm，也就是 5 m 处约 0.46°——而 5 m 处的纵向容差只有 0.68°。
void fillCameraOffset(const nlohmann::json& camera, Observation& observation) {
    const auto offset = camera.find("t_camera2gimbal");
    if (offset == camera.end() || !offset->is_array() || offset->size() != 3) {
        return;
    }
    observation.camera_offset =
        cv::Point3d((*offset)[0].get<double>(), (*offset)[1].get<double>(),
                    (*offset)[2].get<double>());
}

}  // namespace

SimCapture::SimCapture(const std::string& channel) : client_(channel) {}

void SimCapture::pump() {
    client_.pollState();
    client_.requestFrame();
}

bool SimCapture::read(Observation& observation) {
    sim::FramePacket packet;
    if (!client_.tryFrame(packet)) {
        return false;
    }

    const auto& meta = packet.metadata;
    const auto& gimbal = meta.at("gimbal");

    observation.bgr = std::move(packet.bgr);
    observation.sim_time = meta.at("sim_time").get<double>();
    observation.session_id = meta.value("session_id", std::string{});
    fillIntrinsics(meta.at("camera"), observation);
    fillCameraOffset(meta.at("camera"), observation);
    fillWeapon(client_.state(), observation);

    // 【关键】这里是"拍摄这一帧时"的云台姿态，不是最新状态。
    observation.gimbal_yaw = gimbal.at("yaw").get<double>();
    observation.gimbal_pitch = gimbal.at("pitch").get<double>();
    observation.gimbal_yaw_vel = gimbal.value("yaw_vel", 0.0);
    observation.gimbal_pitch_vel = gimbal.value("pitch_vel", 0.0);
    if (const auto origin = gimbal.find("origin");
        origin != gimbal.end() && origin->is_array() && origin->size() == 3) {
        observation.gimbal_origin = cv::Point3d((*origin)[0].get<double>(),
                                                (*origin)[1].get<double>(),
                                                (*origin)[2].get<double>());
    }
    return true;
}

void SimCapture::write(const Command& command) {
    client_.send(command.control, command.fire, command.yaw, command.pitch,
                 command.yaw_vel, command.pitch_vel);
}

const nlohmann::json& SimCapture::stats() const {
    static const nlohmann::json kEmpty = nlohmann::json::object();
    if (client_.state().empty() || !client_.state().contains("stats")) {
        return kEmpty;
    }
    return client_.state().at("stats");
}

bool SimCapture::enabled() const {
    return client_.enabled();
}

bool SimCapture::paused() const {
    return client_.paused();
}

std::string SimCapture::describe() const {
    if (client_.state().empty()) {
        return "sim(未连接)";
    }
    const auto& camera = client_.state().at("camera");
    return "sim " + std::to_string(camera.at("width").get<int>()) + "x" +
           std::to_string(camera.at("height").get<int>()) +
           (client_.enabled() ? " [外部控制已开启]" : " [外部控制未开启]") +
           (client_.paused() ? " [已暂停]" : "");
}
