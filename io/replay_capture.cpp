// io/replay_capture.cpp
#include "io/replay_capture.h"

#include <filesystem>
#include <fstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>

namespace fs = std::filesystem;

ReplayCapture::ReplayCapture(const std::string& directory) : directory_(directory) {
    const fs::path meta_path = fs::path(directory_) / "meta.jsonl";
    std::ifstream meta(meta_path.string());
    if (!meta) {
        throw std::runtime_error("回放目录里没有 meta.jsonl: " + directory_);
    }
    std::string line;
    while (std::getline(meta, line)) {
        if (line.empty()) {
            continue;
        }
        nlohmann::json value = nlohmann::json::parse(line, nullptr, false);
        if (value.is_object()) {
            meta_.push_back(std::move(value));
        }
    }
    if (meta_.empty()) {
        throw std::runtime_error("回放目录里没有有效帧: " + directory_);
    }
}

bool ReplayCapture::read(Observation& observation) {
    if (index_ >= meta_.size()) {
        return false;
    }
    const auto& line = meta_[index_++];

    const std::string path =
        (fs::path(directory_) / line.at("file").get<std::string>()).string();
    observation.bgr = cv::imread(path);
    if (observation.bgr.empty()) {
        throw std::runtime_error("读不到回放图片: " + path);
    }
    observation.sim_time = line.at("sim_time").get<double>();
    observation.fx = line.at("fx").get<double>();
    observation.fy = line.at("fy").get<double>();
    observation.cx = line.at("cx").get<double>();
    observation.cy = line.at("cy").get<double>();
    observation.session_id = line.value("session_id", std::string{});

    const auto& gimbal = line.at("gimbal");
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
    // 旧录制文件没有这两段，保留 Observation 的默认值。
    if (const auto offset = line.find("camera_offset");
        offset != line.end() && offset->is_array() && offset->size() == 3) {
        observation.camera_offset = cv::Point3d((*offset)[0].get<double>(),
                                                 (*offset)[1].get<double>(),
                                                 (*offset)[2].get<double>());
    }
    if (const auto weapon = line.find("weapon");
        weapon != line.end() && weapon->is_object()) {
        observation.bullet_speed = weapon->value("speed", observation.bullet_speed);
        observation.gravity = weapon->value("gravity", observation.gravity);
        observation.launch_delay = weapon->value("delay", observation.launch_delay);
        observation.barrel_length =
            weapon->value("barrel_length", observation.barrel_length);
    }
    return true;
}

bool ReplayCapture::finished() const {
    return index_ >= meta_.size();
}

std::string ReplayCapture::describe() const {
    return "replay " + directory_ + " (" + std::to_string(meta_.size()) + " 帧)";
}
