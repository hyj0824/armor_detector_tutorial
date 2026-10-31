// io/record.cpp
#include "io/record.h"

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>

namespace fs = std::filesystem;

FrameRecorder::FrameRecorder(const std::string& directory) : directory_(directory) {
    std::error_code error;
    fs::create_directories(directory_, error);
    if (error) {
        throw std::runtime_error("无法创建录制目录: " + directory_);
    }
    meta_.open((fs::path(directory_) / "meta.jsonl").string(),
               std::ios::out | std::ios::trunc);
    if (!meta_) {
        throw std::runtime_error("无法写 meta.jsonl");
    }
}

FrameRecorder::~FrameRecorder() = default;

void FrameRecorder::write(const Observation& observation) {
    std::ostringstream name;
    name << "frame_" << std::setw(6) << std::setfill('0') << count_ << ".png";
    const std::string path = (fs::path(directory_) / name.str()).string();
    if (!cv::imwrite(path, observation.bgr)) {
        throw std::runtime_error("写图片失败: " + path);
    }

    nlohmann::json line;
    line["index"] = count_;
    line["file"] = name.str();
    line["sim_time"] = observation.sim_time;
    line["fx"] = observation.fx;
    line["fy"] = observation.fy;
    line["cx"] = observation.cx;
    line["cy"] = observation.cy;
    line["session_id"] = observation.session_id;
    // 相机安装偏移和弹道参数也要落盘，否则回放时会静默用默认值，
    // 和录制时的仿真配置对不上。
    line["camera_offset"] = {observation.camera_offset.x,
                             observation.camera_offset.y,
                             observation.camera_offset.z};
    line["weapon"] = {{"speed", observation.bullet_speed},
                      {"gravity", observation.gravity},
                      {"delay", observation.launch_delay},
                      {"barrel_length", observation.barrel_length}};
    line["gimbal"] = {{"yaw", observation.gimbal_yaw},
                      {"pitch", observation.gimbal_pitch},
                      {"yaw_vel", observation.gimbal_yaw_vel},
                      {"pitch_vel", observation.gimbal_pitch_vel},
                      {"origin", {observation.gimbal_origin.x,
                                  observation.gimbal_origin.y,
                                  observation.gimbal_origin.z}}};
    meta_ << line.dump() << '\n';
    meta_.flush();
    ++count_;
}
