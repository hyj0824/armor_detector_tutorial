// io/sim_client.cpp
// 协议实现。改动这里的风险很高，除非你确实在改通信层，否则不要动。
//
// 三条不变式（rm_aim_sim/docs/protocol.md）：
//   1) 写文件必须先写临时文件再 rename，读的人只会看到完整的旧值或新值；
//   2) 状态要按 publication 去重，"又读到同一个文件"不等于"有新的反馈"；
//   3) 指令里的 publisher_id/session_id 必须来自当前这份新鲜状态。
#include "io/sim_client.h"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <regex>
#include <stdexcept>
#include <thread>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace sim {

namespace {

using Clock = std::chrono::steady_clock;

constexpr auto kStateTimeout = std::chrono::milliseconds(250);
constexpr auto kFreshWindow = std::chrono::milliseconds(200);
constexpr auto kFrameTimeout = std::chrono::seconds(5);

}  // namespace

Client::Client(const std::string& channel) {
    if (!std::regex_match(channel, std::regex("[A-Za-z0-9_-]{1,80}"))) {
        throw std::runtime_error("非法的 channel 名（只允许字母、数字、下划线、连字符）");
    }
    const std::string path = "/dev/shm/rm_aim_sim_" + channel;
    directory_ =
        ::open(path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
    if (directory_ < 0) {
        throw std::runtime_error(
            "找不到仿真通道 " + path +
            "（先启动 rm_aim_sim，并确认两边 channel 一致）");
    }
    struct stat info {};
    if (::fstat(directory_, &info) != 0 || info.st_uid != ::getuid() ||
        (info.st_mode & 0777) != 0700) {
        ::close(directory_);
        directory_ = -1;
        throw std::runtime_error("IPC 目录必须属于当前用户且权限为 0700");
    }
    lock_ = ::openat(directory_, "client.lock", O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (lock_ < 0 || ::flock(lock_, LOCK_EX | LOCK_NB) != 0) {
        if (lock_ >= 0) {
            ::close(lock_);
        }
        ::close(directory_);
        directory_ = -1;
        lock_ = -1;
        throw std::runtime_error(
            "client.lock 已被占用：同一个通道只能有一个控制端");
    }
    token_ = std::to_string(::getpid()) + "-" +
             std::to_string(Clock::now().time_since_epoch().count());
}

Client::~Client() {
    try {
        send(false, false, 0.0, 0.0);
    } catch (...) {
        // 析构里不要抛异常。
    }
    if (lock_ >= 0) {
        ::close(lock_);
    }
    if (directory_ >= 0) {
        ::close(directory_);
    }
}

bool Client::pollState() {
    nlohmann::json value = readJson("state.json");
    if (value.value("protocol", "") != "rm_aim_sim/2" ||
        value.value("type", "") != "state") {
        return false;
    }

    const double wall =
        std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    const double age = wall - value.at("wall_unix_sec").get<double>();
    if (!std::isfinite(age) || age < -0.1 || age > 0.25) {
        return false;  // 崩溃残留的旧状态，拒绝
    }

    if (!state_.empty() &&
        value.at("publisher_id") == state_.at("publisher_id") &&
        value.at("publication") == state_.at("publication")) {
        return false;  // 不是新发布
    }

    // 会话/标定/车型变了：手上的取帧请求作废。
    if (state_.empty() || context(value) != context(state_)) {
        request_.clear();
    }
    state_ = std::move(value);
    last_state_ = Clock::now();
    return true;
}

bool Client::fresh() const {
    return !state_.empty() && Clock::now() - last_state_ <= kFreshWindow;
}

bool Client::paused() const {
    return state_.empty() || state_.value("paused", true);
}

bool Client::enabled() const {
    return !state_.empty() && state_.at("ipc").value("enabled", false);
}

void Client::requestFrame() {
    if (!request_.empty() || !fresh()) {
        return;
    }
    request_ = token_ + "-" + std::to_string(request_count_++);
    requested_ = Clock::now();
    writeJson("request.json", {{"publisher_id", state_.at("publisher_id")},
                               {"request_id", request_}});
}

bool Client::tryFrame(FramePacket& packet) {
    if (request_.empty()) {
        return false;
    }
    if (Clock::now() - requested_ > kFrameTimeout) {
        throw std::runtime_error(
            "等图像超时：仿真必须是图形模式（--headless 没有像素输出）");
    }

    nlohmann::json frame = readJson("frame.json");
    if (frame.value("request_id", "") != request_) {
        return false;
    }
    if (!fresh() || frame.value("protocol", "") != "rm_aim_sim/2" ||
        frame.value("type", "") != "frame" ||
        context(frame) != context(state_)) {
        request_.clear();
        return false;
    }

    // 图像必须和当前状态足够接近，否则说明这不是同一时刻的东西。
    const double age =
        state_.at("sim_time").get<double>() - frame.at("sim_time").get<double>();
    if (!std::isfinite(age) || age > 0.2 || age < -0.1) {
        request_.clear();
        return false;
    }

    const auto& camera = frame.at("camera");
    const int width = camera.at("width").get<int>();
    const int height = camera.at("height").get<int>();
    const std::string encoding = frame.at("encoding").get<std::string>();
    const int channels = encoding == "rgb8" ? 3 : (encoding == "rgba8" ? 4 : 0);
    const std::string filename = "frame-" + request_ + ".raw";
    if (channels == 0 || width < 160 || width > 3840 || height < 120 ||
        height > 2160 || frame.at("frame_file") != filename) {
        throw std::runtime_error("帧元数据非法（尺寸/编码/文件名）");
    }
    const std::size_t bytes =
        static_cast<std::size_t>(width) * height * channels;
    if (frame.at("image_bytes").get<std::size_t>() != bytes) {
        throw std::runtime_error("image_bytes 与宽高编码不一致");
    }

    const int fd =
        ::openat(directory_, filename.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) {
        throw std::runtime_error("帧文件消失：仿真可能重启了，重新连一次");
    }
    cv::Mat rgb;
    try {
        struct stat info {};
        if (::fstat(fd, &info) != 0 || !S_ISREG(info.st_mode) ||
            info.st_size != static_cast<off_t>(bytes)) {
            throw std::runtime_error("帧文件大小或类型不对");
        }
        rgb.create(height, width, CV_MAKETYPE(CV_8U, channels));
        readAll(fd, rgb.data, bytes);
    } catch (...) {
        ::close(fd);
        throw;
    }
    ::close(fd);

    // 复制成自己的 BGR 存储后，才能发下一次请求（下一次请求即"上一帧已消费"）。
    cv::cvtColor(rgb, packet.bgr,
                 channels == 3 ? cv::COLOR_RGB2BGR : cv::COLOR_RGBA2BGR);
    packet.metadata = std::move(frame);
    request_.clear();
    return true;
}

void Client::send(bool control, bool fire, double yaw, double pitch,
                  double yaw_vel, double pitch_vel) {
    if (state_.empty()) {
        return;
    }
    for (double number : {yaw, pitch, yaw_vel, pitch_vel}) {
        if (!std::isfinite(number) || std::abs(number) > 1000.0) {
            throw std::runtime_error("指令里有非法数值");
        }
    }

    const std::string session = state_.at("session_id").get<std::string>();
    if (sequence_session_ != session) {
        sequence_session_ = session;
        sequence_ = 0;
    }
    sequence_ = std::max(sequence_, state_.at("ipc").at("last_seq").get<std::int64_t>() + 1);
    if (sequence_ > 9007199254740991LL) {
        throw std::runtime_error("seq 用尽了");
    }

    // 安全降级：不新鲜 / 暂停 / 未开启外部控制，一律不许控制，更不许开火。
    control = control && fresh() && !paused() && enabled();
    if (!control) {
        fire = false;
    }

    writeJson("command.json",
              {{"protocol", "rm_aim_sim/2"},
               {"type", "command"},
               {"publisher_id", state_.at("publisher_id")},
               {"session_id", state_.at("session_id")},
               {"seq", sequence_++},
               {"control", control},
               {"fire", fire},
               {"yaw", yaw},
               {"pitch", pitch},
               {"yaw_vel", yaw_vel},
               {"pitch_vel", pitch_vel},
               {"yaw_acc", 0.0},
               {"pitch_acc", 0.0}});
}

nlohmann::json Client::context(const nlohmann::json& packet) {
    const auto& camera = packet.at("camera");
    return nlohmann::json::array(
        {packet.at("publisher_id"), packet.at("session_id"),
         packet.at("appearance"), packet.at("paused"),
         packet.at("ipc").at("enabled"), camera.at("width"),
         camera.at("height"), camera.at("K"), camera.at("distortion"),
         camera.at("R_camera2gimbal"), camera.at("t_camera2gimbal")});
}

void Client::readAll(int fd, void* output, std::size_t bytes) {
    auto* data = static_cast<char*>(output);
    while (bytes > 0) {
        const ssize_t count = ::read(fd, data, bytes);
        if (count < 0 && errno == EINTR) {
            continue;
        }
        if (count <= 0) {
            throw std::runtime_error("文件读到一半断了");
        }
        data += count;
        bytes -= static_cast<std::size_t>(count);
    }
}

nlohmann::json Client::readJson(const char* filename) const {
    const int fd = ::openat(directory_, filename, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) {
        return nlohmann::json::object();
    }
    std::string data;
    try {
        struct stat info {};
        if (::fstat(fd, &info) != 0 || !S_ISREG(info.st_mode) ||
            info.st_size <= 0 || info.st_size > 65536) {
            throw std::runtime_error(std::string("JSON 文件大小或类型不对: ") + filename);
        }
        data.resize(static_cast<std::size_t>(info.st_size));
        readAll(fd, data.data(), data.size());
    } catch (...) {
        ::close(fd);
        throw;
    }
    ::close(fd);

    nlohmann::json value = nlohmann::json::parse(data, nullptr, false);
    return value.is_object() ? value : nlohmann::json::object();
}

void Client::writeJson(const char* filename, const nlohmann::json& value) const {
    const std::string temporary = std::string(filename) + ".client.tmp";
    const int fd = ::openat(directory_, temporary.c_str(),
                            O_CREAT | O_TRUNC | O_WRONLY | O_CLOEXEC | O_NOFOLLOW,
                            0600);
    if (fd < 0) {
        throw std::runtime_error(std::string("无法创建临时文件: ") + temporary);
    }
    try {
        const std::string data = value.dump();
        std::size_t written = 0;
        while (written < data.size()) {
            const ssize_t count = ::write(fd, data.data() + written, data.size() - written);
            if (count < 0 && errno == EINTR) {
                continue;
            }
            if (count <= 0) {
                throw std::runtime_error("写临时文件失败");
            }
            written += static_cast<std::size_t>(count);
        }
    } catch (...) {
        ::close(fd);
        ::unlinkat(directory_, temporary.c_str(), 0);
        throw;
    }
    ::close(fd);
    if (::renameat(directory_, temporary.c_str(), directory_, filename) != 0) {
        throw std::runtime_error("原子发布失败");
    }
}

}  // namespace sim
