// io/sim_client.h
// rm_aim_sim 的本机共享内存客户端（rm_aim_sim/2 协议）。
//
// 这是唯一依赖 nlohmann::json 的文件。协议细节见
//   rm_aim_sim/docs/protocol.md
// 这里只做三件事：读最新状态、请求一帧图像、下发一条指令。
//
// 注意：这个类不是线程安全的，单线程用。
#ifndef ARMOR_IO_SIM_CLIENT_H
#define ARMOR_IO_SIM_CLIENT_H

#include <chrono>
#include <cstdint>
#include <string>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

namespace sim {

struct FramePacket {
    cv::Mat bgr;             // 已转成 BGR 的独立图像
    nlohmann::json metadata; // frame.json 原文（同帧的时间、姿态、标定）
};

class Client {
public:
    explicit Client(const std::string& channel);
    ~Client();

    Client(const Client&) = delete;
    Client& operator=(const Client&) = delete;

    // 读到一版更新的状态就返回 true。重复的 publication 不算新。
    bool pollState();

    // 最近一次新状态距今不超过 200 ms。
    bool fresh() const;

    bool paused() const;
    bool enabled() const;  // 仿真端是否打开了"本机自瞄控制"

    const nlohmann::json& state() const { return state_; }

    // 没有未完成的请求、且状态新鲜时才真的发请求。
    void requestFrame();

    // 拿到与请求匹配的一帧就返回 true。超过 5 s 没响应抛异常。
    bool tryFrame(FramePacket& packet);

    // 下发绝对目标角。control 会被就地降级：
    // 状态不新鲜 / 暂停 / 未开启外部控制 时强制为 false。
    void send(bool control, bool fire, double yaw, double pitch,
              double yaw_vel = 0.0, double pitch_vel = 0.0);

private:
    int directory_ = -1;
    int lock_ = -1;

    nlohmann::json state_ = nlohmann::json::object();
    std::string request_;
    std::string token_;
    std::string sequence_session_;
    std::int64_t sequence_ = 0;
    std::uint64_t request_count_ = 0;

    std::chrono::steady_clock::time_point last_state_;
    std::chrono::steady_clock::time_point requested_;

    static nlohmann::json context(const nlohmann::json& packet);
    static void readAll(int fd, void* output, std::size_t bytes);

    nlohmann::json readJson(const char* filename) const;
    void writeJson(const char* filename, const nlohmann::json& value) const;
};

}  // namespace sim

#endif  // ARMOR_IO_SIM_CLIENT_H
