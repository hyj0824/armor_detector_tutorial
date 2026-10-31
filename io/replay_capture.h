// io/replay_capture.h
// ICapture 的离线实现：读 FrameRecorder 录下来的目录。
// 不能控制（write 是空实现），只是让你在没开仿真的情况下反复跑同一批帧。
#ifndef ARMOR_IO_REPLAY_CAPTURE_H
#define ARMOR_IO_REPLAY_CAPTURE_H

#include <cstddef>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "io/capture.h"

class ReplayCapture : public ICapture {
public:
    explicit ReplayCapture(const std::string& directory);

    void pump() override {}
    bool read(Observation& observation) override;
    bool controllable() const override { return false; }
    void write(const Command&) override {}
    bool finished() const override;
    std::string describe() const override;

private:
    std::string directory_;
    std::vector<nlohmann::json> meta_;
    std::size_t index_ = 0;
};

#endif  // ARMOR_IO_REPLAY_CAPTURE_H
