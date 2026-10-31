// io/sim_capture.h
// ICapture 的仿真实现：把共享内存协议翻译成 Observation / Command。
//
// 这一层是"适配器"：协议里的字段名、单位、JSON 结构都不该漏到 pipeline 里。
// 以后接真机相机时，照着这个文件再写一个实现就行。
#ifndef ARMOR_IO_SIM_CAPTURE_H
#define ARMOR_IO_SIM_CAPTURE_H

#include <string>

#include <nlohmann/json.hpp>

#include "io/capture.h"
#include "io/sim_client.h"

class SimCapture : public ICapture {
public:
    explicit SimCapture(const std::string& channel);

    void pump() override;
    bool read(Observation& observation) override;
    bool controllable() const override { return true; }
    void write(const Command& command) override;
    bool finished() const override { return false; }
    std::string describe() const override;

    // 给主循环打印统计用：仿真侧的累计 fired/resolved/hit/hit_rate。
    const nlohmann::json& stats() const;
    bool enabled() const;
    bool paused() const;

private:
    sim::Client client_;
};

#endif  // ARMOR_IO_SIM_CAPTURE_H
