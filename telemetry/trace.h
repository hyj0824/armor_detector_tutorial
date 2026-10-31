// telemetry/trace.h
// 逐帧写一行 JSON，供离线画图与归因。
//
// 这是整套东西里最值得先做好的部分：没有 trace，"云台越推越远"这种症状
// 学生根本看不见；有了 trace，一条曲线就能把问题指到某一层。
#ifndef ARMOR_TELEMETRY_TRACE_H
#define ARMOR_TELEMETRY_TRACE_H

#include <fstream>
#include <string>

#include "core/types.h"
#include "pipeline/pipeline.h"

class Trace {
public:
    explicit Trace(const std::string& path);
    ~Trace();

    Trace(const Trace&) = delete;
    Trace& operator=(const Trace&) = delete;

    bool enabled() const { return out_.is_open(); }

    void frame(const Observation& observation, const Detection& detection,
               const TargetEstimate& estimate, const Command& command);

private:
    std::ofstream out_;
};

#endif  // ARMOR_TELEMETRY_TRACE_H
