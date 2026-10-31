// io/capture.h
// 采集层的抽象。仿真、回放都实现这个接口。
//
// 为什么要有这层：主循环只应该关心"拿到一帧 / 交出一条指令"，
// 不该关心数据是来自 Godot 的共享内存还是磁盘上的录制。
// 以后要换视觉源（真机相机、录像文件），新增一个实现即可，pipeline 不用改。
#ifndef ARMOR_IO_CAPTURE_H
#define ARMOR_IO_CAPTURE_H

#include <string>

#include "core/types.h"

class ICapture {
public:
    virtual ~ICapture() = default;

    // 非阻塞：刷新反馈状态、推进取帧请求。每圈都要调。
    virtual void pump() = 0;

    // 有完整的一帧就填进 observation 并返回 true；否则返回 false（不是错误）。
    virtual bool read(Observation& observation) = 0;

    // 能不能下发指令。回放时为 false。
    virtual bool controllable() const = 0;

    // 下发一条指令。不可控时静默忽略。
    virtual void write(const Command& command) = 0;

    // 数据源结束（回放读完）。
    virtual bool finished() const = 0;

    virtual std::string describe() const = 0;
};

#endif  // ARMOR_IO_CAPTURE_H
