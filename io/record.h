// io/record.h
// 把采集到的帧落盘，供离线调试和回放。
//
// 为什么值得有：调检测器时你不想每次都开 Godot。
// 录 10 秒，然后对着同一批图跑上百次参数，结果还可复现。
#ifndef ARMOR_IO_RECORD_H
#define ARMOR_IO_RECORD_H

#include <fstream>
#include <string>

#include "core/types.h"

class FrameRecorder {
public:
    explicit FrameRecorder(const std::string& directory);
    ~FrameRecorder();

    FrameRecorder(const FrameRecorder&) = delete;
    FrameRecorder& operator=(const FrameRecorder&) = delete;

    void write(const Observation& observation);
    int count() const { return count_; }

private:
    std::string directory_;
    std::ofstream meta_;
    int count_ = 0;
};

#endif  // ARMOR_IO_RECORD_H
