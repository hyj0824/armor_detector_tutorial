// pipeline/pipeline.h
// 把检测结果变成一条云台指令。
//
// v0 的链条只有三段，故意很短：
//
//   所有候选 --(挑一个)--> 选中板 --(像素->角度)--> 目标角 --(开火判据)--> Command
//
// 缺口就在这里，而且是"接口留空"而不是"TODO 注释"：
//   * TargetEstimate::distance 恒为 0  -> 没有弹道补偿，没有提前量
//   * TargetEstimate::yaw_rate/pitch_rate 恒为 0 -> 没有预测
//   * 挑选只用"离画面中心最近"，没有时序 -> 旋转靶会跳板
//
// 填这些字段是你的工作。填之前先想清楚它需要什么信息（比如距离从哪来）。
#ifndef ARMOR_PIPELINE_PIPELINE_H
#define ARMOR_PIPELINE_PIPELINE_H

#include <cstddef>

#include "core/types.h"
#include "vision/detector.h"

// 当前对目标的估计。下游（控制器/开火判据）只认这个结构体，
// 所以你可以随便换内部算法，只要把字段填对。
struct TargetEstimate {
    bool valid = false;
    int armor_index = -1;      // 选中了 detection.armors 里的哪一个
    cv::Point2f pixel{0.0f, 0.0f};  // 瞄准点（图像坐标）

    double distance = 0.0;     // 到云台的距离，单位 m。v0：不知道，恒为 0
    double yaw = 0.0;          // 绝对目标角（含云台当前角），向左为正
    double pitch = 0.0;        // 绝对目标角（含云台当前角），向下为正
    double yaw_rate = 0.0;     // 目标角速度，rad/s。v0：恒为 0
    double pitch_rate = 0.0;   // 同上
    double stamp = 0.0;        // 这份估计对应的时间（仿真秒）
};

struct PipelineConfig {
    DetectorConfig detector;
    bool allow_fire = false;
};

class Pipeline {
public:
    explicit Pipeline(PipelineConfig config);

    const ArmorDetector& detector() const { return detector_; }

    // 纯函数三连：不碰文件、不碰时钟、不碰仿真，输入什么就输出什么。
    // 这样每个阶段都能离线单测，也不用为了改检测器去开 Godot。
    Detection detect(const cv::Mat& frame) const;
    TargetEstimate estimate(const Observation& observation,
                            const Detection& detection) const;
    Command command(const Observation& observation,
                    const TargetEstimate& estimate) const;

private:
    PipelineConfig config_;
    ArmorDetector detector_;
};

#endif  // ARMOR_PIPELINE_PIPELINE_H
