// reference/pipeline_ref.h
// 参考实现：把 v0 缺的四件事（距离 / 关联 / 提前量 / 生命周期）都补上。
//
// 目的不是"给学生的答案"，而是**标定阈值**：
// 先证明每个场景能做到多少，再决定及格线和优秀线画在哪。
//
// 设计上没有追求最优，只追求"一个认真做了两周的学生大概能到的高度"：
//   * 距离：平面 PnP，小/大装甲两种间距取重投影误差小的
//   * 关联：世界系三维位置 + 匀速模型 + alpha-beta 滤波 + 距离门限
//   * 提前量：飞行时间与抬升角互相迭代 5 次
//   * 生命周期：观测超过 250 ms 没有更新就停控停火
#ifndef ARMOR_REFERENCE_PIPELINE_REF_H
#define ARMOR_REFERENCE_PIPELINE_REF_H

#include <deque>
#include <vector>

#include "core/types.h"
#include "pipeline/pipeline.h"
#include "reference/solver_ref.h"
#include "vision/detector.h"

class ReferencePipeline {
public:
    explicit ReferencePipeline(PipelineConfig config);

    const ArmorDetector& detector() const { return detector_; }

    Detection detect(const cv::Mat& frame) const;
    TargetEstimate estimate(const Observation& observation, const Detection& detection);
    Command command(const Observation& observation, const TargetEstimate& estimate) const;

    void reset();

private:
    // 一颗被跟踪的装甲板：世界系位置的历史，用滑动最小二乘同时得到
    // "当前时刻的位置"和速度。
    struct Sample {
        double time = 0.0;
        cv::Point3d position{0.0, 0.0, 0.0};
    };

    bool fit(double now, cv::Point3d& position, cv::Point3d& velocity,
             double& residual) const;

    PipelineConfig config_;
    ArmorDetector detector_;
    std::deque<Sample> history_;
    double last_measure_stamp_ = -1.0;
    bool stable_ = false;
};

#endif  // ARMOR_REFERENCE_PIPELINE_REF_H
