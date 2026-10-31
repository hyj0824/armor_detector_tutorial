// pipeline/pipeline.h
// 把检测结果变成一条云台指令。
//
//   所有候选 --(PnP 解三维)--> 世界系位置 --(跟踪)--> 位置+速度
//            --(提前量+弹道)--> 目标角 --(开火判据)--> Command
//
// 链条比 v0 长，但每一段都能单独离线验证：solver 是纯几何，
// tracker 是纯数值，command 只有判据。所以改哪一段都不用开 Godot。
#ifndef ARMOR_PIPELINE_PIPELINE_H
#define ARMOR_PIPELINE_PIPELINE_H

#include <cstddef>

#include "core/types.h"
#include "pipeline/tracker.h"
#include "vision/detector.h"

struct TargetEstimate {
    bool valid = false;
    int armor_index = -1;      // 选中了 detection.armors 里的哪一个
    cv::Point2f pixel{0.0f, 0.0f};  // 瞄准点（图像坐标，只用于画图）

    double distance = 0.0;     // 到云台旋转中心的距离，单位 m
    double yaw = 0.0;          // 绝对目标角（含云台当前角），向左为正
    double pitch = 0.0;        // 绝对目标角（含云台当前角），向下为正
    double yaw_rate = 0.0;     // 目标角速度，rad/s
    double pitch_rate = 0.0;   // 同上
    double stamp = 0.0;        // 这份估计对应的时间（仿真秒）

    // 下面几项是内部诊断量，不参与控制。trace 会录下来，调参时看它。
    double flight_time = 0.0;  // 出膛到命中的飞行时间
    double residual = 0.0;     // 跟踪拟合残差（米），大说明目标在急转
    int samples = 0;           // 跟踪窗口里有几帧观测
    bool velocity_ok = false;  // 速度估计是否通过物理合理性检查
};

struct PipelineConfig {
    DetectorConfig detector;
    bool allow_fire = false;
};

class Pipeline {
public:
    explicit Pipeline(PipelineConfig config);

    const ArmorDetector& detector() const { return detector_; }

    // detect 是纯函数；estimate / command 带状态（要历史帧才能估速度）。
    Detection detect(const cv::Mat& frame) const;
    TargetEstimate estimate(const Observation& observation,
                            const Detection& detection);
    Command command(const Observation& observation,
                    const TargetEstimate& estimate);

    // 换会话 / 目标彻底丢了 / 仿真重启时把手上的历史清掉。
    void reset();

private:
    PipelineConfig config_;
    ArmorDetector detector_;
    PlateTracker tracker_;

    // 提前量里除了飞行时间和发射延迟，还要补上"这一帧从拍到到发指令"的延迟：
    // 观测已经是旧的了，而目标一直在跑。用相邻帧间隔估它（EMA 平滑）。
    double last_stamp_ = -1.0;
    double latency_ = -1.0;

    // 上一帧解出来的距离。小装甲(0.135)和大装甲(0.230)在 5 m 处的透视
    // 差异是亚像素级的，重投影误差区分不开，所以用时间一致性来选。
    double reference_distance_ = -1.0;
};

#endif  // ARMOR_PIPELINE_PIPELINE_H
