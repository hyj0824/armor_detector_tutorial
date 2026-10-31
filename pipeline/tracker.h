// pipeline/tracker.h
// 跟踪"一块装甲板"在三维世界里的运动。
//
// 为什么要跟踪而不是每帧重新挑：
//   * 提前量要速度，速度只能从历史帧来；
//   * 自转靶上四块板一直在抢"离画面中心最近"，每帧重新挑会让指令抖，
//     而云台是个二阶系统，抖一次就要 0.2 s 才稳。
//
// 全程在世界系里做（而不是图像系）：云台自己在动，图像坐标里看目标
// 的速度不干净；世界系里目标怎么动就是怎么动。
#ifndef ARMOR_PIPELINE_TRACKER_H
#define ARMOR_PIPELINE_TRACKER_H

#include <deque>
#include <vector>

#include <opencv2/core.hpp>

class PlateTracker {
public:
    struct Result {
        bool valid = false;    // 这一帧有没有可用的观测
        int index = -1;        // 选了 candidates 里的哪一个
        cv::Point3d position;  // 这一帧的位置（拟合值；只有 1 帧时就是观测值）
        cv::Point3d velocity;  // 世界系速度，m/s
        double residual = 0.0; // 拟合残差（人均，米）。目标在急转时它会变大
        int samples = 0;       // 当前窗口里有几帧观测
        bool switched = false; // 这一帧刚换了板（历史因此被清空）
    };

    // candidates 按"像不像一块装甲板"降序（matchArmors 已经排好）。
    // 首次锁定时取第 0 个。
    //
    // origin 是云台旋转中心：关联门限要把误差拆成"沿视线/垂直视线"两个方向，
    // 因为 PnP 的深度噪声远大于横向噪声。
    Result update(double time, const std::vector<cv::Point3d>& candidates,
                  const cv::Point3d& origin);

    void reset();

    // 最后一次成功观测的时刻；没观测过返回负值。
    double last_measure_time() const { return last_measure_; }

private:
    struct Sample {
        double time = 0.0;
        cv::Point3d position;
    };

    bool fit(double time, cv::Point3d& position, cv::Point3d& velocity,
             double& residual) const;
    void push(double time, const cv::Point3d& position);
    void prune(double time);

    std::deque<Sample> history_;
    cv::Point3d velocity_{0.0, 0.0, 0.0};
    cv::Point3d smoothed_{0.0, 0.0, 0.0};
    bool smoothed_valid_ = false;
    double last_measure_ = -1.0;
};

#endif  // ARMOR_PIPELINE_TRACKER_H
