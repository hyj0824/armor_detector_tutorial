// core/types.h
// 管线与 IO 层之间的数据结构。这里只放"跨模块"的东西：
// 一帧的输入（Observation）和一次控制的输出（Command）。
//
// 设计上刻意让 Observation 自带全部上下文：像素、时间、标定、云台姿态。
// 下游任何一层都不许再去问"现在是几点""云台现在在哪"——那些是采集层的事。
#ifndef ARMOR_CORE_TYPES_H
#define ARMOR_CORE_TYPES_H

#include <string>

#include <opencv2/core.hpp>

// 一帧的全部输入。
//
// 注意 gimbal_* 是【拍摄这一帧时】的云台实际姿态，不是"现在"的姿态。
// frame.json 里自带这份快照，用它才能把像素角正确加上去；
// 用最新状态会让控制环多一个采样周期的延迟，表现为"能追但有点超调"。
struct Observation {
    cv::Mat bgr;
    double sim_time = 0.0;

    // 相机内参。由 frame.json 的 camera.K 拆开，行优先 3x3。
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;

    // 该帧拍摄时的云台实际姿态。约定：yaw 向左为正，pitch 向下为正，单位 rad。
    double gimbal_yaw = 0.0;
    double gimbal_pitch = 0.0;
    double gimbal_yaw_vel = 0.0;
    double gimbal_pitch_vel = 0.0;

    // 云台旋转中心的世界坐标（m）。要做距离/PnP/提前量就必须有它，
    // 因为换算到世界系时要用 p_world = R(yaw,pitch) * p_gimbal + origin。
    cv::Point3d gimbal_origin{0.0, 0.0, 0.0};

    std::string session_id;

    bool empty() const { return bgr.empty(); }
};

// 一次控制输出。交给采集层写进 command.json。
struct Command {
    bool control = false;
    bool fire = false;
    double yaw = 0.0;    // 绝对目标角，不是增量
    double pitch = 0.0;  // 绝对目标角，向下为正

    // 目标轨迹的前馈角速度（rad/s）。仿真侧会把它算进伺服：
    //   a = w^2*e + 2*w*(goal_v - v) + goal_a
    // 不填也合法，只是跟一个匀速移动的目标会有一个稳态滞后。
    double yaw_vel = 0.0;
    double pitch_vel = 0.0;
};

#endif  // ARMOR_CORE_TYPES_H
