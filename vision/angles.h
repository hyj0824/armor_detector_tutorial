// vision/angles.h
// 像素 -> 角度。v0 的"解算"全部在这里，一共不到 30 行。
//
// 为什么不做 PnP：v0 只需要"把目标压到画面中心"，这件事在角度空间就能完成。
// 代价是拿不到距离，因此做不了弹道补偿和提前量——这就是矩阵靶/移动靶会失效的原因。
#ifndef ARMOR_VISION_ANGLES_H
#define ARMOR_VISION_ANGLES_H

#include <opencv2/core.hpp>

struct RelativeAngle {
    double yaw = 0.0;    // 向左为正
    double pitch = 0.0;  // 向下为正
};

// 把图像上一个点换成"相对光轴"的角度。
//
// 坐标系提醒（这三句是本文件唯一的难点）：
//   * 相机 optical 系：x 向右、y 向下、z 向前
//   * 云台系：x 向前、y 向左、z 向上
//   * 因此像素向右 -> yaw 为负；像素向下 -> pitch 为正
//
// 这里忽略了两件小事：相机相对云台旋转中心的安装偏移（约 8cm/4cm），
// 以及镜头畸变（仿真是零畸变，真机不是）。
RelativeAngle relativeAngleFromPixel(const cv::Point2f& pixel, double fx,
                                    double fy, double cx, double cy);

#endif  // ARMOR_VISION_ANGLES_H
