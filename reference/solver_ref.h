// reference/solver_ref.h
// 参考实现用到的两件事：装甲板三维解算 + 弹道。
//
// 这不是给学生看的答案，而是给讲师用来**标定阈值**的：
// 先证明"这件事能做到多好"，再决定 60 分和 90 分画在哪。
#ifndef ARMOR_REFERENCE_SOLVER_REF_H
#define ARMOR_REFERENCE_SOLVER_REF_H

#include <optional>

#include <opencv2/core.hpp>

#include "core/types.h"
#include "vision/light_bar.h"

namespace ref {

// 小装甲灯条中心间距 135 mm，大装甲 230 mm；灯条长度两者都是 56 mm。
// 参考实现不分类，两种都试一遍，取重投影误差小的那个——这样大装甲也不会翻车。
constexpr double kSmallSpacing = 0.135;
constexpr double kHeroSpacing = 0.230;
constexpr double kLightBarLength = 0.056;

struct ArmorPose {
    cv::Point3d position_gimbal;  // 装甲中心，云台系（x 前、y 左、z 上）
    cv::Point3d position_world;   // 装甲中心，世界系（跟踪/预测都在世界系里做）
    double distance = 0.0;        // 到云台旋转中心的距离
    double yaw = 0.0;             // 世界系方位角，向左为正
    double pitch = 0.0;           // 向下为正
    double reprojection_px = 0.0; // 解算质量，用来挑间距/判退化
    double spacing = 0.0;         // 实际用的是哪个间距
};

// 用两根灯条的上下端点做平面 PnP。失败返回 nullopt。
std::optional<ArmorPose> solveArmor(const LightBar& left, const LightBar& right,
                                    const Observation& observation);

// 世界系 <-> 云台系的旋转，R = Rz(yaw) * Ry(pitch)。
cv::Matx33d gimbalToWorld(double yaw, double pitch);

// 由"到目标的直线方向"算出发射角：比直线多抬 elevation。
// 返回 (yaw, pitch)，pitch 向下为正。
struct AimSolution {
    double yaw = 0.0;
    double pitch = 0.0;
    double flight_time = 0.0;
    double elevation = 0.0;
};
AimSolution solveAim(const cv::Point3d& target_world, const cv::Point3d& origin,
                     double bullet_speed, double gravity);

// 把航向角归一化到 (-pi, pi]。
double wrapPi(double angle);

}  // namespace ref

#endif  // ARMOR_REFERENCE_SOLVER_REF_H
