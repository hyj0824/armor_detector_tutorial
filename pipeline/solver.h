// pipeline/solver.h
// 两件事：把一对灯条解成三维位姿（顺带拿到距离），以及由目标位置算出该打哪个角。
//
// 这两件事是 S1 的全部。没有距离就没有弹道补偿，没有弹道补偿 5.5 m 就要脱靶：
// 23 m/s 的弹丸飞 5.25 m 要 0.24 s，这期间掉 0.28 m，对应 2.8°——而装甲板在
// 5.25 m 处只占 ±0.68°。
#ifndef ARMOR_PIPELINE_SOLVER_H
#define ARMOR_PIPELINE_SOLVER_H

#include <optional>

#include <opencv2/core.hpp>

#include "core/types.h"
#include "vision/light_bar.h"

namespace solver {

// RM 装甲板的物理尺寸（米）。灯条长度两种都一样，只有间距不同。
constexpr double kSmallSpacing = 0.135;
constexpr double kLargeSpacing = 0.230;
constexpr double kLightBarLength = 0.056;

// 一次 PnP 的结果。position_* 都是"装甲板中心"。
struct ArmorPose {
    cv::Point3d position_gimbal;  // 云台系：x 前、y 左、z 上
    cv::Point3d position_world;   // 世界系：跟踪和预测都在这个系里做
    double distance = 0.0;        // 到云台旋转中心的距离
    double reprojection_px = 0.0; // 解算质量，也用来挑"小装甲还是大装甲"
    double spacing = 0.0;         // 最后用了哪个间距假设
    // 板面法向与视线的夹角余弦。接近 1 = 正对，接近 0 = 侧看。
    // 侧看的装甲板在几何上几乎退化（矩形投影成一根线），深度解不准，
    // 所以这个量是用来丢候选的。
    double cos_incidence = 1.0;
};

// 用两根灯条的端点做平面 PnP。解不出来返回 nullopt。
//
// reference_distance 是上一帧定下来的距离。小装甲（0.135）和大装甲（0.230）
// 在 5 m 处只差亚像素级的透视，重投影误差根本区分不开——所以光靠误差会导致
// 假设逐帧在两种之间跳，距离跟着跳 0.3 m 以上。用"上一帧距离"做时间一致性的
// 约束比用重投影误差稳得多。传负数表示没有先验。
std::optional<ArmorPose> solveArmor(const LightBar& left, const LightBar& right,
                                    const Observation& observation,
                                    double reference_distance = -1.0);

// 云台系 -> 世界系，R = Rz(yaw) * Ry(pitch)。
cv::Matx33d gimbalToWorld(double yaw, double pitch);

struct AimSolution {
    bool ok = false;
    double yaw = 0.0;         // 世界系方位角，向左为正
    double pitch = 0.0;       // 向下为正
    double elevation = 0.0;   // 相对直线的抬升量（重力补偿）
    double flight_time = 0.0; // 出膛到命中的飞行时间（不含发射延迟）
};

// 解出"打中 target_world 该用的角度"。
//
// 注意它不是"指向 target 的直线角"：它比直线多抬一点 elevation，
// 因为弹丸在这段时间里会往下掉。所以**不要**拿 commands 的角度去和
// 直线角比大小来判断准不准（会差 1.5~2.8°）。
AimSolution solveAim(const cv::Point3d& target_world,
                     const cv::Point3d& origin,
                     const Observation& observation);

}  // namespace solver

#endif  // ARMOR_PIPELINE_SOLVER_H
