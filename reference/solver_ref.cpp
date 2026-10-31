// reference/solver_ref.cpp
#include "reference/solver_ref.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/calib3d.hpp>

namespace ref {

namespace {

// 灯条的上下端点：按 y 排序取两端，取这条线段的"顶端中心"和"底端中心"。
// 用端点而不是 minAreaRect 的四个角，是因为 mask 的宽度跟真实灯条宽度无关，
// 只有长度是可信的。
struct BarEnds {
    cv::Point2f top;
    cv::Point2f bottom;
};

BarEnds barEnds(const LightBar& bar) {
    cv::Point2f points[4];
    bar.rect.points(points);
    std::sort(points, points + 4,
              [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
    BarEnds ends;
    ends.top = (points[0] + points[1]) * 0.5F;
    ends.bottom = (points[2] + points[3]) * 0.5F;
    return ends;
}

std::optional<ArmorPose> solveWithSpacing(double spacing, const BarEnds& left,
                                          const BarEnds& right,
                                          const Observation& observation) {
    // 物体点：x 沿装甲横向、y 向下、z 朝外，原点在装甲中心。
    const auto half_spacing = static_cast<float>(spacing * 0.5);
    const auto half_length = static_cast<float>(kLightBarLength * 0.5);
    const std::vector<cv::Point3f> object_points = {
        {-half_spacing, -half_length, 0.0F},  // 左灯条上端
        {-half_spacing, half_length, 0.0F},   // 左灯条下端
        {half_spacing, -half_length, 0.0F},   // 右灯条上端
        {half_spacing, half_length, 0.0F},    // 右灯条下端
    };
    const std::vector<cv::Point2f> image_points = {left.top, left.bottom,
                                                   right.top, right.bottom};

    const cv::Mat camera_matrix =
        (cv::Mat_<double>(3, 3) << observation.fx, 0.0, observation.cx, 0.0,
         observation.fy, observation.cy, 0.0, 0.0, 1.0);

    cv::Mat rvec;
    cv::Mat tvec;
    try {
        if (!cv::solvePnP(object_points, image_points, camera_matrix,
                          cv::Mat(), rvec, tvec, false, cv::SOLVEPNP_IPPE)) {
            return std::nullopt;
        }
    } catch (const cv::Exception&) {
        return std::nullopt;
    }

    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, cv::Mat(), projected);
    double error = 0.0;
    for (std::size_t i = 0; i < projected.size(); ++i) {
        error += cv::norm(projected[i] - image_points[i]);
    }
    error /= static_cast<double>(projected.size());

    // optical -> gimbal：仿真的标定固定为 [0 0 1; -1 0 0; 0 -1 0] + 安装偏移。
    // 这里硬编码是因为参考实现只对仿真负责；接真机要读 frame 里的外参。
    const cv::Matx33d r_cam2gimbal(0, 0, 1, -1, 0, 0, 0, -1, 0);
    const cv::Vec3d t_cam2gimbal(0.08, 0.0, 0.04);
    const cv::Vec3d t(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    const cv::Vec3d p_gimbal = r_cam2gimbal * t + t_cam2gimbal;

    ArmorPose pose;
    pose.position_gimbal = cv::Point3d(p_gimbal[0], p_gimbal[1], p_gimbal[2]);
    pose.distance = cv::norm(pose.position_gimbal);
    pose.reprojection_px = error;
    pose.spacing = spacing;

    // 云台系 -> 世界系
    const cv::Matx33d r_gimbal2world =
        gimbalToWorld(observation.gimbal_yaw, observation.gimbal_pitch);
    const cv::Vec3d p_world = r_gimbal2world * p_gimbal +
                              cv::Vec3d(observation.gimbal_origin.x,
                                        observation.gimbal_origin.y,
                                        observation.gimbal_origin.z);
    pose.position_world = cv::Point3d(p_world[0], p_world[1], p_world[2]);
    const double horizontal = std::hypot(p_world[0], p_world[1]);
    pose.yaw = std::atan2(p_world[1], p_world[0]);
    pose.pitch = -std::atan2(p_world[2], horizontal);
    return pose;
}

}  // namespace

double wrapPi(double angle) {
    while (angle > CV_PI) {
        angle -= 2.0 * CV_PI;
    }
    while (angle <= -CV_PI) {
        angle += 2.0 * CV_PI;
    }
    return angle;
}

cv::Matx33d gimbalToWorld(double yaw, double pitch) {
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    // Rz(yaw) * Ry(pitch)
    return cv::Matx33d(cy * cp, -sy, cy * sp,
                       sy * cp, cy, sy * sp,
                       -sp, 0.0, cp);
}

std::optional<ArmorPose> solveArmor(const LightBar& left, const LightBar& right,
                                    const Observation& observation) {
    if (left.center.x > right.center.x) {
        return solveArmor(right, left, observation);
    }
    const BarEnds left_ends = barEnds(left);
    const BarEnds right_ends = barEnds(right);

    const auto small = solveWithSpacing(kSmallSpacing, left_ends, right_ends, observation);
    const auto hero = solveWithSpacing(kHeroSpacing, left_ends, right_ends, observation);

    if (!small && !hero) {
        return std::nullopt;
    }
    if (!small) {
        return hero;
    }
    if (!hero) {
        return small;
    }
    // 两种假设都自洽，取重投影误差更小的那个。
    return small->reprojection_px <= hero->reprojection_px ? small : hero;
}

AimSolution solveAim(const cv::Point3d& target, const cv::Point3d& origin,
                     double bullet_speed, double gravity) {
    const cv::Point3d delta = target - origin;
    const double horizontal = std::hypot(delta.x, delta.y);
    AimSolution solution;
    solution.yaw = std::atan2(delta.y, delta.x);
    if (horizontal < 1e-6 || bullet_speed <= 0.0) {
        solution.pitch = -std::atan2(delta.z, horizontal);
        return solution;
    }

    // 先按直线距离估飞行时间，再迭代修正抬升角（抬升角又影响飞行时间）。
    double flight = std::hypot(horizontal, delta.z) / bullet_speed;
    for (int i = 0; i < 5; ++i) {
        const double drop = 0.5 * gravity * flight * flight;
        const double elevation = std::atan2(drop, horizontal);
        const double path = std::hypot(horizontal, delta.z + drop);
        flight = path / bullet_speed;
        solution.elevation = elevation;
    }
    solution.flight_time = flight;
    solution.pitch = -std::atan2(delta.z, horizontal) - solution.elevation;
    return solution;
}

}  // namespace ref
