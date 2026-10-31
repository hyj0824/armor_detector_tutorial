// pipeline/solver.cpp
#include "pipeline/solver.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/calib3d.hpp>

namespace solver {

namespace {

// 灯条的两端：按 y 排序，取"上端中心"和"下端中心"。
//
// 为什么不用 minAreaRect 的四个角：mask 出来的宽度跟灯条真实宽度无关
// （它取决于曝光和阈值），只有长度是可信的。用端点把宽度这个自由度丢掉。
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
    // 物体坐标：x 沿装甲横向、y 向下、z 朝外，原点在板中心。
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
        // IPPE 是专给平面目标的：四个点、共面，用通用 DLT 会欠定。
        if (!cv::solvePnP(object_points, image_points, camera_matrix, cv::Mat(),
                          rvec, tvec, false, cv::SOLVEPNP_IPPE)) {
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

    // 板面法向在相机系里：物体坐标的 z 轴朝外，所以 n = R * (0,0,1)。
    cv::Mat rotation;
    cv::Rodrigues(rvec, rotation);
    const cv::Vec3d normal(rotation.at<double>(0, 2), rotation.at<double>(1, 2),
                           rotation.at<double>(2, 2));
    const cv::Vec3d t(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    const double norm_t = cv::norm(t);
    const double cos_incidence =
        norm_t > 1e-9 ? std::abs(normal.dot(t)) / norm_t : 0.0;

    // optical -> gimbal。仿真给的标定就是固定的这一组；安装偏移从
    // Observation 里读（忘了它 5 m 处会偏 0.46°，而容差只有 0.68°）。
    const cv::Matx33d r_cam2gimbal(0, 0, 1, -1, 0, 0, 0, -1, 0);
    const cv::Vec3d t_cam2gimbal(observation.camera_offset.x,
                                 observation.camera_offset.y,
                                 observation.camera_offset.z);
    const cv::Vec3d p_gimbal = r_cam2gimbal * t + t_cam2gimbal;

    ArmorPose pose;
    pose.position_gimbal = cv::Point3d(p_gimbal[0], p_gimbal[1], p_gimbal[2]);
    pose.distance = cv::norm(pose.position_gimbal);
    pose.reprojection_px = error;
    pose.spacing = spacing;
    pose.cos_incidence = cos_incidence;

    const cv::Matx33d r_gimbal2world =
        gimbalToWorld(observation.gimbal_yaw, observation.gimbal_pitch);
    const cv::Vec3d p_world =
        r_gimbal2world * p_gimbal + cv::Vec3d(observation.gimbal_origin.x,
                                              observation.gimbal_origin.y,
                                              observation.gimbal_origin.z);
    pose.position_world = cv::Point3d(p_world[0], p_world[1], p_world[2]);
    return pose;
}

}  // namespace

cv::Matx33d gimbalToWorld(double yaw, double pitch) {
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    return cv::Matx33d(cy * cp, -sy, cy * sp,
                       sy * cp, cy, sy * sp,
                       -sp, 0.0, cp);
}

std::optional<ArmorPose> solveArmor(const LightBar& left, const LightBar& right,
                                    const Observation& observation,
                                    double reference_distance) {
    if (left.center.x > right.center.x) {
        return solveArmor(right, left, observation, reference_distance);
    }
    const BarEnds left_ends = barEnds(left);
    const BarEnds right_ends = barEnds(right);

    // 分不清是小装甲还是大装甲，就两种都解一遇。
    //
    // 怎么选：有上一帧的距离就选更接近它的那个（时间一致性）；
    // 没有先验时（首次锁定）才退回重投影误差。为什么不能一直用重投影误差：
    // 0.135 和 0.230 在 5 m 处的透视差异是亚像素级的，两个假设的误差
    // 只差 0.1 px，于是每帧都在两者之间随机挑，距离跟着跳 0.3 m 以上，
    // 而速度是距离的导数——跳 0.3 m 就直接体现为速度估错 2 m/s。
    const auto small = solveWithSpacing(kSmallSpacing, left_ends, right_ends, observation);
    const auto large = solveWithSpacing(kLargeSpacing, left_ends, right_ends, observation);

    if (!small && !large) {
        return std::nullopt;
    }
    if (!small) {
        return large;
    }
    if (!large) {
        return small;
    }
    if (reference_distance > 0.0) {
        const double small_error =
            std::abs(std::log(std::max(small->distance, 1e-3) / reference_distance));
        const double large_error =
            std::abs(std::log(std::max(large->distance, 1e-3) / reference_distance));
        return small_error <= large_error ? small : large;
    }
    return small->reprojection_px <= large->reprojection_px ? small : large;
}

AimSolution solveAim(const cv::Point3d& target_world,
                     const cv::Point3d& origin,
                     const Observation& observation) {
    const cv::Point3d delta = target_world - origin;
    const double horizontal = std::hypot(delta.x, delta.y);
    const double speed = observation.bullet_speed;
    const double gravity = observation.gravity;
    const double barrel = observation.barrel_length;

    AimSolution solution;
    solution.yaw = std::atan2(delta.y, delta.x);
    if (horizontal < 1e-6 || speed <= 0.0) {
        return solution;
    }

    // 炮口在 origin + barrel * 方向 上，所以水平距离和高度差都要减掉
    // 这一段的投影；而抬升角又依赖这两者，于是要迭代。
    //
    // 解的是低伸弹道（低弧）：disc >= 0 才有解，打太远就没有解。
    double elevation = std::atan2(delta.z, horizontal);
    for (int i = 0; i < 8; ++i) {
        const double distance = horizontal - barrel * std::cos(elevation);
        const double height = delta.z - barrel * std::sin(elevation);
        if (distance <= 0.0) {
            return solution;
        }
        if (gravity < 1e-9) {
            elevation = std::atan2(height, distance);
            continue;
        }
        const double discriminant =
            speed * speed * speed * speed -
            gravity * (gravity * distance * distance +
                       2.0 * height * speed * speed);
        if (discriminant < 0.0) {
            return solution;  // 够不着
        }
        elevation = std::atan((speed * speed - std::sqrt(discriminant)) /
                              (gravity * distance));
    }

    solution.ok = true;
    solution.elevation = elevation;
    solution.pitch = -elevation;
    solution.flight_time =
        (horizontal - barrel * std::cos(elevation)) / (speed * std::cos(elevation));
    return solution;
}

}  // namespace solver
