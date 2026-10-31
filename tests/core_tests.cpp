// tests/core_tests.cpp
// 两组回归测试：
//   core          角度约定、绕回、弹道解、合成装甲板的位姿恢复、开火门控
//   ideal_samples 4 张实拍正例的检测框 IoU
//
// 这些测试只锁"行为"，不锁"实现"。你可以随便重写内部代码，只要测试还绿。
// 而一旦你把角度符号改反，core 会立刻红——这是它存在的唯一理由。
//
// 【合成场景是物理自洽的】：用仿真的同一套变换把一个已知位姿的装甲板
// 投影到像素上，再画成两根灯条。所以 PnP 解回来的距离/角度必须和
// 真值对得上。不要用"随手画两根竖线"当夹具——那样两极灯条的长度和
// 间距比例是错的（真装甲是 0.135 / 0.056 = 2.41），任何重投影门限都会
// 把候选扔掉，测试会以一个和角度符号毫无关系的理由变红。
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "core/types.h"
#include "pipeline/pipeline.h"
#include "pipeline/solver.h"
#include "vision/angles.h"
#include "vision/detector.h"
#include "vision/utils.h"

namespace {

namespace fs = std::filesystem;

constexpr double kRadToDeg = 180.0 / CV_PI;
constexpr double kFx = 554.256;  // 640x480、水平视场约 60°
constexpr double kCx = 320.0;
constexpr double kCy = 240.0;

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool nearly(double a, double b, double tolerance = 1e-9) {
    return std::abs(a - b) <= tolerance;
}

Observation makeObservation(double gimbal_yaw = 0.0, double gimbal_pitch = 0.0) {
    Observation observation;
    observation.bgr = cv::Mat::zeros(480, 640, CV_8UC3);
    observation.sim_time = 1.0;
    observation.fx = kFx;
    observation.fy = kFx;
    observation.cx = kCx;
    observation.cy = kCy;
    observation.gimbal_yaw = gimbal_yaw;
    observation.gimbal_pitch = gimbal_pitch;
    observation.gimbal_origin = cv::Point3d(0.0, 0.0, 0.65);
    observation.session_id = "test";
    // 和仿真 configs/default.cfg 一致
    observation.bullet_speed = 23.0;
    observation.gravity = 9.7833;
    observation.launch_delay = 0.015;
    observation.barrel_length = 0.22;
    observation.camera_offset = cv::Point3d(0.08, 0.0, 0.04);
    return observation;
}

// 世界点 -> 像素。这是 solver 的逆变换，用仿真的同一套约定：
//   p_world = R(yaw,pitch) * p_gimbal + origin
//   相机在云台系里偏 camera_offset；optical 系 x 右 y 下 z 前。
cv::Point2f projectWorld(const cv::Point3d& world, const Observation& observation) {
    const cv::Matx33d r_gimbal2world =
        solver::gimbalToWorld(observation.gimbal_yaw, observation.gimbal_pitch);
    const cv::Vec3d rel(world.x - observation.gimbal_origin.x,
                        world.y - observation.gimbal_origin.y,
                        world.z - observation.gimbal_origin.z);
    const cv::Vec3d p_gimbal = r_gimbal2world.t() * rel;
    const double gx = p_gimbal[0] - observation.camera_offset.x;
    const double gy = p_gimbal[1] - observation.camera_offset.y;
    const double gz = p_gimbal[2] - observation.camera_offset.z;
    // gimbal -> optical：x_c = -y_g, y_c = -z_g, z_c = x_g
    const double x_c = -gy;
    const double y_c = -gz;
    const double z_c = gx;
    return {static_cast<float>(observation.fx * x_c / z_c + observation.cx),
            static_cast<float>(observation.fy * y_c / z_c + observation.cy)};
}

// 在图上画一块正对云台的装甲板。center/tangent/up 定义它的位姿，
// spacing 和 length 用真实尺寸，所以 PnP 能把位姿解回来。
void drawArmor(cv::Mat& image, const Observation& observation,
               const cv::Point3d& center, const cv::Point3d& tangent,
               const cv::Point3d& up, const cv::Scalar& color,
               double spacing = solver::kSmallSpacing,
               double length = solver::kLightBarLength) {
    for (int side = -1; side <= 1; side += 2) {
        const cv::Point3d base = center + tangent * (side * spacing * 0.5);
        const cv::Point3d a = base + up * (length * 0.5);
        const cv::Point3d b = base - up * (length * 0.5);
        cv::line(image, projectWorld(a, observation), projectWorld(b, observation),
                 color, 4);
    }
}

// 一块正对云台、位于 (distance, lateral, 0.26) 的装甲板。
void drawFacingArmor(cv::Mat& image, const Observation& observation,
                     double distance, double lateral = 0.0) {
    const cv::Point3d center(distance, lateral, 0.26);
    const cv::Point3d tangent(0.0, 1.0, 0.0);   // 板面横向
    const cv::Point3d up(0.0, 0.0, 1.0);        // 板面纵向（向上）
    drawArmor(image, observation, center, tangent, up, cv::Scalar(0, 0, 255));
}

void testAngleConvention() {
    // 像素向右 -> yaw 为负（云台要向右转 = yaw 减小）
    const RelativeAngle right = relativeAngleFromPixel({420.0f, 240.0f}, kFx, kFx,
                                                       kCx, kCy);
    require(right.yaw < 0.0, "像素在右侧时 yaw 必须为负");
    require(nearly(right.pitch, 0.0), "同一水平线上 pitch 必须为 0");

    // 像素向下 -> pitch 为正（pitch 约定向下为正）
    const RelativeAngle below = relativeAngleFromPixel({320.0f, 340.0f}, kFx, kFx,
                                                       kCx, kCy);
    require(below.pitch > 0.0, "像素在下方时 pitch 必须为正");
    require(nearly(below.yaw, 0.0), "同一竖线上 yaw 必须为 0");

    const RelativeAngle center = relativeAngleFromPixel({320.0f, 240.0f}, kFx, kFx,
                                                        kCx, kCy);
    require(nearly(center.yaw, 0.0) && nearly(center.pitch, 0.0), "光轴中心必须是零角");

    const double expected = std::atan2(100.0, kFx);
    require(std::abs(std::abs(right.yaw) - expected) < 1e-9,
            "yaw 的量级应等于 atan2(dx, fx)");
}

void testWrap() {
    require(std::abs(utils::wrapPi(3.0 * CV_PI) - CV_PI) < 1e-9,
            "3pi 应绕回到 pi");
    require(std::abs(utils::wrapPi(-3.0 * CV_PI) - CV_PI) < 1e-9,
            "-3pi 应绕回到 pi");
    require(nearly(utils::wrapPi(0.3), 0.3), "小角度不应被改动");
}

void testBallisticHitsTarget() {
    // 用 solveAim 给出的角度飞一次抛体，看落点高度能不能贴住目标。
    // 这个测试和仿真无关，只验证弹道公式和符号。
    const Observation observation = makeObservation();
    const cv::Point3d origin(observation.gimbal_origin.x,
                             observation.gimbal_origin.y,
                             observation.gimbal_origin.z);
    for (double distance : {2.0, 4.0, 5.5, 7.0}) {
        const cv::Point3d target(distance, 0.3, 0.26);
        const solver::AimSolution aim = solver::solveAim(target, origin, observation);
        require(aim.ok, "这个距离应该有解");

        const cv::Vec3d direction(std::cos(aim.pitch) * std::cos(aim.yaw),
                                  std::cos(aim.pitch) * std::sin(aim.yaw),
                                  -std::sin(aim.pitch));
        cv::Vec3d position = cv::Vec3d(origin.x, origin.y, origin.z) +
                             direction * observation.barrel_length;
        cv::Vec3d velocity = direction * observation.bullet_speed;
        const double target_horizontal = std::hypot(target.x - origin.x,
                                                    target.y - origin.y);
        const double dt = 1e-5;
        double elapsed = 0.0;
        while (std::hypot(position[0] - origin.x, position[1] - origin.y) <
                   target_horizontal &&
               elapsed < 3.0) {
            velocity[2] -= observation.gravity * dt;
            position += velocity * dt;
            elapsed += dt;
        }
        require(std::abs(position[2] - target.z) < 0.01,
                "按解出来的角飞，落点高度应该贴住目标（差超过 1 cm 说明弹道公式错了）");
        require(std::abs(elapsed - aim.flight_time) < 0.01,
                "flight_time 应该和真飞一遍的时间一致");
    }
}

void testPipelineRecoversPose() {
    // 2.5 m 处一块正对云台的装甲板：距离、方位角都要能解回来。
    PipelineConfig config;
    config.detector.target_color = TargetColor::Red;
    Pipeline pipeline(config);

    const Observation observation = makeObservation();
    cv::Mat image = observation.bgr.clone();
    drawFacingArmor(image, observation, 2.5);

    pipeline.detect(image);  // 首帧只是为了确认检测链路通了
    const Detection detection = pipeline.detect(image);
    require(detection.light_bars.size() == 2, "合成图应检出两根灯条");
    require(detection.armors.size() == 1, "合成图应检出 1 个装甲板候选");

    const TargetEstimate estimate = pipeline.estimate(observation, detection);
    require(estimate.valid, "有候选时估计必须有效");
    require(estimate.distance > 2.2 && estimate.distance < 2.8,
            "距离应接近 2.5 m（解得离谱说明 PnP 的坐标系或尺度错了）");
    require(std::abs(estimate.yaw) < 0.01,
            "目标在正前方，绝对 yaw 应该接近 0");
    require(std::abs(estimate.pixel.x - kCx) < 6.0F, "瞄准点应在画面中央附近");
}

void testPipelineRespectsGimbalPose() {
    // 云台已经左转 10 度，但目标在正前方：绝对角仍应该是 0，不是 10 度。
    PipelineConfig config;
    config.detector.target_color = TargetColor::Red;
    Pipeline pipeline(config);

    const double gimbal_yaw = 10.0 / kRadToDeg;
    const Observation observation = makeObservation(gimbal_yaw, 0.0);
    cv::Mat image = observation.bgr.clone();
    drawFacingArmor(image, observation, 2.5);

    const Detection detection = pipeline.detect(image);
    const TargetEstimate estimate = pipeline.estimate(observation, detection);
    require(estimate.valid, "估计必须有效");
    require(std::abs(utils::wrapPi(estimate.yaw)) < 0.01,
            "绝对角必须抵消掉当前的云台角");
}

void testFireGate() {
    PipelineConfig config;
    config.detector.target_color = TargetColor::Red;
    config.allow_fire = false;
    Pipeline pipeline(config);

    const Observation observation = makeObservation();
    cv::Mat image = observation.bgr.clone();
    drawFacingArmor(image, observation, 2.5);

    const Detection detection = pipeline.detect(image);
    const TargetEstimate estimate = pipeline.estimate(observation, detection);
    const Command command = pipeline.command(observation, estimate);
    require(!command.fire, "allow_fire=false 时绝不能开火");
}

void testNoTargetNoCommand() {
    // 空画面：estimate 无效，command 必须停控（control=false, fire=false）。
    PipelineConfig config;
    config.detector.target_color = TargetColor::Red;
    config.allow_fire = true;
    Pipeline pipeline(config);

    const Observation observation = makeObservation();
    const cv::Mat image = observation.bgr.clone();  // 全黑
    const Detection detection = pipeline.detect(image);
    const TargetEstimate estimate = pipeline.estimate(observation, detection);
    require(!estimate.valid, "没有候选时估计必须无效");
    const Command command = pipeline.command(observation, estimate);
    require(!command.control && !command.fire, "没目标时必须停控停火");
}

double intersectionOverUnion(const cv::Rect& lhs, const cv::Rect& rhs) {
    const cv::Rect intersection = lhs & rhs;
    const double intersection_area = intersection.area();
    return intersection_area / (lhs.area() + rhs.area() - intersection_area + 1e-9);
}

void testIdealSamples() {
    struct Sample {
        const char* filename;
        TargetColor color;
        cv::Rect truth;
    };
    const Sample samples[] = {
        {"ideal_blue_near.jpg", TargetColor::Blue, cv::Rect(105, 412, 919, 265)},
        {"ideal_blue_robot.jpg", TargetColor::Blue, cv::Rect(226, 229, 172, 70)},
        {"ideal_red_robot.jpg", TargetColor::Red, cv::Rect(180, 431, 171, 94)},
        {"ideal_red_front.jpg", TargetColor::Red, cv::Rect(419, 357, 494, 167)},
    };

    for (const Sample& sample : samples) {
        const fs::path path = fs::path(ARMOR_PROJECT_DIR) / "assets" / "ideal" /
                              sample.filename;
        const cv::Mat image = cv::imread(path.string());
        require(!image.empty(), "读不到正例图片");

        DetectorConfig config;
        config.target_color = sample.color;
        const ArmorDetector detector(config);
        const Detection detection = detector.detect(image);

        // 只要求"标注框对应的候选出现在结果里"——这是检测器的职责。
        // "从多个候选里挑对哪一个"是 pipeline 的事。
        require(!detection.armors.empty(), "正例应该至少产生一个候选");
        double best_iou = 0.0;
        for (const Armor& armor : detection.armors) {
            best_iou = std::max(best_iou,
                                intersectionOverUnion(armor.boundingBox(), sample.truth));
        }
        require(best_iou >= 0.65, "没有任何候选框与标注框的 IoU 达到 0.65");
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "用法: armor_tests <core|ideal_samples>\n";
        return EXIT_FAILURE;
    }
    const std::string name = argv[1];
    try {
        if (name == "core") {
            testAngleConvention();
            testWrap();
            testBallisticHitsTarget();
            testPipelineRecoversPose();
            testPipelineRespectsGimbalPose();
            testFireGate();
            testNoTargetNoCommand();
        } else if (name == "ideal_samples") {
            testIdealSamples();
        } else {
            throw std::invalid_argument("未知测试: " + name);
        }
        std::cout << "PASS: " << name << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << name << ": " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
