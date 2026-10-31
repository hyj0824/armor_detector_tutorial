// tests/core_tests.cpp
// 两组回归测试：
//   core          角度约定的符号、目标选择、绕回
//   ideal_samples 4 张实拍正例的检测框 IoU
//
// 这些测试只锁"行为"，不锁"实现"。你可以随便重写内部代码，只要测试还绿。
// 而一旦你把角度符号改反，core 会立刻红——这是它存在的唯一理由。
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
#include "vision/angles.h"
#include "vision/detector.h"
#include "vision/light_bar.h"
#include "vision/utils.h"

namespace {

namespace fs = std::filesystem;

constexpr double kRadToDeg = 180.0 / CV_PI;

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool nearly(double a, double b, double tolerance = 1e-9) {
    return std::abs(a - b) <= tolerance;
}

// 640x480、水平视场角约 60 度的假相机。
Observation makeObservation(double gimbal_yaw = 0.0, double gimbal_pitch = 0.0) {
    Observation observation;
    observation.bgr = cv::Mat::zeros(480, 640, CV_8UC3);
    observation.sim_time = 1.0;
    observation.fx = 554.256;
    observation.fy = 554.256;
    observation.cx = 320.0;
    observation.cy = 240.0;
    observation.gimbal_yaw = gimbal_yaw;
    observation.gimbal_pitch = gimbal_pitch;
    observation.session_id = "test";
    return observation;
}

cv::Mat makeScene(const cv::Scalar& color, int center_x = 320) {
    cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::rectangle(image, cv::Rect(center_x - 65, 190, 10, 100), color, cv::FILLED);
    cv::rectangle(image, cv::Rect(center_x + 55, 190, 10, 100), color, cv::FILLED);
    cv::rectangle(image, cv::Rect(30, 30, 70, 10), color, cv::FILLED);
    return image;
}

void testAngleConvention() {
    // 像素向右 -> yaw 为负（云台要向右转=yaw 减小）
    const RelativeAngle right = relativeAngleFromPixel({420.0f, 240.0f}, 554.256,
                                                       554.256, 320.0, 240.0);
    require(right.yaw < 0.0, "像素在右侧时 yaw 必须为负");
    require(nearly(right.pitch, 0.0), "同一水平线上 pitch 必须为 0");

    // 像素向下 -> pitch 为正（pitch 约定向下为正）
    const RelativeAngle below = relativeAngleFromPixel({320.0f, 340.0f}, 554.256,
                                                       554.256, 320.0, 240.0);
    require(below.pitch > 0.0, "像素在下方时 pitch 必须为正");
    require(nearly(below.yaw, 0.0), "同一竖线上 yaw 必须为 0");

    // 光轴中心 -> 零角
    const RelativeAngle center = relativeAngleFromPixel({320.0f, 240.0f}, 554.256,
                                                        554.256, 320.0, 240.0);
    require(nearly(center.yaw, 0.0) && nearly(center.pitch, 0.0), "光轴中心必须是零角");

    // 100 像素 / fx 的反正切，量级要合理
    const double expected = std::atan2(100.0, 554.256);
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

void testPipelineSelectsCenterArmor() {
    PipelineConfig config;
    config.detector.target_color = TargetColor::Red;
    const Pipeline pipeline(config);

    const Observation observation = makeObservation();
    const Detection detection = pipeline.detect(makeScene(cv::Scalar(0, 0, 255)));
    require(detection.light_bars.size() == 2, "合成图应检出两根灯条");
    require(detection.armors.size() == 1, "合成图应检出 1 个装甲板候选");

    const TargetEstimate estimate = pipeline.estimate(observation, detection);
    require(estimate.valid, "有候选时估计必须有效");
    require(std::abs(estimate.pixel.x - 320.0f) < 3.0f, "瞄准点应在画面中央附近");

    // v0 的三个空位：距离、两个角速度都还为 0。
    require(nearly(estimate.distance, 0.0), "v0 不应凭空给出距离");
    require(nearly(estimate.yaw_rate, 0.0), "v0 不应凭空给角速度");
}

void testPipelineRespectsGimbalPose() {
    PipelineConfig config;
    config.detector.target_color = TargetColor::Red;
    const Pipeline pipeline(config);

    // 云台已经左转 10 度，画面中心的目标绝对角就应该是 10 度。
    const double gimbal_yaw = 10.0 / kRadToDeg;
    const Observation observation = makeObservation(gimbal_yaw, 0.0);
    const Detection detection = pipeline.detect(makeScene(cv::Scalar(0, 0, 255)));
    const TargetEstimate estimate = pipeline.estimate(observation, detection);

    require(estimate.valid, "估计必须有效");
    require(std::abs(utils::wrapPi(estimate.yaw - gimbal_yaw)) < 0.01,
            "绝对角必须等于【当帧云台角 + 相对角】，而不是只等于相对角");
}

void testFireGate() {
    PipelineConfig config;
    config.detector.target_color = TargetColor::Red;
    config.allow_fire = false;
    const Pipeline disabled_fire(config);

    const Observation observation = makeObservation();
    const Detection detection = disabled_fire.detect(makeScene(cv::Scalar(0, 0, 255)));
    const TargetEstimate estimate = disabled_fire.estimate(observation, detection);
    const Command command = disabled_fire.command(observation, estimate);
    require(!command.fire, "allow_fire=false 时绝不能开火");
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

        // 只要求“标注框对应的候选出现在结果里”——这是检测器的职责。
        // “从多个候选里挑对哪一个”是 pipeline 的事，而 v0 的挑法是刻意很弱的（见
        // pipeline.cpp），所以不在这里用最高分候选去断言 IoU。
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
            testPipelineSelectsCenterArmor();
            testPipelineRespectsGimbalPose();
            testFireGate();
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
