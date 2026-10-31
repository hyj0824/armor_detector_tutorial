// vision/detector.cpp
#include "vision/detector.h"

#include <cstdint>
#include <stdexcept>

#include <opencv2/core.hpp>

#include "vision/preprocess.h"

ArmorDetector::ArmorDetector(DetectorConfig config) : config_(config) {
    // 配置错误应该尽早炸掉，而不是跑出莫名其妙的结果。
    if (config_.preprocess.saturation_min < 0 ||
        config_.preprocess.saturation_min > 255 ||
        config_.preprocess.value_min < 0 || config_.preprocess.value_min > 255 ||
        config_.preprocess.color_difference_min < 0 ||
        config_.preprocess.color_difference_min > 255 ||
        config_.preprocess.morphology_kernel < 1 ||
        config_.light_bar.min_area <= 0.0 ||
        config_.light_bar.max_area < config_.light_bar.min_area ||
        config_.light_bar.min_aspect_ratio <= 0.0 ||
        config_.light_bar.max_aspect_ratio <
            config_.light_bar.min_aspect_ratio ||
        config_.armor.max_angle_diff_deg <= 0.0 ||
        config_.armor.max_length_ratio <= 1.0 ||
        config_.armor.max_y_offset_ratio <= 0.0 ||
        config_.armor.min_x_offset_ratio < 0.0 ||
        config_.armor.max_x_offset_ratio < config_.armor.min_x_offset_ratio ||
        config_.armor.min_score < 0.0 || config_.armor.min_score > 1.0) {
        throw std::invalid_argument("invalid armor detector configuration");
    }
}

Detection ArmorDetector::detect(const cv::Mat& frame) const {
    const std::int64_t begin = cv::getTickCount();

    Detection result;
    result.mask = makeColorMask(frame, config_.target_color, config_.preprocess);
    result.light_bars = findLightBars(result.mask, config_.light_bar);
    result.armors = matchArmors(result.light_bars, config_.armor);

    result.detect_ms =
        static_cast<double>(cv::getTickCount() - begin) * 1000.0 /
        cv::getTickFrequency();
    return result;
}
