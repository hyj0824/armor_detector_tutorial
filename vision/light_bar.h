// vision/light_bar.h
// 灯条：一个长条形的 RotatedRect，加上预先算好的长度/宽度/倾角，
// 这样后面过滤和配对时不用反复重算。
#ifndef ARMOR_VISION_LIGHT_BAR_H
#define ARMOR_VISION_LIGHT_BAR_H

#include <vector>

#include <opencv2/core.hpp>

#include "vision/config.h"

struct LightBar {
    cv::RotatedRect rect;
    double length = 0.0;  // 长边
    double width = 0.0;   // 短边
    double angle = 0.0;   // 长边相对竖直方向的夹角，单位度
    cv::Point2f center{0.0f, 0.0f};

    LightBar() = default;
    explicit LightBar(const cv::RotatedRect& r);

    double aspectRatio() const;
    bool isValid(const LightBarConfig& config) const;
};

// findContours -> minAreaRect -> 几何过滤 -> 按 x 排序
std::vector<LightBar> findLightBars(const cv::Mat& binary,
                                    const LightBarConfig& config);

#endif  // ARMOR_VISION_LIGHT_BAR_H
