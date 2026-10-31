// vision/light_bar.cpp
#include "vision/light_bar.h"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

#include "vision/utils.h"

LightBar::LightBar(const cv::RotatedRect& r) : rect(r) {
    length = std::max(r.size.height, r.size.width);
    width = std::min(r.size.height, r.size.width);
    angle = utils::angleFromVertical(r);
    center = r.center;
}

double LightBar::aspectRatio() const {
    return width > 1e-6 ? length / width : 0.0;
}

bool LightBar::isValid(const LightBarConfig& config) const {
    // 注意边界：这里用严格不等号，是行为约定的一部分。
    // 改成 <= 会让刚好等于阈值的轮廓行为不同，写测试时要覆盖这一点。
    const double area = rect.size.area();
    const double ratio = aspectRatio();
    const bool area_ok = config.min_area < area && area < config.max_area;
    const bool ratio_ok = config.min_aspect_ratio < ratio &&
                          ratio < config.max_aspect_ratio;
    const bool angle_ok = std::abs(angle) < config.max_tilt_deg;
    return area_ok && ratio_ok && angle_ok;
}

std::vector<LightBar> findLightBars(const cv::Mat& binary,
                                    const LightBarConfig& config) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    std::vector<LightBar> bars;
    bars.reserve(contours.size());
    for (const auto& contour : contours) {
        LightBar bar(cv::minAreaRect(contour));
        if (bar.isValid(config)) {
            bars.push_back(bar);
        }
    }

    // 按 x 排序，后面配对时只需要看右侧的灯条，省一半比较。
    std::sort(bars.begin(), bars.end(),
              [](const LightBar& a, const LightBar& b) {
                  return a.center.x < b.center.x;
              });
    return bars;
}
