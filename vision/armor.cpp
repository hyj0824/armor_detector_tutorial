// vision/armor.cpp
#include "vision/armor.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

Armor::Armor(const LightBar& left, const LightBar& right,
             const ArmorConfig& config)
    : left_(left), right_(right) {
    // 四个几何量全部做了归一化，这样换分辨率、换距离时阈值不用重调。
    const double average_length = (left.length + right.length) / 2.0;
    const double angle_diff = std::abs(left.angle - right.angle);
    const double length_ratio =
        std::max(left.length, right.length) /
        std::max(1.0, std::min(left.length, right.length));
    const double y_offset_ratio =
        std::abs(left.center.y - right.center.y) / average_length;
    const double x_offset_ratio =
        std::abs(left.center.x - right.center.x) / average_length;

    const bool geometry_ok =
        angle_diff <= config.max_angle_diff_deg &&
        length_ratio <= config.max_length_ratio &&
        y_offset_ratio <= config.max_y_offset_ratio &&
        x_offset_ratio >= config.min_x_offset_ratio &&
        x_offset_ratio <= config.max_x_offset_ratio;
    if (!geometry_ok) {
        score_ = 0.0;
        return;
    }

    const double angle_score = 1.0 - angle_diff / config.max_angle_diff_deg;
    const double length_score =
        1.0 - (length_ratio - 1.0) / (config.max_length_ratio - 1.0);
    const double y_score = 1.0 - y_offset_ratio / config.max_y_offset_ratio;
    score_ = std::clamp(0.35 * angle_score + 0.35 * length_score +
                            0.30 * y_score,
                        0.0, 1.0);
    if (score_ < config.min_score) {
        score_ = 0.0;
    }
}

cv::Point2f Armor::center() const {
    return (left_.center + right_.center) * 0.5F;
}

cv::Rect Armor::boundingBox() const {
    const cv::Rect l = left_.rect.boundingRect();
    const cv::Rect r = right_.rect.boundingRect();
    return l | r;
}

bool Armor::isValid() const {
    return score_ > 0.0;
}

std::vector<Armor> matchArmors(const std::vector<LightBar>& bars,
                               const ArmorConfig& config) {
    std::vector<Armor> armors;
    for (std::size_t i = 0; i < bars.size(); ++i) {
        for (std::size_t j = i + 1; j < bars.size(); ++j) {
            Armor armor(bars[i], bars[j], config);
            if (armor.isValid()) {
                armors.push_back(armor);
            }
        }
    }
    std::sort(armors.begin(), armors.end(),
              [](const Armor& a, const Armor& b) {
                  return a.score() > b.score();
              });
    return armors;
}
