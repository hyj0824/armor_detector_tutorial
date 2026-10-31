// vision/armor.h
// 装甲板候选：两根灯条 + 一个几何评分。
// 注意这里只有【图像平面】的信息，没有任何三维位姿——v0 不做 PnP。
#ifndef ARMOR_VISION_ARMOR_H
#define ARMOR_VISION_ARMOR_H

#include <vector>

#include <opencv2/core.hpp>

#include "vision/config.h"
#include "vision/light_bar.h"

class Armor {
public:
    Armor() = default;
    Armor(const LightBar& left, const LightBar& right,
          const ArmorConfig& config);

    const LightBar& left() const { return left_; }
    const LightBar& right() const { return right_; }

    double score() const { return score_; }
    cv::Point2f center() const;   // 两根灯条中心的中点
    cv::Rect boundingBox() const; // 两灯条外接矩形的并集
    bool isValid() const;

private:
    LightBar left_;
    LightBar right_;
    double score_ = 0.0;
};

// 所有灯条两两配对，保留得分不为 0 的候选，按分数从高到低排序。
std::vector<Armor> matchArmors(const std::vector<LightBar>& bars,
                               const ArmorConfig& config);

#endif  // ARMOR_VISION_ARMOR_H
