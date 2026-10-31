// vision/detector.h
// 检测器：mask -> 灯条 -> 装甲板候选。
//
// v0 的检测器只输出【图像平面】的结果。它不知道目标有多远，也不知道朝哪。
// 目标选择（挑哪一块板）不在这里，属于 pipeline 的事——见 pipeline/pipeline.h。
#ifndef ARMOR_VISION_DETECTOR_H
#define ARMOR_VISION_DETECTOR_H

#include <vector>

#include <opencv2/core.hpp>

#include "vision/armor.h"
#include "vision/config.h"
#include "vision/light_bar.h"

struct Detection {
    cv::Mat mask;
    std::vector<LightBar> light_bars;
    std::vector<Armor> armors;
    double detect_ms = 0.0;
};

class ArmorDetector {
public:
    explicit ArmorDetector(DetectorConfig config = {});

    Detection detect(const cv::Mat& frame) const;

    const DetectorConfig& config() const { return config_; }

private:
    DetectorConfig config_;
};

#endif  // ARMOR_VISION_DETECTOR_H
