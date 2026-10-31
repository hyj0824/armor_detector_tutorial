// vision/preprocess.h
// 颜色分割：BGR 图像 -> 二值 mask，只保留"目标颜色且足够饱和明亮"的像素。
#ifndef ARMOR_VISION_PREPROCESS_H
#define ARMOR_VISION_PREPROCESS_H

#include <opencv2/core.hpp>

#include "vision/config.h"

// 三个动作：
//   1) HSV 阈值（红色跨 Hue 首尾，需要两段取并集）
//   2) 对手色通道差（红用 R-B，蓝用 B-R）压掉灰白背景
//   3) 开/闭运算去掉噪点、补上空洞
// 返回单通道 8U 二值图。
cv::Mat makeColorMask(const cv::Mat& src, TargetColor color,
                      const PreprocessConfig& config);

#endif  // ARMOR_VISION_PREPROCESS_H
