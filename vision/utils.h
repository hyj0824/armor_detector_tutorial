// vision/utils.h
// 通用小工具：文件 / 几何 / 绘制。与检测逻辑无关的助手集中在这里。
#ifndef ARMOR_VISION_UTILS_H
#define ARMOR_VISION_UTILS_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace utils {

// 返回全小写副本（仅处理 ASCII，足够用于扩展名比较）。
std::string toLower(std::string text);

// 按扩展名判断是否为支持的图片文件。
bool isImageFile(const std::string& path);

// 列出目录下的图片文件，按路径名排序，保证输入顺序稳定。
std::vector<std::string> collectImages(const std::string& dir);

// 长边相对竖直方向的夹角，单位为度，范围 (-90, 90]。
// 注意 cv::RotatedRect 自带的 angle 语义随 OpenCV 版本变化过，
// 这里一律从四个顶点重算，避免踩这个坑。
double angleFromVertical(const cv::RotatedRect& rect);

// 用四条边绘制旋转矩形。
void drawRotatedRect(cv::Mat& frame, const cv::RotatedRect& rect,
                     const cv::Scalar& color, int thickness);

// 把角度归一化到 (-pi, pi]。
double wrapPi(double angle);

}  // namespace utils

#endif  // ARMOR_VISION_UTILS_H
