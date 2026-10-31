// vision/utils.cpp
#include "vision/utils.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <set>

#include <opencv2/imgproc.hpp>

namespace fs = std::filesystem;

namespace utils {

std::string toLower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return text;
}

bool isImageFile(const std::string& path) {
    static const std::set<std::string> kImageExtensions = {
        ".png", ".jpg", ".jpeg", ".bmp", ".webp", ".ppm"};
    const std::string extension = fs::path(path).extension().string();
    return kImageExtensions.count(toLower(extension)) > 0;
}

std::vector<std::string> collectImages(const std::string& dir) {
    std::vector<std::string> images;
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.is_regular_file() && isImageFile(entry.path().string())) {
            images.push_back(entry.path().string());
        }
    }
    std::sort(images.begin(), images.end());
    return images;
}

double angleFromVertical(const cv::RotatedRect& rect) {
    cv::Point2f points[4];
    rect.points(points);
    cv::Point2f long_edge = points[1] - points[0];
    if (cv::norm(points[2] - points[1]) > cv::norm(long_edge)) {
        long_edge = points[2] - points[1];
    }
    double axis_angle = std::atan2(long_edge.y, long_edge.x) * 180.0 / CV_PI;
    while (axis_angle < 0.0) {
        axis_angle += 180.0;
    }
    while (axis_angle >= 180.0) {
        axis_angle -= 180.0;
    }
    return axis_angle - 90.0;
}

void drawRotatedRect(cv::Mat& frame, const cv::RotatedRect& rect,
                     const cv::Scalar& color, int thickness) {
    cv::Point2f points[4];
    rect.points(points);
    for (int i = 0; i < 4; ++i) {
        cv::line(frame, points[i], points[(i + 1) % 4], color, thickness);
    }
}

double wrapPi(double angle) {
    while (angle > CV_PI) {
        angle -= 2.0 * CV_PI;
    }
    while (angle <= -CV_PI) {
        angle += 2.0 * CV_PI;
    }
    return angle;
}

}  // namespace utils
