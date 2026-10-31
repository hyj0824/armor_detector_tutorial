// vision/angles.cpp
#include "vision/angles.h"

#include <cmath>

RelativeAngle relativeAngleFromPixel(const cv::Point2f& pixel, double fx,
                                    double fy, double cx, double cy) {
    const double dx = static_cast<double>(pixel.x) - cx;
    const double dy = static_cast<double>(pixel.y) - cy;

    RelativeAngle angle;
    angle.yaw = -std::atan2(dx, fx);
    angle.pitch = std::atan2(dy, fy);
    return angle;
}
