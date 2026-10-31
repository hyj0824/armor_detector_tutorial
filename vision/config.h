// vision/config.h
// 检测器的全部可调参数。改这里就能做参数敏感性实验，不用改代码。
#ifndef ARMOR_VISION_CONFIG_H
#define ARMOR_VISION_CONFIG_H

enum class TargetColor {
    Red,
    Blue,
};

struct PreprocessConfig {
    int saturation_min = 55;
    int value_min = 100;
    int color_difference_min = 35;
    // 核大小要能盖住"过曝后被挖空的白洞"。
    // 3 在 3.8~5.5 m 有效；改 5 能救回 3.5 m，但 assets/ideal 里的
    // ideal_blue_robot.jpg 会掉到 0 个候选；改 7 两边都崩。
    // ctest 的 ideal_samples 就是用来盯住这个权衡的。
    int morphology_kernel = 3;
};

struct LightBarConfig {
    double min_area = 30.0;
    double max_area = 50000.0;
    double min_aspect_ratio = 1.5;
    double max_aspect_ratio = 18.0;
    double max_tilt_deg = 45.0;
};

struct ArmorConfig {
    double max_angle_diff_deg = 20.0;
    double max_length_ratio = 1.8;
    double max_y_offset_ratio = 0.7;
    double min_x_offset_ratio = 0.5;
    double max_x_offset_ratio = 6.0;
    double min_score = 0.35;
};

struct DetectorConfig {
    TargetColor target_color = TargetColor::Red;
    PreprocessConfig preprocess;
    LightBarConfig light_bar;
    ArmorConfig armor;
};

#endif  // ARMOR_VISION_CONFIG_H
