// pipeline/tracker.cpp
#include "pipeline/tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

// 关联门限。注意它**不能**是一个球形门限。
//
// PnP 的误差不是各向同性的：装甲板只有 4 个共面点，深度（沿视线方向）
// 是最不可观的那一维，实测噪声 0.1~0.3 m；而垂直于视线的方向只受角点
// 像素误差影响，噪声只有几厘米。用一个 0.25 m 的球门限的后果是：
// 深度噪声一抖就判"不是同一块板"，历史被清空，速度估计反复重启。
//
// 所以拆成两个方向：径向松，切向紧。切向 0.15 m 远小于相邻两块板
// 0.35 m 的间距，所以不会把两块板搞混。
constexpr double kRadialGate = 0.8;
constexpr double kTangentialGate = 0.15;

// 滑窗。用最小二乘直线拟合同时拿到"当前位置"和速度：
// 一阶滤波（alpha-beta）在目标有加速度时，位置和速度会各带一份滞后，
// 而提前量对速度误差极其敏感。
constexpr std::size_t kWindowSize = 8;
constexpr double kWindowSeconds = 0.28;

// 观测的低通时间常数。
//
// 为什么必须先低通再拟合：PnP 的深度噪声有 0.1~0.3 m，而最小二乘在
// 0.145 s 的窗口上把这点噪声除以窗口长度，就变成 1~2 m/s 的速度误差——
// 比目标真速度还大。速度估歪，提前量就歪，5.5 m 处 10 cm 的偏差就是 1°。
//
// 一阶低通把噪声压下去 ~3 倍，代价是约 kSmoothTau 的群延迟；
// 对恒定速度来说这个延迟是可以精确补回来的：拟合时把评估点往后挪同样多。
constexpr double kSmoothTau = 0.0;

// 超过这么久没有新观测就认为跟丢了，把历史作废。
// 必须小于仿真的指令看门狗（250 ms），否则你会对着一个已经不存在的位置继续发指令。
constexpr double kTimeout = 0.25;

}  // namespace

void PlateTracker::reset() {
    history_.clear();
    velocity_ = cv::Point3d(0.0, 0.0, 0.0);
    smoothed_ = cv::Point3d(0.0, 0.0, 0.0);
    smoothed_valid_ = false;
    last_measure_ = -1.0;
}

bool PlateTracker::fit(double time, cv::Point3d& position, cv::Point3d& velocity,
                       double& residual) const {
    if (history_.size() < 2) {
        return false;
    }
    double mean_t = 0.0;
    cv::Point3d mean_p(0.0, 0.0, 0.0);
    for (const Sample& sample : history_) {
        mean_t += sample.time;
        mean_p += sample.position;
    }
    const double n = static_cast<double>(history_.size());
    mean_t /= n;
    mean_p *= 1.0 / n;

    double var_t = 0.0;
    cv::Point3d cov(0.0, 0.0, 0.0);
    for (const Sample& sample : history_) {
        const double dt = sample.time - mean_t;
        var_t += dt * dt;
        cov += (sample.position - mean_p) * dt;
    }
    if (var_t < 1e-9) {
        return false;
    }
    velocity = cov * (1.0 / var_t);
    position = mean_p + velocity * (time - mean_t);

    residual = 0.0;
    for (const Sample& sample : history_) {
        const cv::Point3d predicted = position + velocity * (sample.time - time);
        residual += cv::norm(sample.position - predicted);
    }
    residual /= n;
    return true;
}

void PlateTracker::push(double time, const cv::Point3d& position) {
    history_.push_back({time, position});
}

void PlateTracker::prune(double time) {
    while (history_.size() > kWindowSize) {
        history_.pop_front();
    }
    while (!history_.empty() && history_.front().time < time - kWindowSeconds) {
        history_.pop_front();
    }
}

namespace {

// 把误差拆成"沿视线"和"垂直视线"两个分量。
void splitError(const cv::Point3d& delta, const cv::Point3d& line_of_sight,
                double& radial, double& tangential) {
    const double norm = cv::norm(line_of_sight);
    if (norm < 1e-9) {
        radial = cv::norm(delta);
        tangential = 0.0;
        return;
    }
    const cv::Point3d unit = line_of_sight * (1.0 / norm);
    const double along = delta.dot(unit);
    radial = std::abs(along);
    tangential = cv::norm(delta - unit * along);
}

}  // namespace

PlateTracker::Result PlateTracker::update(double time,
                                          const std::vector<cv::Point3d>& candidates,
                                          const cv::Point3d& origin) {
    Result result;

    if (candidates.empty()) {
        // 这一帧什么都没看到。先别急着丢历史：命中后灯条会灭 50 ms，
        // 那期间"看不见"是正常的。超过 kTimeout 才真的放弃。
        if (last_measure_ >= 0.0 && time - last_measure_ > kTimeout) {
            reset();
        }
        return result;
    }

    // 1) 预测"这一帧它应该在哪"。
    cv::Point3d predicted(0.0, 0.0, 0.0);
    bool have_prediction = false;
    {
        cv::Point3d position;
        cv::Point3d velocity;
        double residual = 0.0;
        if (history_.size() >= 2) {
            have_prediction = fit(time, position, velocity, residual);
        } else if (!history_.empty()) {
            position = history_.back().position +
                       velocity_ * (time - history_.back().time);
            have_prediction = true;
        }
        predicted = position;
    }

    // 2) 用预测位置做关联。优先留住已经在跟的那块板。
    int best = -1;
    if (have_prediction) {
        // 视线方向用"预测位置 - 云台旋转中心"：
        // 深度误差沿着它，横向误差垂直于它。
        const cv::Point3d line_of_sight = predicted - origin;
        double best_cost = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            double radial = 0.0;
            double tangential = 0.0;
            splitError(candidates[i] - predicted, line_of_sight, radial, tangential);
            if (radial > kRadialGate || tangential > kTangentialGate) {
                continue;
            }
            // 归一化之后比大小，免得某一维把另一维吃掉。
            const double cost = radial / kRadialGate + tangential / kTangentialGate;
            if (cost < best_cost) {
                best_cost = cost;
                best = static_cast<int>(i);
            }
        }
    }

    bool switched = false;
    if (best < 0) {
        // 有候选、但都不在门限内：要么是第一次锁定，要么旧板转走了、
        // 现在是另一块板。
        //
        // 【关键】这里不能像"等它转回来"那样处理：自转 4 rad/s 时同一块板
        // 要 1.5 s 才转回来，那期间等于瞎了。所以立刻在新板上重开一条轨。
        // 速度沿用手上这份——四块板装在同一台车上，刚体速度大致可以迁移，
        // 这样换板后不用等两三帧才敢做提前量。
        best = 0;
        switched = !history_.empty();
        history_.clear();
        // 平滑状态不能跨板继承（两块板的位置本来就不同），
        // 但速度可以——刚体速度对四块板是同一份。
        smoothed_valid_ = false;
    }

    // 低通：alpha 由帧间隔和 kSmoothTau 定，帧率变了也不用重调。
    const cv::Point3d measured = candidates[static_cast<std::size_t>(best)];
    if (!smoothed_valid_) {
        smoothed_ = measured;
        smoothed_valid_ = true;
    } else {
        const double gap = (last_measure_ >= 0.0) ? std::max(time - last_measure_, 1e-3)
                                                  : 1.0 / 60.0;
        const double alpha = gap / (kSmoothTau + gap);
        smoothed_ = smoothed_ + (measured - smoothed_) * alpha;
    }

    push(time, smoothed_);
    prune(time);
    last_measure_ = time;

    cv::Point3d position;
    cv::Point3d velocity;
    double residual = 0.0;
    // 评估点往后挪 kSmoothTau，正好抵消低通带来的群延迟。
    const bool fitted = fit(time + kSmoothTau, position, velocity, residual);

    result.valid = true;
    result.index = best;
    result.switched = switched;
    result.samples = static_cast<int>(history_.size());
    if (fitted) {
        result.position = position;
        result.velocity = velocity;
        result.residual = residual;
        velocity_ = velocity;
    } else {
        // 只有一帧观测：位置用观测值（补上群延迟），速度沿用上一次拟合的。
        result.position = smoothed_ +
                          (switched ? velocity_ * kSmoothTau : cv::Point3d(0.0, 0.0, 0.0));
        result.velocity = switched ? velocity_ : cv::Point3d(0.0, 0.0, 0.0);
        result.residual = 0.0;
    }
    return result;
}
