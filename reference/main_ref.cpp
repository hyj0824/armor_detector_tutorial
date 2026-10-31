// reference/main_ref.cpp
// 参考实现的可执行入口。和 app/main.cpp 是同一套采集/记录/画图，
// 只是把 Pipeline 换成了 ReferencePipeline。
//
// 这个目标默认不参与构建，只有 -DBUILD_REFERENCE=ON 才会生成。
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "app/options.h"
#include "io/capture.h"
#include "io/sim_capture.h"
#include "pipeline/pipeline.h"
#include "reference/pipeline_ref.h"
#include "telemetry/trace.h"
#include "vision/utils.h"

namespace {

using Clock = std::chrono::steady_clock;

volatile std::sig_atomic_t g_exit = 0;

void onSignal(int) { g_exit = 1; }

std::string number(const nlohmann::json& object, const char* key) {
    if (!object.is_object() || !object.contains(key) || !object[key].is_number()) {
        return "-";
    }
    return std::to_string(object[key].get<double>());
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseOptions(argc, argv);

        PipelineConfig pipeline_config;
        pipeline_config.detector.target_color = options.color;
        pipeline_config.allow_fire = options.allow_fire;

        ReferencePipeline pipeline(pipeline_config);
        auto capture = std::make_unique<SimCapture>(options.channel);
        Trace trace(options.trace);

        std::signal(SIGINT, onSignal);
        std::signal(SIGTERM, onSignal);

        std::cout << "[input] " << capture->describe() << " (reference)\n";

        const auto begin = Clock::now();
        auto next_send = begin;
        Command last_command;
        bool have_command = false;

        int frames = 0;
        int frames_with_target = 0;
        double detect_ms_sum = 0.0;
        double error_sum = 0.0;
        int error_count = 0;

        while (g_exit == 0) {
            const auto now = Clock::now();
            if (options.duration > 0.0 &&
                std::chrono::duration<double>(now - begin).count() >= options.duration) {
                break;
            }
            if (options.max_frames > 0 && frames >= options.max_frames) {
                break;
            }

            capture->pump();

            Observation observation;
            if (capture->read(observation)) {
                ++frames;
                const Detection detection = pipeline.detect(observation.bgr);
                const TargetEstimate estimate = pipeline.estimate(observation, detection);
                const Command command = pipeline.command(observation, estimate);

                last_command = command;
                have_command = true;

                detect_ms_sum += detection.detect_ms;
                if (estimate.valid) {
                    ++frames_with_target;
                    error_sum += std::hypot(
                        utils::wrapPi(estimate.yaw - observation.gimbal_yaw),
                        estimate.pitch - observation.gimbal_pitch);
                    ++error_count;
                }
                trace.frame(observation, detection, estimate, command);

                if (options.show_gui) {
                    cv::Mat canvas = observation.bgr.clone();
                    if (estimate.valid) {
                        cv::drawMarker(canvas, estimate.pixel, cv::Scalar(0, 255, 0),
                                       cv::MARKER_CROSS, 20, 1);
                    }
                    cv::imshow("armor_aim_ref", canvas);
                    cv::imshow("mask", detection.mask);
                    const int key = cv::waitKey(1);
                    if (key == 'q' || key == 27) {
                        break;
                    }
                }
            }

            if (now >= next_send) {
                capture->write(have_command ? last_command : Command{});
                next_send += std::chrono::milliseconds(20);
                if (next_send < now) {
                    next_send = now + std::chrono::milliseconds(20);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        if (options.show_gui) {
            cv::destroyAllWindows();
        }

        constexpr double kRadToDeg = 180.0 / CV_PI;
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "[self] frames=" << frames << " with_target=" << frames_with_target
                  << " target_rate="
                  << (frames > 0 ? static_cast<double>(frames_with_target) / frames : 0.0)
                  << " mean_detect_ms="
                  << (frames > 0 ? detect_ms_sum / frames : 0.0)
                  << " mean_aim_error_deg="
                  << (error_count > 0 ? error_sum / error_count * kRadToDeg : 0.0)
                  << '\n';

        const auto& stats = capture->stats();
        std::cout << "[sim] fired=" << number(stats, "fired")
                  << " resolved=" << number(stats, "resolved")
                  << " hit=" << number(stats, "hit")
                  << " hit_rate=" << number(stats, "hit_rate")
                  << " kills=" << number(stats, "kills") << '\n';
        return frames > 0 ? 0 : 2;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
