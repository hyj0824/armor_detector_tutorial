// app/main.cpp
// 主循环。它的职责只有三件事：
//   1) 从采集层拿 Observation
//   2) 交给 pipeline 走一遍（检测 -> 估计 -> 指令）
//   3) 把指令交给采集层，顺便写 trace / 画窗口 / 记统计
//
// 主循环里不应该出现任何算法细节。这样换采集源、换算法都不用动它。
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
#include "io/record.h"
#include "io/replay_capture.h"
#include "io/sim_capture.h"
#include "pipeline/pipeline.h"
#include "telemetry/trace.h"
#include "vision/utils.h"

namespace {

using Clock = std::chrono::steady_clock;

volatile std::sig_atomic_t g_exit = 0;

void onSignal(int) { g_exit = 1; }

constexpr double kRadToDeg = 180.0 / CV_PI;

// 超过这么久没读到新观测，就认为目标已经丢了，停控停火。
// 仿真的指令看门狗是 250 ms（超过它仿真会自己把指令降级），所以这里必须更小。
constexpr double kObservationTimeout = 0.20;

// 把中间结果画出来。图上一共有四层信息，看的时候按顺序排查：
//   mask -> 灯条 -> 候选 -> 选中目标
void drawDebug(cv::Mat& frame, const Observation& observation,
               const Detection& detection, const TargetEstimate& estimate,
               const Command& command) {
    for (const LightBar& bar : detection.light_bars) {
        utils::drawRotatedRect(frame, bar.rect, cv::Scalar(0, 255, 255), 1);
    }
    for (const Armor& armor : detection.armors) {
        cv::rectangle(frame, armor.boundingBox(), cv::Scalar(0, 160, 0), 1);
    }
    if (estimate.valid) {
        cv::rectangle(frame, detection.armors[static_cast<std::size_t>(estimate.armor_index)]
                                 .boundingBox(),
                      cv::Scalar(0, 255, 0), 2);
        cv::drawMarker(frame, estimate.pixel, cv::Scalar(0, 255, 0),
                       cv::MARKER_CROSS, 20, 1);
    }

    const cv::Point center(frame.cols / 2, frame.rows / 2);
    cv::drawMarker(frame, center, cv::Scalar(255, 255, 255), cv::MARKER_CROSS, 24, 1);

    const double error_yaw =
        estimate.valid ? utils::wrapPi(estimate.yaw - observation.gimbal_yaw) : 0.0;
    const double error_pitch =
        estimate.valid ? estimate.pitch - observation.gimbal_pitch : 0.0;

    std::ostringstream error_text;
    error_text << std::fixed << std::setprecision(2)
               << "aim error: yaw=" << error_yaw * kRadToDeg
               << " pitch=" << error_pitch * kRadToDeg << " deg";

    std::vector<std::string> lines = {
        "bars=" + std::to_string(detection.light_bars.size()) +
            " armors=" + std::to_string(detection.armors.size()) +
            " detect=" + std::to_string(static_cast<int>(detection.detect_ms)) + "ms",
        estimate.valid ? "target: yaw=" + std::to_string(estimate.yaw * kRadToDeg) +
                             " pitch=" + std::to_string(estimate.pitch * kRadToDeg)
                       : std::string("target: none"),
        error_text.str(),
        command.fire ? "FIRE" : "",
    };
    for (std::size_t i = 0; i < lines.size(); ++i) {
        cv::putText(frame, lines[i], cv::Point(12, 28 + 26 * static_cast<int>(i)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 3);
        cv::putText(frame, lines[i], cv::Point(12, 28 + 26 * static_cast<int>(i)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 1);
    }
}

std::string jsonNumberOrDash(const nlohmann::json& object, const char* key) {
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
        Pipeline pipeline(pipeline_config);

        std::unique_ptr<ICapture> capture;
        if (!options.replay.empty()) {
            capture = std::make_unique<ReplayCapture>(options.replay);
        } else {
            capture = std::make_unique<SimCapture>(options.channel);
        }

        Trace trace(options.trace);
        std::unique_ptr<FrameRecorder> recorder;
        if (!options.record.empty()) {
            recorder = std::make_unique<FrameRecorder>(options.record);
        }

        std::signal(SIGINT, onSignal);
        std::signal(SIGTERM, onSignal);

        std::cout << "[input] " << capture->describe() << '\n';
        if (capture->controllable() && !options.allow_fire) {
            std::cout << "[note] 未加 --fire，只瞄准不开火（先看角度，再看命中）\n";
        }

        const auto begin = Clock::now();
        auto next_send = begin;

        Command last_command;
        bool have_command = false;
        // 最后一次"真的读到一帧"的时刻。指令按 50 Hz 下发，但帧率更低，
        // 中间帧复用旧指令——所以必须知道这条旧指令到底有多旧。
        auto last_frame = begin;
        bool have_frame = false;
        std::string session_id;

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
                // 仿真重启会换 session_id。不清理的话会拿着上一次的目标继续发指令。
                if (!session_id.empty() && observation.session_id != session_id) {
                    pipeline.reset();
                    have_command = false;
                }
                session_id = observation.session_id;
                last_frame = Clock::now();
                have_frame = true;

                const Detection detection = pipeline.detect(observation.bgr);
                const TargetEstimate estimate =
                    pipeline.estimate(observation, detection);
                const Command command = pipeline.command(observation, estimate);

                last_command = command;
                have_command = true;

                detect_ms_sum += detection.detect_ms;
                if (estimate.valid) {
                    ++frames_with_target;
                    const double error =
                        std::hypot(utils::wrapPi(estimate.yaw - observation.gimbal_yaw),
                                   estimate.pitch - observation.gimbal_pitch);
                    error_sum += error;
                    ++error_count;
                }

                trace.frame(observation, detection, estimate, command);
                if (recorder) {
                    recorder->write(observation);
                }

                if (options.show_gui) {
                    cv::Mat canvas = observation.bgr.clone();
                    drawDebug(canvas, observation, detection, estimate, command);
                    cv::imshow("armor_aim", canvas);
                    cv::imshow("mask", detection.mask);
                    const int key = cv::waitKey(1);
                    if (key == 'q' || key == 27) {
                        break;
                    }
                }
            }

            // 控制频率和取图频率解耦：50 Hz 下发，中间帧复用上一条指令。
            //
            // 但"复用"有个前提：观测还是新的。超过 200 ms 没读到新帧就发空指令
            // （control=false, fire=false）——真机上那对应"丢目标后停火"，
            // 而仿真的指令看门狗是 250 ms，所以这个门限必须比它小。
            if (capture->controllable() && now >= next_send) {
                const bool fresh =
                    have_command && have_frame &&
                    std::chrono::duration<double>(now - last_frame).count() <
                        kObservationTimeout;
                capture->write(fresh ? last_command : Command{});
                next_send += std::chrono::milliseconds(20);
                if (next_send < now) {
                    next_send = now + std::chrono::milliseconds(20);
                }
            }

            if (capture->finished()) {
                break;
            }
            // 回放不需要限速；仿真靠 request/response 自然节流。
            if (capture->controllable()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }

        if (options.show_gui) {
            cv::destroyAllWindows();
        }

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "[self] frames=" << frames
                  << " with_target=" << frames_with_target
                  << " target_rate="
                  << (frames > 0 ? static_cast<double>(frames_with_target) / frames : 0.0)
                  << " mean_detect_ms="
                  << (frames > 0 ? detect_ms_sum / frames : 0.0)
                  << " mean_aim_error_deg="
                  << (error_count > 0 ? error_sum / error_count * kRadToDeg : 0.0)
                  << '\n';

        if (auto* sim = dynamic_cast<SimCapture*>(capture.get())) {
            const auto& stats = sim->stats();
            std::cout << "[sim] fired=" << jsonNumberOrDash(stats, "fired")
                      << " resolved=" << jsonNumberOrDash(stats, "resolved")
                      << " hit=" << jsonNumberOrDash(stats, "hit")
                      << " hit_rate=" << jsonNumberOrDash(stats, "hit_rate")
                      << " kills=" << jsonNumberOrDash(stats, "kills") << '\n';
            if (!sim->enabled()) {
                std::cout << "[warn] 仿真端没有打开\"本机自瞄控制\"，"
                             "指令会被丢弃。请用 ./run.sh -- --remote 启动。\n";
            }
        }

        return frames > 0 ? 0 : 2;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
