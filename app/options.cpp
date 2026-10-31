// app/options.cpp
#include "app/options.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

void printUsage(const char* program) {
    std::cout
        << "用法: " << program << " [options]\n"
        << "\n"
        << "  --channel=NAME     仿真通道，默认 default\n"
        << "  --color=red|blue   目标颜色，默认 blue\n"
        << "  --fire             允许开火（默认不开火，只看瞄准）\n"
        << "  --duration=SEC     运行时长，0 表示不限\n"
        << "  --max-frames=N     处理 N 帧后退出，0 表示不限\n"
        << "  --no-gui           不开窗口（远程/无显示器时用）\n"
        << "  --trace=PATH       逐帧写 JSONL，配合 tools/plot_trace.py\n"
        << "  --record=DIR       把帧和云台姿态落盘\n"
        << "  --replay=DIR       离线回放录制目录，不连仿真\n"
        << "  --help             显示本帮助\n"
        << "\n"
        << "例:\n"
        << "  ./build/armor_aim --color=blue --no-gui\n"
        << "  ./build/armor_aim --color=blue --fire --trace=/tmp/t.jsonl\n"
        << "  ./build/armor_aim --replay=/tmp/replay1 --no-gui\n";
}

namespace {

std::string takeValue(const std::string& arg, const std::string& key) {
    return arg.substr(key.size());
}

}  // namespace

Options parseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--no-gui") {
            options.show_gui = false;
        } else if (arg == "--fire") {
            options.allow_fire = true;
        } else if (arg.rfind("--channel=", 0) == 0) {
            options.channel = takeValue(arg, "--channel=");
        } else if (arg.rfind("--trace=", 0) == 0) {
            options.trace = takeValue(arg, "--trace=");
        } else if (arg.rfind("--record=", 0) == 0) {
            options.record = takeValue(arg, "--record=");
        } else if (arg.rfind("--replay=", 0) == 0) {
            options.replay = takeValue(arg, "--replay=");
        } else if (arg.rfind("--color=", 0) == 0) {
            const std::string value = takeValue(arg, "--color=");
            if (value == "red") {
                options.color = TargetColor::Red;
            } else if (value == "blue") {
                options.color = TargetColor::Blue;
            } else {
                throw std::invalid_argument("--color 只能是 red 或 blue");
            }
        } else if (arg.rfind("--duration=", 0) == 0) {
            const std::string value = takeValue(arg, "--duration=");
            std::size_t used = 0;
            options.duration = std::stod(value, &used);
            if (used != value.size() || !std::isfinite(options.duration) ||
                options.duration < 0.0) {
                throw std::invalid_argument("--duration 必须是非负数");
            }
        } else if (arg.rfind("--max-frames=", 0) == 0) {
            const std::string value = takeValue(arg, "--max-frames=");
            std::size_t used = 0;
            const double number = std::stod(value, &used);
            if (used != value.size() || !std::isfinite(number) || number < 0.0 ||
                number != std::floor(number) || number > 1e9) {
                throw std::invalid_argument("--max-frames 必须是非负整数");
            }
            options.max_frames = static_cast<int>(number);
        } else {
            throw std::invalid_argument("未知参数: " + arg + "（--help 查看用法）");
        }
    }
    if (!options.replay.empty() && !options.record.empty()) {
        throw std::invalid_argument("--replay 与 --record 不能同时使用");
    }
    return options;
}
