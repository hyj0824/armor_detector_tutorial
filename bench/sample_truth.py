#!/usr/bin/env python3
"""持续采样 state.json 里的真值，写成 JSONL。

为什么要单独一个进程：
`state.json` 是"最新值邮箱"，只保留最后一次发布。要把它和逐帧 trace 对齐，
就必须按接近状态发布频率（默认 60 Hz）把历史记下来。

这个进程只读 `state.json`，不碰 `client.lock`，所以可以和自瞄客户端同时跑。
但它拿到的是真值——**不能**把它给学生代码用，只给评分/调试用。

用法：
    python3 bench/sample_truth.py --channel=default --out=out/truth-static.jsonl
Ctrl+C 结束。
"""

import argparse
import json
import os
import signal
import sys
import time

RUNNING = True


def on_signal(_signum, _frame):
    global RUNNING
    RUNNING = False


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--channel", default="default")
    parser.add_argument("--out", required=True)
    parser.add_argument("--hz", type=float, default=60.0)
    args = parser.parse_args()

    path = os.path.join("/dev/shm", "rm_aim_sim_%s" % args.channel, "state.json")
    period = 1.0 / args.hz
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    count = 0
    last_key = None
    with open(args.out, "w", encoding="utf-8") as handle:
        while RUNNING:
            try:
                with open(path, "r", encoding="utf-8") as source:
                    state = json.load(source)
            except (OSError, ValueError):
                time.sleep(period)
                continue

            # 同一版本的状态只记一次（publication 是发布序号）。
            key = state.get("publication")
            if key == last_key:
                time.sleep(period)
                continue
            last_key = key

            gimbal = state.get("gimbal", {})
            truth = state.get("truth", {})
            line = {
                "t": state.get("sim_time"),
                "gimbal": {"yaw": gimbal.get("yaw"), "pitch": gimbal.get("pitch"),
                           "origin": gimbal.get("origin")},
                "angle": truth.get("angle"),
                "weapon": state.get("weapon", {}),
                "armors": truth.get("armors", []),
                "health": state.get("health", {}).get("current"),
                "stats": state.get("stats", {}),
            }
            handle.write(json.dumps(line) + "\n")
            handle.flush()
            count += 1
            time.sleep(period)

    print(json.dumps({"type": "truth_sample_result", "samples": count, "out": args.out}),
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
