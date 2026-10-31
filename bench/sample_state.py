#!/usr/bin/env python3
"""持续采样 `state.json`，把"评估需要的一切"落到一个 JSONL 里。

为什么不再用 trace：
评分必须**不依赖被测二进制**。trace 是自瞄程序自己写的，它一改（或者根本不写），
评分就没了。而 `state.json` 是仿真发布的权威数据，里面已经有评分要的全部东西：

    tick 行  —— 云台真实姿态、四块装甲的真值位姿、仿真统计、画面内可见装甲数
    ev   行  —— 每发弹丸的 `shot`（开火瞬间的真实出膛角）和配对的 `impact`
                （命中 / 落地 / 超时 + 落点三维坐标 + 命中哪一块板）

于是"打得准不准"可以按**发**算，而且算的是炮口真正指的方向，不是我们请求的方向。

注意两件事：
1) `events` 在仿真侧只保留最近 128 条（`simulation.gd:_trim_events`），
   8 秒场景就有 100~160 条。所以必须**逐帧累积 + 按 (type, id) 去重**，
   不能等跑完读一次快照。
2) 这个进程只读 `state.json`，不碰 `client.lock`，所以可以和自瞄客户端同时跑。
   它拿到的是真值——**不许**给自瞄代码用，只给评分/调试用。

用法:
    python3 bench/sample_state.py --channel=default --out=out/state-static.jsonl
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


def tick_row(state):
    gimbal = state.get("gimbal", {})
    truth = state.get("truth", {})
    camera = state.get("camera", {})
    return {
        "k": "tick",
        "t": state.get("sim_time"),
        # 墙钟。和 t 一起用可以算出"仿真跑得比真实时间快还是慢"：
        # 渲染跟不上时 Godot 会少跑物理步，仿真时间就落后于墙钟。
        "wall": state.get("wall_unix_sec"),
        "gimbal": {
            "yaw": gimbal.get("yaw"),
            "pitch": gimbal.get("pitch"),
            "yaw_vel": gimbal.get("yaw_vel"),
            "pitch_vel": gimbal.get("pitch_vel"),
            "origin": gimbal.get("origin"),
        },
        "angle": truth.get("angle"),
        "center": truth.get("center"),
        "armors": truth.get("armors", []),
        # 仿真认为"完整落在画面内且灯亮着"的装甲数。它和检测器有没有检出来是两件事，
        # 差值的来源只能是检测/跟踪——这正是评分想看的。
        "visible": len(camera.get("armors", [])),
        "weapon": state.get("weapon", {}),
        "health": state.get("health", {}).get("current"),
        "stats": state.get("stats", {}),
        "paused": bool(state.get("paused", False)),
    }


def event_rows(state):
    """只返回 new 的事件行；调用方负责去重。

    `shot` 和 `impact` 复用同一个 `id`（弹丸 id），所以去重键必须是 (type, id)。
    `health_set` 用的是自己的计数器，同样落在 (type, id) 下互不干扰。
    """
    for event in state.get("events", []) or []:
        if not isinstance(event, dict):
            continue
        yield dict(event, k="ev")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--channel", default="default")
    parser.add_argument("--out", required=True)
    parser.add_argument("--hz", type=float, default=80.0,
                        help="轮询频率，默认 80（仿真发布是 60 Hz，留 1.3 倍余量）")
    args = parser.parse_args()

    path = os.path.join("/dev/shm", "rm_aim_sim_%s" % args.channel, "state.json")
    period = 1.0 / max(args.hz, 1.0)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    ticks = 0
    events = 0
    seen = set()
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
            key = state.get("paused"), state.get("publication")
            if key == last_key:
                time.sleep(period)
                continue
            last_key = key

            handle.write(json.dumps(tick_row(state)) + "\n")
            ticks += 1
            for event in event_rows(state):
                ident = (event.get("type"), event.get("id"))
                if ident in seen:
                    continue
                seen.add(ident)
                handle.write(json.dumps(event) + "\n")
                events += 1
            handle.flush()
            time.sleep(period)

    print(json.dumps({"type": "state_sample_result", "ticks": ticks,
                      "events": events, "out": args.out}), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
