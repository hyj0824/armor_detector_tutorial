#!/usr/bin/env python3
"""随机静止靶：每次跑之前随机一个靶位，专门用来测"接手/收敛"。

为什么需要它：
`static` 场景永远是同一个靶位，所以第一次跑和第一百次跑的初始误差一样。
把靶位随机掉，每一轮都是一次**阶跃输入**，可以直接量出：

    * 阶跃幅度  —— 开局时差多少度
    * 收敛时间  —— 从开局到误差进窗口并稳住
    * 超调      —— 收敛过程中反向偏了多少
    * 稳态误差  —— 最后 1 秒的平均偏差

这四件事才是"云台转得怎么样"，而不是"能不能命中"。

数据源是 `bench/truth.py` 写的 aim JSONL（逐 tick 已经对齐到理想弹道解），
**不依赖被测程序写任何日志**，所以自己写 pipeline 的同学也能直接用这个工具。

每次跑都重启仿真，所以 gimbal 都从 yaw=0 / pitch=0 开始，起点一致。
（想要"一轮里多次随机跳靶"，需要给仿真加 step 模式，见 README。）

用法:
    python3 bench/random_target.py --runs 6
    python3 bench/random_target.py --runs 6 --duration 6 --seed 7
"""

import argparse
import json
import math
import os
import random
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAD2DEG = 180.0 / math.pi
WINDOW_DEG = 0.6          # 与 pipeline.cpp 里的开火窗口一致
HOLD_S = 0.30             # 误差要连续稳住这么久才算收敛


def wrap(angle):
    """把角度归一化到 (-pi, pi]。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

ARMOR_Z = 0.2603703641430668


def write_scenario(path, rng, rotate_phase, distance_min, distance_max):
    distance = rng.uniform(distance_min, distance_max)
    # 横向偶尔超出水平视场（±29.5°），这样部分轮次会从"看不见"开始，
    # "接管时间"才有意义，而不是每轮都恰好已经看得见。
    lateral = rng.uniform(-2.2, 2.2)
    # 默认让 0 号装甲正面朝向云台，这样一轮里不会有"两块板争中心"的歧义，
    # 量到的就纯粹是云台阶跃响应。加 --rotation 才随机朝向。
    phase = (rng.uniform(0.0, 2.0 * math.pi) if rotate_phase
             else math.atan2(-lateral, -distance))
    robot = rng.choice(["infantry_3", "hero"])
    text = (
        "; 由 bench/random_target.py 生成，不要手改。\n"
        "[appearance]\n"
        'robot="%s"\n'
        "[target]\n"
        "center=Vector3(%.4f, %.4f, %.6f)\n"
        "translate=false\n"
        "rotate=false\n"
        "phase=%.6f\n"
        "max_health=10000\n"
    ) % (robot, distance, lateral, ARMOR_Z, phase)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(text)
    return {"distance": distance, "lateral": lateral, "phase": phase, "robot": robot}


def load_jsonl(path):
    rows = []
    try:
        with open(path, "r", encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
    except OSError:
        pass
    return rows


def analyse(rows, log_path, start=0.0):
    """输入 truth.py 写的 aim JSONL（逐 tick 已对齐到理想弹道解）。

    指标全部只靠云台角 + 真值：
        initial_deg   接管瞬间的误差幅度（阶跃幅度）
        settle_s      从开始到误差进窗口并稳住 HOLD_S
        overshoot_deg 收敛过程中的反向超调
        steady_deg    最后 1 秒的平均偏差
    """
    ticks = [r for r in rows if r.get("valid") and "ideal_yaw" in r]
    if not ticks:
        return {"ok": False, "reason": "全程没有可对齐的云台/目标"}

    for row in ticks:
        row["gyaw"] = wrap(row["yaw"] - row["ideal_yaw"])
        row["gpitch"] = row["pitch"] - row["ideal_pitch"]

    first = ticks[0]["t"]
    initial = math.hypot(ticks[0]["gyaw"], ticks[0]["gpitch"])
    window = WINDOW_DEG / RAD2DEG
    settled_at = None
    for i, row in enumerate(ticks):
        if abs(row["gyaw"]) > window or abs(row["gpitch"]) > window:
            continue
        end = row["t"] + HOLD_S
        hold = [r for r in ticks[i:] if r["t"] <= end]
        if hold and hold[-1]["t"] >= end - 0.1 and all(
                abs(r["gyaw"]) <= window and abs(r["gpitch"]) <= window
                for r in hold):
            settled_at = row["t"]
            break
    settle = (settled_at - first) if settled_at is not None else None

    # 超调：收敛过程中最大的一次反向偏差（以 yaw 为主，pitch 通常小得多）
    overshoot = 0.0
    if settled_at is not None:
        sign = 0.0
        for row in ticks:
            error = row["gyaw"]
            if abs(error) < window * 0.5:
                continue
            current = 1.0 if error > 0 else -1.0
            if sign == 0.0:
                sign = current
            elif current != sign:
                overshoot = max(overshoot, abs(error))
            if row["t"] >= settled_at:
                break

    tail = [r for r in ticks if r["t"] >= ticks[-1]["t"] - 1.0]
    steady = (sum(math.hypot(r["gyaw"], r["gpitch"]) for r in tail)
              / max(len(tail), 1))

    hit_rate = float("nan")
    fired = 0
    try:
        with open(log_path, "r", encoding="utf-8") as handle:
            for line in handle:
                if line.startswith("[sim]"):
                    for token in line.split():
                        if token.startswith("hit_rate="):
                            hit_rate = float(token.split("=")[1])
                        elif token.startswith("fired="):
                            fired = int(float(token.split("=")[1]))
    except OSError:
        pass

    return {
        "ok": True,
        "initial_deg": round(initial * RAD2DEG, 2),
        "settle_s": round(settle, 2) if settle is not None else None,
        "overshoot_deg": round(overshoot * RAD2DEG, 2),
        "steady_deg": round(steady * RAD2DEG, 2),
        "fired": fired,
        "hit_rate": hit_rate,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--runs", type=int, default=6)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--distance-min", type=float, default=4.0)
    parser.add_argument("--distance-max", type=float, default=5.5)
    parser.add_argument("--rotation", action="store_true",
                        help="随机装甲朝向（会引入目标选择难度，干扰纯云台响应测量）")
    args = parser.parse_args()

    rng = random.Random(args.seed)
    scenario = os.path.join(ROOT, "scenarios", "_random.cfg")
    results = []

    for index in range(args.runs):
        label = "random_%02d" % index
        info = write_scenario(scenario, rng, args.rotation,
                              args.distance_min, args.distance_max)
        print("[random] run %d/%d  dist=%.2fm lateral=%+.2fm robot=%s"
              % (index + 1, args.runs, info["distance"], info["lateral"], info["robot"]),
              flush=True)
        env = dict(os.environ, LABEL=label)
        with open(os.path.join(ROOT, "out", "bench-%s.log" % label), "w",
                  encoding="utf-8") as log:
            subprocess.run([os.path.join(ROOT, "bench", "run.sh"), "_random",
                            str(args.duration)],
                           cwd=ROOT, env=env, stdout=log,
                           stderr=subprocess.STDOUT, check=False)

        rows = load_jsonl(os.path.join(ROOT, "out", "aim-%s.jsonl" % label))
        metrics = analyse(rows, os.path.join(ROOT, "out", "client-%s.log" % label))
        metrics.update(info)
        metrics["label"] = label
        results.append(metrics)
        print("           -> %s" % json.dumps(metrics, ensure_ascii=False), flush=True)
        time.sleep(3.0)

    ok = [r for r in results if r.get("ok")]
    print()
    if ok:
        settles = [r["settle_s"] for r in ok if r["settle_s"] is not None]
        overshoots = [r["overshoot_deg"] for r in ok]
        steadies = [r["steady_deg"] for r in ok]
        hits = [r["hit_rate"] for r in ok if not math.isnan(r["hit_rate"])]

        print("成功接管: %d/%d" % (len(ok), len(results)))
        if settles:
            print("收敛时间: 平均 %.2fs  最大 %.2fs"
                  % (sum(settles) / len(settles), max(settles)))
        print("超调:     平均 %.2f°  最大 %.2f°"
              % (sum(overshoots) / len(overshoots), max(overshoots)))
        print("稳态误差: 平均 %.2f°  最大 %.2f°"
              % (sum(steadies) / len(steadies), max(steadies)))
        if hits:
            print("平均命中率: %.2f" % (sum(hits) / len(hits)))

        out = os.path.join(ROOT, "out", "random-summary.json")
        with open(out, "w", encoding="utf-8") as handle:
            json.dump(results, handle, ensure_ascii=False, indent=2)
        print("明细: %s" % out)
    else:
        print("全部失败")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
