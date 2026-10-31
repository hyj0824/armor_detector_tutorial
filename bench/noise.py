#!/usr/bin/env python3
"""同一个场景重复跑 N 次，量"测量噪声"。

为什么必须先有这个：
改进 A 让命中率从 0.17 涨到 0.21，这是真的变好了，还是这一轮运气好？
不知道噪声，就没法回答，所有 A/B 对比都是自欺欺人。

用法:
    python3 bench/noise.py translate --runs 5
    python3 bench/noise.py spin --runs 5 --duration 8
"""

import argparse
import json
import os
import statistics
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "out", "noise")

KEYS = ["target_rate", "hit_rate", "mean_aim_error_deg", "mean_detect_ms", "frames"]


def read_metrics(label):
    metrics = {}
    path = os.path.join(OUT, "client-%s.log" % label)
    try:
        with open(path, "r", encoding="utf-8") as handle:
            for line in handle:
                if not (line.startswith("[self]") or line.startswith("[sim]")):
                    continue
                for token in line.split():
                    key, sep, value = token.partition("=")
                    if not sep:
                        continue
                    try:
                        metrics[key] = float(value)
                    except ValueError:
                        pass
    except OSError:
        pass
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenario")
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--duration", type=float, default=8.0)
    args = parser.parse_args()

    os.makedirs(OUT, exist_ok=True)
    samples = []
    for index in range(args.runs):
        label = "%s_r%02d" % (args.scenario, index)
        env = dict(os.environ, LABEL=label, OUT_DIR=OUT)
        print("[noise] %d/%d %s" % (index + 1, args.runs, label), flush=True)
        subprocess.run([os.path.join(ROOT, "bench", "run.sh"), args.scenario,
                        str(args.duration)],
                       cwd=ROOT, env=env, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, check=False)
        metrics = read_metrics(label)
        if metrics:
            samples.append(metrics)
        time.sleep(3.0)

    if not samples:
        parser.exit(1, "没有采到任何结果\n")

    print()
    print("%-22s %8s %8s %8s %8s" % ("指标", "平均", "标准差", "最小", "最大"))
    print("-" * 60)
    summary = {}
    for key in KEYS:
        values = [m[key] for m in samples if key in m]
        if not values:
            continue
        mean = statistics.fmean(values)
        std = statistics.stdev(values) if len(values) > 1 else 0.0
        summary[key] = {"mean": mean, "std": std,
                        "min": min(values), "max": max(values)}
        print("%-22s %8.3f %8.3f %8.3f %8.3f"
              % (key, mean, std, min(values), max(values)))

    print("\n判据：两次运行的差异要明显大于 2 倍标准差，才算真的改进了。")
    path = os.path.join(OUT, "noise-%s.json" % args.scenario)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump({"scenario": args.scenario, "runs": len(samples),
                   "summary": summary}, handle, ensure_ascii=False, indent=2)
    print("明细: %s" % path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
