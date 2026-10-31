#!/usr/bin/env python3
"""扫参数，画出"检测工作区间"。

最有用的两张图：
  * 距离 -> 检出率：告诉你检测的上/下边界在哪、边界是不是很脆
  * 曝光 -> 检出率：告诉你"曝光越低越难"这个直觉对不对

用法:
    python3 bench/sweep.py distance --values 3,3.5,4,4.5,5,5.5,6,7
    python3 bench/sweep.py exposure --values 0.1,0.4,1.0,2.5

每次都会重启仿真（状态必须干净），所以一个点大约 25 秒。
"""

import argparse
import json
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "out", "sweep")
SCENARIO = os.path.join(ROOT, "scenarios", "_sweep.cfg")
TARGET_Z = 0.2603703641430668


def write_config(kind, value):
    if kind == "distance":
        text = ("; 由 bench/sweep.py 生成，不要手改\n[target]\n"
                "center=Vector3(%.4f, 0, %.6f)\n"
                "translate=false\nrotate=false\nmax_health=10000\n"
                % (value, TARGET_Z))
    else:
        text = ("; 由 bench/sweep.py 生成，不要手改\n[camera]\n"
                "brightness_gain=%.4f\n"
                "[target]\ncenter=Vector3(4, 0, %.6f)\n"
                "translate=false\nrotate=false\nmax_health=10000\n"
                % (value, TARGET_Z))
    with open(SCENARIO, "w", encoding="utf-8") as handle:
        handle.write(text)


def read_client_metrics(label):
    metrics = {}
    path = os.path.join(OUT, "client-%s.log" % label)
    try:
        with open(path, "r", encoding="utf-8") as handle:
            for line in handle:
                if not line.startswith("[self]"):
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
    parser.add_argument("kind", choices=["distance", "exposure"])
    parser.add_argument("--values", required=True, help="逗号分隔，例如 3,4,5,6")
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args()

    values = [float(v) for v in args.values.split(",") if v.strip()]
    if not values:
        parser.error("--values 是空的")
    os.makedirs(OUT, exist_ok=True)

    unit = "m" if args.kind == "distance" else "x"
    print("%-12s %10s %10s %10s %10s" % (args.kind, "检出率", "有目标帧", "瞄准误差", "检测ms"))
    print("-" * 58)

    rows = []
    for value in values:
        write_config(args.kind, value)
        label = "sweep_%s_%s" % (args.kind, ("%g" % value).replace(".", "p"))
        env = dict(os.environ, LABEL=label, OUT_DIR=OUT)
        subprocess.run([os.path.join(ROOT, "bench", "run.sh"), "_sweep",
                        str(args.duration)],
                       cwd=ROOT, env=env, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, check=False)

        metrics = read_client_metrics(label)
        rate = metrics.get("target_rate")
        frames = metrics.get("with_target")
        error = metrics.get("mean_aim_error_deg")
        cost = metrics.get("mean_detect_ms")
        rows.append({"value": value, "target_rate": rate,
                     "with_target": frames, "aim_error_deg": error,
                     "detect_ms": cost})
        print("%-12s %10s %10s %10s %10s"
              % ("%.2f%s" % (value, unit),
                 "-" if rate is None else "%.2f" % rate,
                 "-" if frames is None else "%.0f" % frames,
                 "-" if error is None else "%.2f" % error,
                 "-" if cost is None else "%.1f" % cost))
        time.sleep(3.0)

    summary = os.path.join(OUT, "sweep-%s.json" % args.kind)
    with open(summary, "w", encoding="utf-8") as handle:
        json.dump(rows, handle, ensure_ascii=False, indent=2)
    print("\n明细: %s" % summary)
    try:
        os.remove(SCENARIO)
    except OSError:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
