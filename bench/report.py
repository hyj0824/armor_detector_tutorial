#!/usr/bin/env python3
"""把多次 bench 的结果汇总成一张表。

用途：
  * 学生自测时看"我这轮比上轮好在哪"；
  * 提交时把这张表贴进报告，助教不用一条条翻日志。

用法:
    bench/run.sh static 8
    bench/run.sh spin 8
    python3 bench/report.py                  # 读 out/ 下所有结果
    python3 bench/report.py --format=csv
"""

import argparse
import glob
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "out")

# 输出顺序固定，方便前后两轮对比
ORDER = ["static", "translate", "spin", "combined", "far", "lowlight"]


def parse_client_log(path):
    result = {}
    try:
        with open(path, "r", encoding="utf-8") as handle:
            for line in handle:
                if line.startswith("[self]"):
                    for key in ("frames", "with_target", "target_rate",
                                "mean_detect_ms", "mean_aim_error_deg"):
                        match = re.search(key + r"=([-\d.]+)", line)
                        if match:
                            result[key] = float(match.group(1))
                elif line.startswith("[sim]"):
                    for key in ("fired", "resolved", "hit", "hit_rate", "kills"):
                        match = re.search(key + r"=([-\d.]+)", line)
                        if match:
                            result[key] = float(match.group(1))
    except OSError:
        return None
    return result or None


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out-dir", default=OUT)
    parser.add_argument("--format", choices=["md", "csv"], default="md")
    parser.add_argument("--save", help="同时写到文件")
    args = parser.parse_args()

    labels = []
    for path in glob.glob(os.path.join(args.out_dir, "client-*.log")):
        label = os.path.basename(path)[len("client-"):-len(".log")]
        if label.startswith("_"):
            continue
        labels.append(label)
    if not labels:
        parser.exit(1, "out/ 里没有结果。先跑 bench/run.sh <场景> <秒数>\n")

    def rank(label):
        base = label.split("_")[0]
        return (ORDER.index(base) if base in ORDER else len(ORDER), label)

    labels.sort(key=rank)

    rows = []
    for label in labels:
        metrics = parse_client_log(os.path.join(args.out_dir, "client-%s.log" % label))
        if not metrics:
            continue
        aim_path = os.path.join(args.out_dir, "aim-%s.json" % label)
        if os.path.exists(aim_path):
            try:
                with open(aim_path, "r", encoding="utf-8") as handle:
                    aim = json.load(handle)
                # 用"弹丸到达时刻"的误差，而不是"当前时刻"的误差：
                # 提前量做对了的系统，在当前时刻看上去反而是偏的。
                metrics["aim_error_mean_deg"] = aim.get(
                    "impact_error_mean_deg", aim.get("aim_error_mean_deg"))
                metrics["switch_per_s"] = aim.get("switch_per_s")
                metrics["distance_m"] = aim.get("distance_m")
            except (OSError, ValueError):
                pass
        meta_path = os.path.join(args.out_dir, "meta-%s.json" % label)
        if os.path.exists(meta_path):
            try:
                with open(meta_path, "r", encoding="utf-8") as handle:
                    meta = json.load(handle)
                duration = float(meta.get("duration") or 0.0)
                if duration > 0 and "hit" in metrics:
                    # 每秒命中 = 精度 × 吞吐。只看 hit_rate 会被"很少开火但很准"骗过去。
                    metrics["hit_per_s"] = metrics["hit"] / duration
                metrics["binary"] = os.path.basename(meta.get("binary", ""))
            except (OSError, ValueError):
                pass
        metrics["label"] = label
        rows.append(metrics)

    if not rows:
        parser.exit(1, "解析不到任何 [self]/[sim] 行\n")

    columns = [
        ("label", "场景", "%s"),
        ("frames", "帧", "%.0f"),
        ("target_rate", "检出率", "%.2f"),
        ("hit_rate", "命中率", "%.2f"),
        ("fired", "发射", "%.0f"),
        ("hit_per_s", "命中/s", "%.2f"),
        ("switch_per_s", "换板/s", "%.2f"),
        ("aim_error_mean_deg", "落点误差", "%.2f"),
        ("mean_detect_ms", "检测ms", "%.1f"),
    ]

    if args.format == "csv":
        lines = [",".join(name for _, name, _ in columns)]
        for row in rows:
            cells = []
            for key, _, fmt in columns:
                value = row.get(key)
                cells.append(str(value) if key == "label"
                             else ("-" if value is None else fmt % value))
            lines.append(",".join(cells))
        text = "\n".join(lines) + "\n"
    else:
        header = "| " + " | ".join(name for _, name, _ in columns) + " |"
        divider = "|" + "|".join(["---"] * len(columns)) + "|"
        lines = [header, divider]
        for row in rows:
            cells = []
            for key, _, fmt in columns:
                value = row.get(key)
                cells.append(str(value) if key == "label"
                             else ("—" if value is None else fmt % value))
            lines.append("| " + " | ".join(cells) + " |")
        lines.append("")
        lines.append("检出率 = 有目标的帧 / 总帧；命中率 = hit / resolved（仿真统计）。")
        lines.append("命中/s = hit / 运行秒数 —— 同时体现精度和吞吐，**这个才是主指标**。")
        text = "\n".join(lines) + "\n"

    sys.stdout.write(text)
    if args.save:
        with open(args.save, "w", encoding="utf-8") as handle:
            handle.write(text)
        print("写入 %s" % args.save, file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
