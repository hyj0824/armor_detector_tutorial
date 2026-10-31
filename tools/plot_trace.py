#!/usr/bin/env python3
"""把 --trace 生成的 JSONL 画成图。

用途：一眼看出问题出在"检测断了"、"目标在跳板"还是"控制跟不上"。

依赖：matplotlib（只用标准库 + matplotlib，没装就想办法装）
    python3 -m pip install matplotlib

用法：
    ./build/armor_aim --color=blue --fire --trace=/tmp/t.jsonl
    python3 tools/plot_trace.py /tmp/t.jsonl
    python3 tools/plot_trace.py /tmp/t.jsonl --save /tmp/t.png
"""

import argparse
import json
import sys


def load(path):
    rows = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                # 进程被强杀时最后一行可能是半条，忽略即可。
                continue
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("trace", help="--trace= 写出的 JSONL 文件")
    parser.add_argument("--save", help="存成图片而不是弹窗口")
    args = parser.parse_args()

    rows = load(args.trace)
    if not rows:
        parser.exit(1, "trace 里没有数据\n")

    try:
        import matplotlib
        if args.save:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        parser.exit(1, "需要 matplotlib: python3 -m pip install matplotlib\n")

    t = [r["t"] for r in rows]
    valid = [1 if r["est"]["valid"] else 0 for r in rows]
    bars = [r["det"]["bars"] for r in rows]
    armors = [r["det"]["armors"] for r in rows]
    err_yaw = [r["err"]["yaw"] for r in rows]
    err_pitch = [r["err"]["pitch"] for r in rows]
    fire = [1 if r["cmd"]["fire"] else 0 for r in rows]
    yaw_cmd = [r["cmd"]["yaw"] for r in rows]
    yaw_gim = [r["gim"]["yaw"] for r in rows]
    switch = sum(1 for a, b in zip(rows, rows[1:])
                 if a["est"]["idx"] != b["est"]["idx"] and b["est"]["valid"])

    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)

    axes[0].plot(t, bars, label="light bars")
    axes[0].plot(t, armors, label="armor candidates")
    axes[0].plot(t, valid, "k--", label="target valid")
    axes[0].set_ylabel("count")
    axes[0].legend(loc="upper right")
    axes[0].set_title("detection / association")

    axes[1].plot(t, [v * 57.2958 for v in err_yaw], label="yaw error")
    axes[1].plot(t, [v * 57.2958 for v in err_pitch], label="pitch error")
    axes[1].axhline(0, color="k", linewidth=0.6)
    axes[1].set_ylabel("deg")
    axes[1].legend(loc="upper right")
    axes[1].set_title("aim error (command - gimbal)")

    axes[2].plot(t, [v * 57.2958 for v in yaw_cmd], label="command yaw")
    axes[2].plot(t, [v * 57.2958 for v in yaw_gim], label="gimbal yaw")
    axes[2].set_ylabel("deg")
    axes[2].legend(loc="upper right")
    axes[2].set_title("yaw: command vs actual")

    axes[3].step(t, fire, where="post", label="fire")
    axes[3].set_ylabel("fire")
    axes[3].set_xlabel("sim time (s)")
    axes[3].legend(loc="upper right")
    axes[3].set_title("fire gate")

    fig.suptitle("armor_aim trace  (armor switches: %d)" % switch)
    fig.tight_layout()

    print("frames           :", len(rows))
    print("target rate      : %.3f" % (sum(valid) / len(rows)))
    print("armor switches   :", switch)
    print("mean |yaw err|   : %.2f deg" % (sum(abs(v) for v in err_yaw) / len(rows) * 57.2958))
    print("mean |pitch err| : %.2f deg" % (sum(abs(v) for v in err_pitch) / len(rows) * 57.2958))
    print("fire frames      :", sum(fire))

    if args.save:
        fig.savefig(args.save, dpi=110)
        print("saved:", args.save)
    else:
        plt.show()


if __name__ == "__main__":
    sys.exit(main())
