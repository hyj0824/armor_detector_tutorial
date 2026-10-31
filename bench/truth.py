#!/usr/bin/env python3
"""把 trace（学生/基线的逐帧记录）和真值对齐，算出"到底瞄得准不准"。

trace 里的 `err` 是**指令角减云台实际角**，只能说明"云台听不听话"，
不能说明"瞄的地方对不对"。要判断后者必须引入真值。

做法：
  * 用 `sim_time` 把 trace 的每一帧和最近的 truth 采样配对；
  * 用真值里四块装甲的三维中心，算"指令方向"到最近装甲的夹角；
  * 换板次数、可见装甲集合变化，用来观察目标关联是否稳定。

这个脚本属于评分/调试工具，**学生代码不许读 truth**。
"""

import argparse
import json
import math
import statistics
import sys

RAD2DEG = 180.0 / math.pi


def load_jsonl(path):
    rows = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def forward(yaw, pitch):
    return (math.cos(pitch) * math.cos(yaw),
            math.cos(pitch) * math.sin(yaw),
            -math.sin(pitch))


def angle_between(a, b):
    dot = sum(x * y for x, y in zip(a, b))
    norm_a = math.sqrt(sum(x * x for x in a))
    norm_b = math.sqrt(sum(x * x for x in b))
    if norm_a < 1e-9 or norm_b < 1e-9:
        return float("nan")
    ratio = max(-1.0, min(1.0, dot / (norm_a * norm_b)))
    return math.acos(ratio)


def nearest_truth(truth_rows, t):
    """二分找时间上最近的采样。"""
    lo, hi = 0, len(truth_rows) - 1
    if hi < 0:
        return None
    while lo < hi:
        mid = (lo + hi) // 2
        if truth_rows[mid]["t"] < t:
            lo = mid + 1
        else:
            hi = mid
    best = truth_rows[lo]
    if lo > 0 and abs(truth_rows[lo - 1]["t"] - t) < abs(best["t"] - t):
        best = truth_rows[lo - 1]
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("trace")
    parser.add_argument("--truth", required=True, help="sample_truth.py 的输出")
    parser.add_argument("--out", help="把逐帧结果写成 JSONL")
    args = parser.parse_args()

    trace = load_jsonl(args.trace)
    truth = sorted(load_jsonl(args.truth), key=lambda r: r["t"])
    if not trace or not truth:
        parser.exit(1, "trace 或 truth 是空的\n")

    rows = []
    for sample in trace:
        if not sample["est"]["valid"] or not sample["cmd"]["control"]:
            rows.append({"t": sample["t"], "valid": False})
            continue
        reference = nearest_truth(truth, sample["t"])
        armors = reference["armors"]
        if not armors:
            rows.append({"t": sample["t"], "valid": False})
            continue

        origin = reference["gimbal"]["origin"]
        direction = forward(sample["cmd"]["yaw"], sample["cmd"]["pitch"])

        errors = []
        for armor in armors:
            delta = [armor["center"][i] - origin[i] for i in range(3)]
            errors.append(angle_between(direction, delta) * RAD2DEG)

        # 可见装甲：外法向朝向云台的那几块。
        # 为什么必须区分：靶车对向的两块板在图像上投影几乎重叠（4 m 处相差不到 1 度），
        # 拿“角度最近的装甲”去评分会随机落到背面那块上，指标就没意义了。
        visible = [i for i, armor in enumerate(armors)
                   if sum(armor["normal"][k] * (origin[k] - armor["center"][k])
                          for k in range(3)) > 0.0]
        pool = visible if visible else list(range(len(armors)))
        best = min(pool, key=lambda i: errors[i])
        nearest_any = min(range(len(errors)), key=lambda i: errors[i])

        # 提前量补偿过的瞄准，必须和"弹丸到达时"的目标比，而不是"现在"的目标比。
        # 否则一个正确的提前量会显示成一处误差。
        speed = float(reference.get("weapon", {}).get("speed") or 23.0)
        gravity = float(reference.get("weapon", {}).get("gravity") or 9.7833)
        flight = float(sample["est"].get("dist") or 0.0) / max(speed, 1.0)
        if flight <= 0.0:
            flight = math.sqrt(sum((armors[best]["center"][i] - origin[i]) ** 2
                                   for i in range(3))) / max(speed, 1.0)
        impact_ref = nearest_truth(truth, sample["t"] + flight)
        impact_error = errors[best]
        if impact_ref and impact_ref["armors"]:
            o2 = impact_ref["gimbal"]["origin"]
            arm2 = min(impact_ref["armors"],
                       key=lambda a: abs(a.get("id", 0) - armors[best].get("id", 0)))
            d2 = [arm2["center"][i] - o2[i] for i in range(3)]
            impact_error = angle_between(direction, d2) * RAD2DEG
        drop_deg = math.degrees(math.atan2(0.5 * gravity * flight * flight,
                                           max(math.hypot(direction[0], direction[1]) * 10.0, 1e-6)))

        rows.append({
            "t": sample["t"],
            "valid": True,
            "aim_error_deg": errors[best],
            "impact_error_deg": impact_error,
            "flight": flight,
            "drop_deg": drop_deg,
            "nearest_id": best,
            "nearest_any_id": nearest_any,
            "visible": visible,
            "distance_m": math.sqrt(sum((armors[best]["center"][i] - origin[i]) ** 2
                                        for i in range(3))),
            "fire": sample["cmd"]["fire"],
        })

    valid = [r for r in rows if r["valid"]]
    if not valid:
        parser.exit(1, "没有有效帧可以和真值对齐\n")

    errors = sorted(r["aim_error_deg"] for r in valid)
    impact = sorted(r["impact_error_deg"] for r in valid)
    switches = sum(1 for a, b in zip(valid, valid[1:]) if a["nearest_id"] != b["nearest_id"])
    fire_frames = sum(1 for r in valid if r["fire"])
    span = max(valid[-1]["t"] - valid[0]["t"], 1e-6)

    summary = {
        "frames": len(rows),
        "valid": len(valid),
        "aim_error_mean_deg": round(statistics.fmean(errors), 3),
        "aim_error_median_deg": round(statistics.median(errors), 3),
        "aim_error_p95_deg": round(errors[max(0, int(len(errors) * 0.95) - 1)], 3),
        "aim_error_max_deg": round(errors[-1], 3),
        "impact_error_mean_deg": round(statistics.fmean(impact), 3),
        "impact_error_p95_deg": round(impact[max(0, int(len(impact) * 0.95) - 1)], 3),
        "switch_per_s": round(switches / span, 2),
        "distance_m": round(statistics.fmean(r["distance_m"] for r in valid), 2),
        "fire_ratio": round(fire_frames / len(valid), 3),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))

    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            for row in rows:
                handle.write(json.dumps(row) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
