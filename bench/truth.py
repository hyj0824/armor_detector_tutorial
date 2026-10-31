#!/usr/bin/env python3
"""把 `bench/sample_state.py` 的采样变成评分指标。

这一版**不读 trace**。数据源只有仿真发布的 `state.json`，所以：

  * 不依赖被测二进制写不写日志 —— 连一行都不打印的程序也能被评分；
  * 拿不到"我们请求了什么"，只能拿到"云台实际指向哪、弹丸落到哪"。
    这对评分是好事：伺服滞后、指令被丢弃这些事不会污染指标，
    它们只能通过命中率表现出来。

四个指标，**别混着看**：

  track_error_deg   逐 tick：云台实际指向 vs **理想弹道解**。
                    这是"解算+跟踪做得对不对"的总误差，含伺服滞后。
                    为什么不是"vs 装甲中心"：正确的弹道解**故意**瞄高
                    1.5~2.6°（重力下坠），拿直线去比会把对的判成错的。
  offset_cm         逐发：命中落点到装甲板中心的板面内距离。
                    这是"打得准不准"的直接测量，只对命中发有意义。
  hit_rate          仿真统计的命中率（authoritative）。
  miss_kind         没打中的分几种：ground 打低了 / body 打机身上了 /
                    expired 提前到期（多半是距离估远或弹速算错）。

用法:
    python3 bench/truth.py out/state-far.jsonl --out out/aim-far.jsonl
"""

import argparse
import bisect
import json
import math
import statistics
import sys

RAD2DEG = 180.0 / math.pi
# 弹道参数的兜底值，和 rm_aim_sim 的 configs/default.cfg 一致。
# 正常情况下从 state.json 的 weapon 段读，只有老协议才用兜底。
FALLBACK_WEAPON = {"speed": 23.0, "gravity": 9.7833, "barrel_length": 0.22,
                   "delay": 0.015}


def load(path):
    """读采样文件，返回 (tick 列表, 事件列表)。"""
    ticks, events = [], []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            (events if row.get("k") == "ev" else ticks).append(row)
    if not ticks:
        raise SystemExit("采样文件里没有 tick: %s" % path)
    ticks.sort(key=lambda r: r["t"])
    return ticks, events


def forward(yaw, pitch):
    return (math.cos(pitch) * math.cos(yaw),
            math.cos(pitch) * math.sin(yaw),
            -math.sin(pitch))


def angle_between(a, b):
    dot = sum(x * y for x, y in zip(a, b))
    norm = math.sqrt(sum(x * x for x in a)) * math.sqrt(sum(x * x for x in b))
    if norm < 1e-12:
        return float("nan")
    return math.acos(max(-1.0, min(1.0, dot / norm)))


def solve_aim(delta, weapon, laps=8):
    """和仿真 `aim_assist._ballistic()` 同一个公式：解出打中的那个仰角。

    返回 (yaw, pitch, flight_time)，pitch 向下为正。够不着时返回 None。
    """
    speed = float(weapon.get("speed") or FALLBACK_WEAPON["speed"])
    gravity = float(weapon.get("gravity") or FALLBACK_WEAPON["gravity"])
    barrel = float(weapon.get("barrel_length") or FALLBACK_WEAPON["barrel_length"])
    horizontal = math.hypot(delta[0], delta[1])
    elevation = math.atan2(delta[2], horizontal)
    for _ in range(laps):
        distance = horizontal - barrel * math.cos(elevation)
        height = delta[2] - barrel * math.sin(elevation)
        if distance <= 0.0:
            return None
        if gravity < 1e-9:
            elevation = math.atan2(height, distance)
        else:
            discriminant = (speed ** 4 -
                            gravity * (gravity * distance * distance +
                                       2.0 * height * speed * speed))
            if discriminant < 0.0:
                return None
            elevation = math.atan((speed * speed - math.sqrt(discriminant)) /
                                  (gravity * distance))
    flight = (horizontal - barrel * math.cos(elevation)) / (speed * math.cos(elevation))
    return math.atan2(delta[1], delta[0]), -elevation, flight


def visible_ids(armors, origin):
    """外法向朝向云台的装甲。

    对向的两块板在图像上投影几乎重叠（4 m 处相差不到 1 度），不筛掉的话
    "角度最近的装甲"会随机落到背面那块上，指标就没意义了。
    """
    ids = [a["id"] for a in armors
           if sum(a["normal"][k] * (origin[k] - a["center"][k]) for k in range(3)) > 0.0]
    return ids if ids else [a["id"] for a in armors]


def nearest_armor(armors, origin, direction, pool=None):
    best, best_angle, best_distance = None, float("nan"), float("nan")
    for armor in armors:
        if pool is not None and armor["id"] not in pool:
            continue
        delta = [armor["center"][k] - origin[k] for k in range(3)]
        angle = angle_between(direction, delta)
        if best is None or angle < best_angle:
            best, best_angle = armor, angle
            best_distance = math.sqrt(sum(x * x for x in delta))
    return best, best_angle, best_distance


def armour_frame(armor):
    """由真值的 normal / normal_yaw 还原板面坐标系 (tangent, up)。

    simulation.gd: normal = radial*cos(tilt) + z*sin(tilt)，tangent = (-sin, cos, 0)，
    up = normal × tangent。
    """
    yaw = armor.get("normal_yaw")
    if yaw is None:
        return None, None
    tangent = (-math.sin(yaw), math.cos(yaw), 0.0)
    normal = armor["normal"]
    up = (normal[1] * tangent[2] - normal[2] * tangent[1],
          normal[2] * tangent[0] - normal[0] * tangent[2],
          normal[0] * tangent[1] - normal[1] * tangent[0])
    return tangent, up


def quantile(values, q):
    if not values:
        return None
    ordered = sorted(values)
    index = -1 if q >= 1.0 else max(0, int(len(ordered) * q) - 1)
    return round(ordered[index], 3)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("state", help="sample_state.py 的输出")
    parser.add_argument("--out", help="逐 tick / 逐发的明细写成 JSONL")
    args = parser.parse_args()

    ticks, events = load(args.state)
    times = [row["t"] for row in ticks]

    def tick_at(t):
        index = bisect.bisect_left(times, t)
        index = min(max(index, 0), len(times) - 1)
        if index > 0 and abs(times[index - 1] - t) < abs(times[index] - t):
            index -= 1
        return ticks[index]

    def armor_by_id(t, ident):
        for armor in tick_at(t).get("armors", []):
            if armor["id"] == ident:
                return armor
        return None

    # ---- 1. 逐 tick：云台指向 vs 理想弹道解 --------------------------------
    tracked = []
    switches = 0
    previous_id = None
    straight_errors = []
    distance_list = []
    for row in ticks:
        if row.get("paused") or not row.get("armors"):
            tracked.append({"t": row["t"], "valid": False})
            continue
        origin = row["gimbal"].get("origin")
        yaw, pitch = row["gimbal"].get("yaw"), row["gimbal"].get("pitch")
        if origin is None or yaw is None or pitch is None:
            tracked.append({"t": row["t"], "valid": False})
            continue
        direction = forward(yaw, pitch)
        pool = visible_ids(row["armors"], origin)
        armor, straight, distance = nearest_armor(row["armors"], origin, direction, pool)
        if armor is None:
            tracked.append({"t": row["t"], "valid": False})
            continue
        if previous_id is not None and armor["id"] != previous_id:
            switches += 1
        previous_id = armor["id"]
        straight_errors.append(straight * RAD2DEG)
        distance_list.append(distance)

        # 理想解：目标在"弹丸到达时刻"的位置。飞行时间又依赖仰角，迭代几次。
        weapon = row.get("weapon") or FALLBACK_WEAPON
        ideal = None
        flight = distance / float(weapon.get("speed") or FALLBACK_WEAPON["speed"])
        for _ in range(3):
            future = armor_by_id(row["t"] + flight, armor["id"]) or armor
            delta = [future["center"][k] - origin[k] for k in range(3)]
            solution = solve_aim(delta, weapon)
            if solution is None:
                break
            ideal, flight = solution, flight * 0.5 + solution[2] * 0.5

        entry = {"t": row["t"], "valid": True, "nearest_id": armor["id"],
                 "straight_deg": straight * RAD2DEG, "distance_m": distance,
                 "visible": row.get("visible"), "yaw": yaw, "pitch": pitch}
        if ideal is not None:
            entry["track_deg"] = angle_between(direction, forward(ideal[0], ideal[1])) * RAD2DEG
            entry["flight"] = flight
            entry["ideal_yaw"] = ideal[0]
            entry["ideal_pitch"] = ideal[1]
        tracked.append(entry)

    valid = [r for r in tracked if r["valid"]]
    span = max(times[-1] - times[0], 1e-6) if len(times) > 1 else 1e-6
    track = [r["track_deg"] for r in valid if "track_deg" in r]

    # ---- 2. 逐发弹丸：落点 -------------------------------------------------
    shots, impacts = {}, {}
    for event in events:
        if event.get("type") == "shot":
            shots[event["id"]] = event
        elif event.get("type") == "impact":
            impacts[event["id"]] = event

    shot_rows, offsets, kinds = [], [], {}
    for shot_id, shot in sorted(shots.items()):
        impact = impacts.get(shot_id)
        kind = impact.get("kind") if impact else "unresolved"
        kinds[kind] = kinds.get(kind, 0) + 1
        entry = {"k": "shot", "t": shot["time"], "id": shot_id, "kind": kind}
        if impact and impact.get("point"):
            target = None
            if impact.get("armor_id", -1) >= 0:
                target = armor_by_id(impact["time"], impact["armor_id"])
            if target is None:
                pool = tick_at(impact["time"]).get("armors") or []
                if pool:
                    target = min(pool, key=lambda a: math.dist(a["center"], impact["point"]))
            if target is None:
                shot_rows.append(entry)
                continue
            local = [impact["point"][k] - target["center"][k] for k in range(3)]
            tangent, up = armour_frame(target)
            if tangent is not None:
                u = sum(local[k] * tangent[k] for k in range(3))
                v = sum(local[k] * up[k] for k in range(3))
                entry["offset_cm"] = round(math.hypot(u, v) * 100.0, 2)
                if kind == "hit":
                    offsets.append(entry["offset_cm"])
            # 到板中心的直线残差，给"打飞了多远"一个直观量
            entry["miss_cm"] = round(math.sqrt(sum(x * x for x in local)) * 100.0, 2)
        shot_rows.append(entry)

    summary = {
        "ticks": len(ticks),
        "valid": len(valid),
        # 主指标：跟踪误差（相对理想弹道解，含伺服滞后）
        "track_error_mean_deg": round(statistics.fmean(track), 3) if track else None,
        "track_error_median_deg": round(statistics.median(track), 3) if track else None,
        "track_error_p95_deg": quantile(track, 0.95),
        # 参考值：相对"装甲中心直线"的偏差。正确的弹道解会在这里显示
        # 1.5~2.6°，所以它只适合和同一个场景的其它实现比，不适合跨场景比。
        "straight_error_mean_deg": round(statistics.fmean(straight_errors), 3)
        if straight_errors else None,
        "switch_per_s": round(switches / span, 2),
        "distance_m": round(statistics.fmean(distance_list), 2) if distance_list else None,
        # 逐发
        "shots": len(shots),
        "shots_per_s": round(len(shots) / span, 2),
        "offset_cm_mean": round(statistics.fmean(offsets), 2) if offsets else None,
        "offset_cm_p95": quantile(offsets, 0.95),
        "miss_kind": {k: v for k, v in sorted(kinds.items())},
        "span_s": round(span, 3),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))

    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            for row in tracked:
                handle.write(json.dumps(row) + "\n")
            for row in shot_rows:
                handle.write(json.dumps(row) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
