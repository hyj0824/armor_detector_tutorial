#!/usr/bin/env python3
"""检查 rm_aim_sim 的接口契约有没有变。

课程依赖的是一条"看不见的协议"。上游一改，学生和你都会在开课当天才发现。
这个脚本把依赖项写成断言，上游一动它就红。

它自己启动仿真、自己发一次取帧请求、自己校验，不依赖 armor_aim。

用法:
    python3 bench/sim_smoke.py
    python3 bench/sim_smoke.py --keep-running    # 手动调试时留着不关
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CHANNEL = "smoke"
SHM = "/dev/shm/rm_aim_sim_%s" % CHANNEL
CONFIG = os.path.join(ROOT, "scenarios", "static.cfg")


def fail(message):
    print("FAIL: " + message)
    return 1


def read_json(name, timeout=5.0):
    path = os.path.join(SHM, name)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with open(path, "r", encoding="utf-8") as handle:
                return json.load(handle)
        except (OSError, ValueError):
            time.sleep(0.02)
    return None


def read_json_matching(name, predicate, timeout=10.0):
    """读到满足条件的内容为止。

    不能只读一次：上一次运行留下的 frame.json 还在共享内存里，
    直接读会拿到旧 request_id（这个坑已经在 smoke 里踩过一次）。
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = read_json(name, timeout=0.05)
        if value is not None and predicate(value):
            return value
        time.sleep(0.02)
    return None


def write_json(name, value):
    path = os.path.join(SHM, name)
    temporary = path + ".smoke.tmp"
    with open(temporary, "w", encoding="utf-8") as handle:
        json.dump(value, handle)
    os.replace(temporary, path)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sim-root", default=os.path.join(ROOT, "..", "rm_aim_sim"))
    parser.add_argument("--keep-running", action="store_true")
    args = parser.parse_args()

    sim_root = os.path.abspath(args.sim_root)
    if not os.access(os.path.join(sim_root, "run.sh"), os.X_OK):
        return fail("找不到 %s/run.sh" % sim_root)

    # 通道名只能改 config，不能走命令行，所以现场生成一份。
    config = os.path.join(ROOT, "scenarios", "_smoke.cfg")
    with open(CONFIG, "r", encoding="utf-8") as handle:
        text = handle.read()
    with open(config, "w", encoding="utf-8") as handle:
        handle.write(text + '\n[ipc]\nchannel="%s"\n' % CHANNEL)

    log = open(os.path.join(ROOT, "out", "sim-smoke.log"), "w", encoding="utf-8") \
        if os.path.isdir(os.path.join(ROOT, "out")) else subprocess.DEVNULL

    # 清掉上一次留下的信箱：state.json/frame.json 是“最新值邮箱”，
    # 不清就会读到上一次的 publisher_id，然后新仿真根本不认我们的请求。
    for name in ("state.json", "frame.json", "request.json"):
        try:
            os.remove(os.path.join(SHM, name))
        except OSError:
            pass

    process = subprocess.Popen([os.path.join(sim_root, "run.sh"), "--", "--remote",
                                "--config=" + config,
                                "--quit-after-seconds=25"],
                               cwd=sim_root, stdout=log, stderr=subprocess.STDOUT)
    code = 0
    try:
        state = read_json("state.json", timeout=40.0)
        if state is None:
            return fail("40 秒内没等到 state.json（仿真起来了吗？图形模式？）")

        # --- 状态契约 ---------------------------------------------------
        for key in ("protocol", "type", "session_id", "sim_time", "paused",
                    "gimbal", "camera", "truth", "stats"):
            if key not in state:
                return fail("state.json 缺字段: %s" % key)
        if state["protocol"] != "rm_aim_sim/2" or state["type"] != "state":
            return fail("协议版本变了: %s/%s" % (state["protocol"], state["type"]))
        for key in ("yaw", "pitch", "origin", "quaternion_wxyz"):
            if key not in state["gimbal"]:
                return fail("state.gimbal 缺字段: %s" % key)
        for key in ("K", "distortion", "R_camera2gimbal", "t_camera2gimbal",
                    "width", "height"):
            if key not in state["camera"]:
                return fail("state.camera 缺字段: %s" % key)
        if len(state["truth"].get("armors", [])) != 4:
            return fail("truth.armors 应该恒为 4 块")
        for key in ("fired", "resolved", "hit", "hit_rate"):
            if key not in state["stats"]:
                return fail("state.stats 缺字段: %s" % key)
            
        # --- 评分器依赖的契约（bench/sample_state.py + bench/truth.py）---
        # 这些字段被删了，评分会静默变成错数字，所以必须在这里卡住。
        for key in ("speed", "gravity", "barrel_length"):
            if key not in state.get("weapon", {}):
                return fail("state.weapon 缺字段: %s（弹道解算要用）" % key)
        for key in ("yaw_vel", "pitch_vel"):
            if key not in state["gimbal"]:
                return fail("state.gimbal 缺字段: %s（随机靶响应分析要用）" % key)
        if "armors" not in state["camera"]:
            return fail("state.camera 缺 armors（可见装甲数要用）")
        for armor in state["truth"].get("armors", []):
            for key in ("id", "center", "normal", "normal_yaw"):
                if key not in armor:
                    return fail("truth.armors[] 缺字段: %s（落点投影要用）" % key)
        events = state.get("events")
        if not isinstance(events, list):
            return fail("state 缺 events 列表（逐发弹丸评分要用）")
        for event in events:
            kind = event.get("type")
            if kind == "shot":
                for key in ("id", "time", "yaw", "pitch"):
                    if key not in event:
                        return fail("shot 事件缺字段: %s（出膛角要用）" % key)
            elif kind == "impact":
                for key in ("id", "time", "kind", "point"):
                    if key not in event:
                        return fail("impact 事件缺字段: %s（落点要用）" % key)

        # --- 取帧握手 ---------------------------------------------------
        request_id = "smoke-%d" % os.getpid()
        write_json("request.json", {"publisher_id": state["publisher_id"],
                                    "request_id": request_id})
        frame = read_json_matching(
            "frame.json", lambda f: f.get("request_id") == request_id, timeout=10.0)
        if frame is None:
            return fail("10 秒内没等到匹配的 frame.json（是图形模式吗？）")

        # --- 最重要的一条：图像元数据里不许有 truth ----------------------
        if "truth" in frame:
            return fail("frame.json 里出现了 truth：真值泄漏，必须修")

        for key in ("encoding", "image_bytes", "frame_file", "camera", "sim_time"):
            if key not in frame:
                return fail("frame.json 缺字段: %s" % key)
        camera = frame["camera"]
        channels = 3 if frame["encoding"] == "rgb8" else 4
        expect = camera["width"] * camera["height"] * channels
        if frame["image_bytes"] != expect:
            return fail("image_bytes 与宽高/编码不一致")
        if len(camera["K"]) != 9:
            return fail("camera.K 不是 3x3")

        path = os.path.join(SHM, frame["frame_file"])
        size = os.path.getsize(path) if os.path.exists(path) else -1
        if size != expect:
            return fail("原始帧文件大小不对: %d != %d" % (size, expect))

        print("PASS: 状态/取帧/图像契约一致")
        print("      %dx%d %s %.1f MB  sim_time=%.3f"
              % (camera["width"], camera["height"], frame["encoding"],
                 expect / 1e6, frame["sim_time"]))
    finally:
        if not args.keep_running:
            process.send_signal(signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
        try:
            os.remove(config)
        except OSError:
            pass

    return code


if __name__ == "__main__":
    sys.exit(main())
