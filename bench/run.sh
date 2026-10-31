#!/usr/bin/env bash
# 跑一个场景：起仿真 -> 采真值 -> 跑自瞄 -> 收统计 -> 关仿真。
#
# 用法:
#   bench/run.sh static            # 默认 10 秒
#   bench/run.sh spin 15
#
# 环境变量:
#   SIM_ROOT   rm_aim_sim 的路径（默认 ../rm_aim_sim）
#   OUT_DIR    输出目录（默认 ./out）
#   LABEL      结果标签（默认等于场景名）
#   ARMOR_TRACE=1  额外让被测程序写 trace（可选，只有实现了 --trace 的程序才认）
#
# 评分数据全部来自仿真发布的 state.json，不依赖被测程序写任何日志。
#
# 注意：仿真必须图形模式启动（--headless 没有像素），所以需要有显示器或 Xvfb。
set -uo pipefail

SCENARIO=${1:-static}
DURATION=${2:-10}
DURATION_INT=${DURATION%%.*}
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SIM=${SIM_ROOT:-"$ROOT/../rm_aim_sim"}
OUT=${OUT_DIR:-"$ROOT/out"}
LABEL=${LABEL:-$SCENARIO}
CHANNEL=${CHANNEL:-armorbench}
CONFIG_SRC="$ROOT/scenarios/$SCENARIO.cfg"
CONFIG="$OUT/_bench-$LABEL.cfg"

BIN=${ARMOR_BIN:-"$ROOT/build/armor_aim"}
if [[ ! -x "$BIN" ]]; then
  echo "先编译: cmake -S . -B build && cmake --build build -j" >&2
  echo "（参考实现要加 -DBUILD_REFERENCE=ON，然后用 ARMOR_BIN=build/armor_aim_ref）" >&2
  exit 1
fi
if [[ ! -f "$CONFIG_SRC" ]]; then
  echo "没有这个场景: $CONFIG_SRC" >&2
  ls "$ROOT/scenarios" >&2
  exit 1
fi
if [[ ! -x "$SIM/run.sh" ]]; then
  echo "找不到仿真: $SIM/run.sh（用 SIM_ROOT=... 指定）" >&2
  exit 1
fi

mkdir -p "$OUT"
# 后面的 --config= 会传给仿真，而仿真以自己的目录为 cwd，所以必须是绝对路径。
OUT=$(cd "$OUT" && pwd)
CHANNEL=${CHANNEL:-armorbench}
CONFIG="$OUT/_bench-$LABEL.cfg"

EXTRA=()
if [[ "${ARMOR_TRACE:-0}" == "1" ]]; then
  EXTRA+=("--trace=$OUT/trace-$LABEL.jsonl")
fi

# 用**独立通道**，不要和别人手起的仿真抢 default：
# 同一通道只允许一个客户端，抢到了对方就会启动失败。
# 顺手清掉上一次被强杀留下的锁。
rm -rf "/dev/shm/rm_aim_sim_$CHANNEL"
{ cat "$CONFIG_SRC"; printf '\n[ipc]\nchannel="%s"\n' "$CHANNEL"; } > "$CONFIG"

# 评分器是可选的：教师仓库里有这两个文件，学生仓库里没有。
# 缺了就只出命中率（[sim] 那行），不报错。
SAMPLER="$ROOT/bench/sample_state.py"
ALIGN="$ROOT/bench/truth.py"
[[ -f "$SAMPLER" && -f "$ALIGN" ]] || SAMPLER=""

SIM_PID=""
TRUTH_PID=""
cleanup() {
  [[ -n "$TRUTH_PID" ]] && kill "$TRUTH_PID" 2>/dev/null
  [[ -n "$SIM_PID" ]] && kill "$SIM_PID" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT

echo "[bench] scenario=$SCENARIO duration=${DURATION}s"
"$SIM/run.sh" -- --remote --config="$CONFIG" \
    --quit-after-seconds=$((DURATION_INT + 12)) > "$OUT/sim-$LABEL.log" 2>&1 &
SIM_PID=$!

# 等仿真把通道建起来（最多 30 秒）。
for _ in $(seq 1 60); do
  if grep -q "Aim Lab ready" "$OUT/sim-$LABEL.log" 2>/dev/null; then
    break
  fi
  if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "仿真启动失败，看 $OUT/sim-$LABEL.log" >&2
    tail -20 "$OUT/sim-$LABEL.log" >&2
    exit 1
  fi
  sleep 0.5
done
sleep 1

if [[ -n "$SAMPLER" ]]; then
  python3 "$SAMPLER" --channel="$CHANNEL" --out "$OUT/state-$LABEL.jsonl" &
  TRUTH_PID=$!
  sleep 0.5
fi

"$BIN" --channel="$CHANNEL" --color=blue --fire --no-gui \
    --duration="$DURATION" "${EXTRA[@]}" \
    | tee "$OUT/client-$LABEL.log"

kill "$TRUTH_PID" 2>/dev/null
TRUTH_PID=""

# 健康检查：客户端一帧都没拿到，多半是通道/启动竞争，这一轮的数字没有意义。
if ! grep -q '^\[self\]' "$OUT/client-$LABEL.log" 2>/dev/null; then
  echo "[bench][warn] 客户端没有输出 [self] 行：这一轮无效（通道被占/启动竞争），请重跑" >&2
fi

# 记一份运行元数据：report.py 要用它算"每秒命中"，也用它区分基线/参考实现。
python3 - "$OUT/meta-$LABEL.json" "$SCENARIO" "$DURATION" "$BIN" <<'PYEOF'
import json, sys
json.dump({"scenario": sys.argv[2], "duration": float(sys.argv[3]),
           "binary": sys.argv[4]}, open(sys.argv[1], "w"))
PYEOF

if [[ -n "$SAMPLER" ]]; then
  echo "[bench] 对齐真值:"
  python3 "$ALIGN" "$OUT/state-$LABEL.jsonl" \
      --out "$OUT/aim-$LABEL.jsonl" | tee "$OUT/aim-$LABEL.json"
fi

exit 0
