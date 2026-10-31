#!/usr/bin/env bash
# 跑完整套场景，结果落到 $OUT_DIR（默认 out/<标签>），最后汇总成一张表。
#
# 用法:
#   bench/run_all.sh                       # 全部 7 个场景，每个 8 秒，标签 full
#   TAG=before bench/run_all.sh            # 换标签，方便 A/B
#   DUR=8 TAG=after bench/run_all.sh
#   ARMOR_BIN=build/armor_aim_ref TAG=ref bench/run_all.sh
#
# 之后对比：
#   python3 bench/report.py --out-dir out/before
#   python3 bench/report.py --out-dir out/after
set -uo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
DUR=${DUR:-8}
TAG=${TAG:-full}
export OUT_DIR=${OUT_DIR:-"$ROOT/out/$TAG"}
SCENARIOS=${SCENARIOS:-"static lowlight far translate spin combined toofar"}
mkdir -p "$OUT_DIR"

for s in $SCENARIOS; do
  echo "===== $s ====="
  LABEL=$s "$ROOT/bench/run.sh" "$s" "$DUR" 2>&1 \
    | grep -E '^\[sim\]|^\[self\]|track_error_median|offset_cm_mean|"shots"'
done

echo "===== report ====="
python3 "$ROOT/bench/report.py" --out-dir "$OUT_DIR" --save "$OUT_DIR/report.md"
