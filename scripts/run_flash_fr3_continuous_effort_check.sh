#!/usr/bin/env bash
# Offline only; no ROS, network clients, activation or execution mode.
set -euo pipefail
[[ $# -eq 2 && $1 == --run ]] || { printf '用法：bash %s --run 已归档的连续预测目录\n' "$0" >&2; exit 2; }
FLASH_EFFORT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FLASH_EFFORT_RUN="$(realpath -e -- "$2")"
[[ -f "$FLASH_EFFORT_RUN/packets.json" ]] || { printf '缺少 packets.json\n' >&2; exit 2; }
FLASH_EFFORT_NAME="flash_fr3_continuous_effort_check_$(date +%Y%m%d_%H%M%S_%N)"
FLASH_EFFORT_TEMP="/tmp/$FLASH_EFFORT_NAME"
FLASH_EFFORT_ARCHIVE="$FLASH_EFFORT_ROOT/artifacts/flash_fr3/continuous_effort_checks"
docker exec franka mkdir -p "$FLASH_EFFORT_TEMP/code" "$FLASH_EFFORT_TEMP/source"
for FLASH_EFFORT_FILE in flash_fr3_continuous_effort.py check_flash_fr3_continuous_effort.py; do
  docker cp "$FLASH_EFFORT_ROOT/scripts/$FLASH_EFFORT_FILE" "franka:$FLASH_EFFORT_TEMP/code/" >/dev/null
done
for FLASH_EFFORT_FILE in packets.json report.json prediction_000.json; do
  docker cp "$FLASH_EFFORT_RUN/$FLASH_EFFORT_FILE" "franka:$FLASH_EFFORT_TEMP/source/" >/dev/null
done
docker exec -e OPENBLAS_NUM_THREADS=1 franka python3 "$FLASH_EFFORT_TEMP/code/check_flash_fr3_continuous_effort.py" \
  --run "$FLASH_EFFORT_TEMP/source" --output "$FLASH_EFFORT_TEMP/result"
mkdir -p "$FLASH_EFFORT_ARCHIVE/$FLASH_EFFORT_NAME"
docker cp "franka:$FLASH_EFFORT_TEMP/result/." "$FLASH_EFFORT_ARCHIVE/$FLASH_EFFORT_NAME/" >/dev/null
printf '离线持续力矩准入记录：%s/%s\n' "$FLASH_EFFORT_ARCHIVE" "$FLASH_EFFORT_NAME"
