#!/usr/bin/env bash
# Read-only camera/state capture and remote inference. No robot commands.
set -euo pipefail
if [[ $# -gt 2 ]]; then
  printf '用法：bash %s [标签] [已有对照 report.json]\n' "$0" >&2
  exit 2
fi
FLASH_PROBE_LABEL="${1:-probe}"
if [[ ! "$FLASH_PROBE_LABEL" =~ ^[a-zA-Z0-9_-]{1,40}$ ]]; then
  printf '标签只接受 1 至 40 个字母、数字、下划线或连字符\n' >&2
  exit 2
fi
FLASH_PROBE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FLASH_PROBE_CODE=/tmp/flash_fr3_grasp_probe_code
FLASH_PROBE_NAME="flash_fr3_grasp_probe_${FLASH_PROBE_LABEL}_$(date +%Y%m%d_%H%M%S_%N)"
FLASH_PROBE_OUT="/tmp/$FLASH_PROBE_NAME"
FLASH_PROBE_ARCHIVE="$FLASH_PROBE_ROOT/artifacts/flash_fr3/grasp_probes"
FLASH_PROBE_ARGS=()
docker exec franka mkdir -p "$FLASH_PROBE_CODE"
docker cp "$FLASH_PROBE_ROOT/scripts/." "franka:$FLASH_PROBE_CODE/" >/dev/null
docker cp "$FLASH_PROBE_ROOT/artifacts/flash_fr3/task_setup/events.json" "franka:$FLASH_PROBE_CODE/events.json" >/dev/null
if [[ $# -eq 2 ]]; then
  test -f "$2"
  docker cp "$2" "franka:$FLASH_PROBE_CODE/reference.json" >/dev/null
  FLASH_PROBE_ARGS=(--reference "$FLASH_PROBE_CODE/reference.json")
fi
FLASH_PROBE_EXIT=0
docker exec -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e OPENBLAS_NUM_THREADS=1 -e OPENCV_FOR_THREADS_NUM=1 franka bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ros/ros2_ws/install/setup.bash
    exec /usr/bin/python3 "$1/run_flash_fr3_grasp_probe.py" --events "$1/events.json" \
      --output "$2" --label "$3" "${@:4}"
  ' bash "$FLASH_PROBE_CODE" "$FLASH_PROBE_OUT" "$FLASH_PROBE_LABEL" "${FLASH_PROBE_ARGS[@]}" || FLASH_PROBE_EXIT=$?
if docker exec franka test -d "$FLASH_PROBE_OUT"; then
  mkdir -p "$FLASH_PROBE_ARCHIVE"
  docker cp "franka:$FLASH_PROBE_OUT" "$FLASH_PROBE_ARCHIVE/" >/dev/null
  printf '只读抓取预测对照：%s\n' "$FLASH_PROBE_ARCHIVE/$FLASH_PROBE_NAME"
fi
exit "$FLASH_PROBE_EXIT"
