#!/usr/bin/env bash
# Default is read-only. --execute moves only the arm; no gripper goals.
set -euo pipefail
FLASH_HISTORY_MODE="${1:---check}"
if [[ $# -gt 1 || ( "$FLASH_HISTORY_MODE" != --check && "$FLASH_HISTORY_MODE" != --execute ) ]]; then
  printf '用法：bash %s [--check | --execute]\n' "$0" >&2
  exit 2
fi
FLASH_HISTORY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FLASH_HISTORY_CODE=/tmp/flash_fr3_retimed_code
FLASH_HISTORY_NAME="flash_fr3_retimed_${FLASH_HISTORY_MODE#--}_$(date +%Y%m%d_%H%M%S_%N)"
FLASH_HISTORY_OUT="/tmp/$FLASH_HISTORY_NAME"
FLASH_HISTORY_ARCHIVE="$FLASH_HISTORY_ROOT/artifacts/flash_fr3/retimed_trials"
FLASH_HISTORY_ARGS=()
FLASH_HISTORY_DOCKER_ARGS=()
if [[ "$FLASH_HISTORY_MODE" == --execute ]]; then
  FLASH_HISTORY_ARGS=(--execute)
  FLASH_HISTORY_DOCKER_ARGS=(-it)
fi
test -f "$FLASH_HISTORY_ROOT/artifacts/flash_fr3/home_poses/task_start.json"
docker exec franka mkdir -p "$FLASH_HISTORY_CODE"
docker cp "$FLASH_HISTORY_ROOT/scripts/." "franka:$FLASH_HISTORY_CODE/" >/dev/null
docker cp "$FLASH_HISTORY_ROOT/artifacts/flash_fr3/task_setup/events.json" "franka:$FLASH_HISTORY_CODE/events.json" >/dev/null
docker cp "$FLASH_HISTORY_ROOT/artifacts/flash_fr3/home_poses/task_start.json" "franka:$FLASH_HISTORY_CODE/home.json" >/dev/null
FLASH_HISTORY_EXIT=0
docker exec "${FLASH_HISTORY_DOCKER_ARGS[@]}" -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e OPENBLAS_NUM_THREADS=1 -e OPENCV_FOR_THREADS_NUM=1 franka bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ros/ros2_ws/install/setup.bash
    exec /usr/bin/python3 "$1/run_flash_fr3_retimed_approach.py" --events "$1/events.json" \
      --home "$1/home.json" --output "$2" "${@:3}"
  ' bash "$FLASH_HISTORY_CODE" "$FLASH_HISTORY_OUT" "${FLASH_HISTORY_ARGS[@]}" || FLASH_HISTORY_EXIT=$?
if docker exec franka test -d "$FLASH_HISTORY_OUT"; then
  mkdir -p "$FLASH_HISTORY_ARCHIVE"
  docker cp "franka:$FLASH_HISTORY_OUT" "$FLASH_HISTORY_ARCHIVE/" >/dev/null
  printf '重定时接近检查：%s\n' "$FLASH_HISTORY_ARCHIVE/$FLASH_HISTORY_NAME"
fi
exit "$FLASH_HISTORY_EXIT"
