#!/usr/bin/env bash
# --save/check never move. --execute returns the arm, then opens the hand.
set -euo pipefail
FLASH_RETURN_MODE="${1:---check}"
if [[ $# -gt 0 ]]; then shift; fi
case "$FLASH_RETURN_MODE" in --save|--check|--execute) ;; *)
  printf '用法：bash %s [--save | --check | --execute] [--pose FILE.json]\n' "$0" >&2; exit 2;; esac
FLASH_RETURN_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FLASH_RETURN_POSE="$FLASH_RETURN_ROOT/artifacts/flash_fr3/home_poses/task_start.json"
if [[ $# -gt 0 ]]; then
  if [[ $# -ne 2 || "$1" != --pose ]]; then printf '姿态参数应为 --pose FILE.json\n' >&2; exit 2; fi
  FLASH_RETURN_POSE="$2"
fi
if [[ "$FLASH_RETURN_MODE" == --save ]]; then
  if [[ -e "$FLASH_RETURN_POSE" ]]; then printf '姿态文件已存在，拒绝覆盖：%s\n' "$FLASH_RETURN_POSE" >&2; exit 2; fi
else
  test -f "$FLASH_RETURN_POSE"
fi
FLASH_RETURN_CODE=/tmp/flash_fr3_return_code
FLASH_RETURN_NAME="flash_fr3_return_${FLASH_RETURN_MODE#--}_$(date +%Y%m%d_%H%M%S_%N)"
FLASH_RETURN_OUT="/tmp/$FLASH_RETURN_NAME"
FLASH_RETURN_ARCHIVE="$FLASH_RETURN_ROOT/artifacts/flash_fr3/return_trials"
mkdir -p "$FLASH_RETURN_ARCHIVE"
docker exec franka mkdir -p "$FLASH_RETURN_CODE"
docker cp "$FLASH_RETURN_ROOT/scripts/." "franka:$FLASH_RETURN_CODE/" >/dev/null
FLASH_RETURN_ARGS=(); FLASH_RETURN_DOCKER=()
if [[ "$FLASH_RETURN_MODE" == --save ]]; then
  FLASH_RETURN_INPUT="$FLASH_RETURN_OUT/saved_pose.json"
  FLASH_RETURN_ARGS=(--save)
else
  FLASH_RETURN_INPUT="/tmp/${FLASH_RETURN_NAME}_target.json"
  docker cp "$FLASH_RETURN_POSE" "franka:$FLASH_RETURN_INPUT" >/dev/null
fi
if [[ "$FLASH_RETURN_MODE" == --execute ]]; then FLASH_RETURN_ARGS=(--execute); FLASH_RETURN_DOCKER=(-it); fi
FLASH_RETURN_EXIT=0
docker exec "${FLASH_RETURN_DOCKER[@]}" -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e OPENBLAS_NUM_THREADS=1 franka bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ros/ros2_ws/install/setup.bash
    exec /usr/bin/python3 "$1/run_flash_fr3_return.py" --output "$2" --pose "$3" "${@:4}"
  ' bash "$FLASH_RETURN_CODE" "$FLASH_RETURN_OUT" "$FLASH_RETURN_INPUT" "${FLASH_RETURN_ARGS[@]}" || FLASH_RETURN_EXIT=$?
if docker exec franka test -d "$FLASH_RETURN_OUT"; then
  docker cp "franka:$FLASH_RETURN_OUT" "$FLASH_RETURN_ARCHIVE/" >/dev/null
  printf '回位记录：%s/%s\n' "$FLASH_RETURN_ARCHIVE" "$FLASH_RETURN_NAME"
fi
if [[ "$FLASH_RETURN_MODE" == --save && "$FLASH_RETURN_EXIT" -eq 0 ]]; then
  mkdir -p "$(dirname -- "$FLASH_RETURN_POSE")"
  # noclobber also protects against another save finishing at the same time.
  (set -o noclobber; cat "$FLASH_RETURN_ARCHIVE/$FLASH_RETURN_NAME/saved_pose.json" > "$FLASH_RETURN_POSE")
  printf '已保存当前位置：%s\n' "$FLASH_RETURN_POSE"
fi
exit "$FLASH_RETURN_EXIT"
