#!/usr/bin/env bash
# Check modes are read-only; execution modes send one gripper goal only.
set -euo pipefail
FLASH_GRASP_MODE="${1:---check}"
if [[ $# -gt 1 || ( "$FLASH_GRASP_MODE" != --check && "$FLASH_GRASP_MODE" != --execute &&
                   "$FLASH_GRASP_MODE" != --open-check && "$FLASH_GRASP_MODE" != --open ) ]]; then
  printf '用法：bash %s [--check | --execute | --open-check | --open]\n' "$0" >&2
  exit 2
fi
FLASH_GRASP_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FLASH_GRASP_PLAN="$FLASH_GRASP_ROOT/artifacts/flash_fr3/gripper_plans/stationary_grasp_20261030_01/plan.json"
FLASH_GRASP_CODE=/tmp/flash_fr3_stationary_grasp_code
FLASH_GRASP_NAME="flash_fr3_stationary_grasp_${FLASH_GRASP_MODE#--}_$(date +%Y%m%d_%H%M%S_%N)"
FLASH_GRASP_OUT="/tmp/$FLASH_GRASP_NAME"
FLASH_GRASP_ARCHIVE="$FLASH_GRASP_ROOT/artifacts/flash_fr3/stationary_grasp_trials"
FLASH_GRASP_ARGS=();FLASH_GRASP_DOCKER=()
if [[ "$FLASH_GRASP_MODE" == --execute ]]; then FLASH_GRASP_ARGS=(--execute);FLASH_GRASP_DOCKER=(-it);fi
if [[ "$FLASH_GRASP_MODE" == --open-check ]]; then FLASH_GRASP_ARGS=(--open);fi
if [[ "$FLASH_GRASP_MODE" == --open ]]; then FLASH_GRASP_ARGS=(--open --execute);FLASH_GRASP_DOCKER=(-it);fi
test -f "$FLASH_GRASP_PLAN"
docker exec franka mkdir -p "$FLASH_GRASP_CODE"
docker cp "$FLASH_GRASP_ROOT/scripts/." "franka:$FLASH_GRASP_CODE/" >/dev/null
docker cp "$FLASH_GRASP_PLAN" "franka:$FLASH_GRASP_CODE/plan.json" >/dev/null
FLASH_GRASP_EXIT=0
docker exec "${FLASH_GRASP_DOCKER[@]}" -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e OPENBLAS_NUM_THREADS=1 franka bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ros/ros2_ws/install/setup.bash
    exec /usr/bin/python3 "$1/run_flash_fr3_stationary_grasp.py" --plan "$1/plan.json" \
      --output "$2" "${@:3}"
  ' bash "$FLASH_GRASP_CODE" "$FLASH_GRASP_OUT" "${FLASH_GRASP_ARGS[@]}" || FLASH_GRASP_EXIT=$?
if docker exec franka test -d "$FLASH_GRASP_OUT"; then
  mkdir -p "$FLASH_GRASP_ARCHIVE"
  docker cp "franka:$FLASH_GRASP_OUT" "$FLASH_GRASP_ARCHIVE/" >/dev/null
  printf '静止闭爪记录：%s/%s\n' "$FLASH_GRASP_ARCHIVE" "$FLASH_GRASP_NAME"
fi
exit "$FLASH_GRASP_EXIT"
