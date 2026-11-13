#!/usr/bin/env bash
# Check modes are read-only. Execute modes request real effort activation.
set -euo pipefail
FLASH_TORQUE_MODE="${1:---check}"
[[ $# -le 1 ]] || { printf '仅接受一个模式参数。\n' >&2; exit 2; }
case "$FLASH_TORQUE_MODE" in
  --check|--execute|--check-velocity|--execute-velocity|--check-settled-velocity|--execute-settled-velocity|--check-sdk-velocity|--execute-sdk-velocity) ;;
  *) printf '用法：bash %s [--check | --execute | --check-velocity | --execute-velocity | --check-settled-velocity | --execute-settled-velocity | --check-sdk-velocity | --execute-sdk-velocity]\n' "$0" >&2; exit 2 ;;
esac
FLASH_TORQUE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FLASH_TORQUE_CODE=/tmp/flash_fr3_torque_hold_code
FLASH_TORQUE_NAME="flash_fr3_torque_hold_${FLASH_TORQUE_MODE#--}_$(date +%Y%m%d_%H%M%S_%N)"
FLASH_TORQUE_ARCHIVE="$FLASH_TORQUE_ROOT/artifacts/flash_fr3/torque_hold_trials"
mkdir -p "$FLASH_TORQUE_ARCHIVE"
exec 9>"$FLASH_TORQUE_ARCHIVE/.client.lock"
flock -n 9 || { printf '已有力矩保持客户端运行。\n' >&2; exit 1; }
docker exec franka mkdir -p "$FLASH_TORQUE_CODE"
docker cp "$FLASH_TORQUE_ROOT/scripts/." "franka:$FLASH_TORQUE_CODE/" >/dev/null
FLASH_TORQUE_FLAGS=(); FLASH_TORQUE_DOCKER=()
if [[ "$FLASH_TORQUE_MODE" == --execute* ]]; then FLASH_TORQUE_FLAGS+=(--execute); FLASH_TORQUE_DOCKER=(-it); fi
if [[ "$FLASH_TORQUE_MODE" == *-velocity ]]; then FLASH_TORQUE_FLAGS+=(--velocity-probe); fi
if [[ "$FLASH_TORQUE_MODE" == *-settled-velocity ]]; then FLASH_TORQUE_FLAGS+=(--settle-before-probe); fi
if [[ "$FLASH_TORQUE_MODE" == *-sdk-velocity ]]; then FLASH_TORQUE_FLAGS+=(--settle-before-probe --sdk-torque-audit); fi
FLASH_TORQUE_EXIT=0
docker exec "${FLASH_TORQUE_DOCKER[@]}" -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e OPENBLAS_NUM_THREADS=1 franka bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ros/ros2_ws/install/setup.bash
    exec /usr/bin/python3 "$1/run_flash_fr3_torque_hold.py" --output "$2" "${@:3}"
  ' bash "$FLASH_TORQUE_CODE" "/tmp/$FLASH_TORQUE_NAME" "${FLASH_TORQUE_FLAGS[@]}" || FLASH_TORQUE_EXIT=$?
if docker exec franka test -d "/tmp/$FLASH_TORQUE_NAME"; then
  docker cp "franka:/tmp/$FLASH_TORQUE_NAME" "$FLASH_TORQUE_ARCHIVE/" >/dev/null
  printf '力矩保持记录：%s/%s\n' "$FLASH_TORQUE_ARCHIVE" "$FLASH_TORQUE_NAME"
fi
exit "$FLASH_TORQUE_EXIT"
