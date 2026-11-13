#!/usr/bin/env bash
set -euo pipefail
FLASH_RAW_MODE="${1:---prepare}"
if [[ $# -gt 0 ]]; then shift; fi
[[ "$FLASH_RAW_MODE" == --prepare || "$FLASH_RAW_MODE" == --execute ]] || {
  printf '用法：bash %s [--prepare | --execute] [--seconds N] [--arm-only]\n' "$0" >&2; exit 2;
}
FLASH_RAW_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FLASH_RAW_ARCHIVE="$FLASH_RAW_ROOT/artifacts/flash_fr3/raw_policy_trials"
FLASH_RAW_NAME="flash_fr3_raw_policy_${FLASH_RAW_MODE#--}_$(date +%Y%m%d_%H%M%S_%N)"
mkdir -p "$FLASH_RAW_ARCHIVE" "$FLASH_RAW_ROOT/artifacts/flash_fr3/torque_hold_trials"
exec 9>"$FLASH_RAW_ROOT/artifacts/flash_fr3/torque_hold_trials/.client.lock"
flock -n 9 || { printf '已有机械臂客户端运行。\n' >&2; exit 1; }
docker exec franka mkdir -p /tmp/flash_fr3_raw_policy_code
docker cp "$FLASH_RAW_ROOT/scripts/." franka:/tmp/flash_fr3_raw_policy_code/ >/dev/null
if [[ -f "$FLASH_RAW_ROOT/artifacts/flash_fr3/task_setup/events.json" ]]; then
  docker cp "$FLASH_RAW_ROOT/artifacts/flash_fr3/task_setup/events.json" franka:/tmp/flash_fr3_raw_policy_code/events.json >/dev/null
fi
FLASH_RAW_ARGS=("$@"); FLASH_RAW_DOCKER=()
if [[ "$FLASH_RAW_MODE" == --execute ]]; then
  FLASH_RAW_ARGS+=(--execute); FLASH_RAW_DOCKER=(-it)
  docker exec franka test -f /home/ros/ros2_ws/install/flash_fr3_model_controller/lib/libflash_fr3_continuous_effort_plugin.so || {
    printf '持续力矩插件尚未安装。\n' >&2; exit 1;
  }
fi
FLASH_RAW_EXIT=0
docker exec "${FLASH_RAW_DOCKER[@]}" -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e OPENBLAS_NUM_THREADS=1 franka bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ros/ros2_ws/install/setup.bash
    exec python3 /tmp/flash_fr3_raw_policy_code/run_flash_fr3_raw_policy.py --output "$1" "${@:2}"
  ' bash "/tmp/$FLASH_RAW_NAME" "${FLASH_RAW_ARGS[@]}" || FLASH_RAW_EXIT=$?
if docker exec franka test -d "/tmp/$FLASH_RAW_NAME"; then
  docker cp "franka:/tmp/$FLASH_RAW_NAME" "$FLASH_RAW_ARCHIVE/" >/dev/null
  printf '原始 FLASH 连续执行记录：%s/%s\n' "$FLASH_RAW_ARCHIVE" "$FLASH_RAW_NAME"
fi
exit "$FLASH_RAW_EXIT"
