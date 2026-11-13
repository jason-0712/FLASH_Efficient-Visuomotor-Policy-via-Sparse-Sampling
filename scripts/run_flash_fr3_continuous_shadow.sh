#!/usr/bin/env bash
# Subscriber/read-only services and HTTP inference only; no execution mode.
set -euo pipefail
[[ $# -eq 0 ]] || { printf '此入口只读，不接受执行参数。\n' >&2; exit 2; }
FLASH_CONT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FLASH_CONT_NAME="flash_fr3_continuous_shadow_$(date +%Y%m%d_%H%M%S_%N)"
FLASH_CONT_ARCHIVE="$FLASH_CONT_ROOT/artifacts/flash_fr3/continuous_shadow_runs"
mkdir -p "$FLASH_CONT_ARCHIVE" "$FLASH_CONT_ROOT/artifacts/flash_fr3/torque_hold_trials"
exec 9>"$FLASH_CONT_ROOT/artifacts/flash_fr3/torque_hold_trials/.client.lock"
flock -n 9 || { printf '已有机械臂客户端运行。\n' >&2; exit 1; }
docker exec franka mkdir -p /tmp/flash_fr3_continuous_code
docker cp "$FLASH_CONT_ROOT/scripts/." franka:/tmp/flash_fr3_continuous_code/ >/dev/null
FLASH_CONT_EXIT=0
docker exec -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e OPENBLAS_NUM_THREADS=1 \
  franka bash -lc 'source /opt/ros/humble/setup.bash
    source /home/ros/ros2_ws/install/setup.bash
    exec python3 /tmp/flash_fr3_continuous_code/run_flash_fr3_continuous_shadow.py --output "$1"
  ' bash "/tmp/$FLASH_CONT_NAME" || FLASH_CONT_EXIT=$?
if docker exec franka test -d "/tmp/$FLASH_CONT_NAME"; then
  docker cp "franka:/tmp/$FLASH_CONT_NAME" "$FLASH_CONT_ARCHIVE/" >/dev/null
  printf '连续物理时间预测记录：%s/%s\n' "$FLASH_CONT_ARCHIVE" "$FLASH_CONT_NAME"
fi
exit "$FLASH_CONT_EXIT"
