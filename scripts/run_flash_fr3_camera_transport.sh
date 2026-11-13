#!/usr/bin/env bash
# Passive subscriptions only. This script never activates arm or gripper IO.
set -euo pipefail
FLASH_CAMERA_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FLASH_CAMERA_NAME="flash_fr3_camera_transport_$(date +%Y%m%d_%H%M%S_%N)"
FLASH_CAMERA_OUT="/tmp/$FLASH_CAMERA_NAME"
FLASH_CAMERA_CODE=/tmp/flash_fr3_camera_transport_code
FLASH_CAMERA_ARCHIVE="$FLASH_CAMERA_ROOT/artifacts/flash_fr3/camera_transport"
mkdir -p "$FLASH_CAMERA_ARCHIVE"
docker exec franka mkdir -p "$FLASH_CAMERA_CODE"
docker cp "$FLASH_CAMERA_ROOT/scripts/check_flash_fr3_camera_transport.py" "franka:$FLASH_CAMERA_CODE/" >/dev/null
docker cp "$FLASH_CAMERA_ROOT/scripts/flash_fr3_transport_diagnostics.py" "franka:$FLASH_CAMERA_CODE/" >/dev/null
FLASH_CAMERA_TTY=()
if [[ -t 0 && -t 1 ]]; then FLASH_CAMERA_TTY=(-it); fi
FLASH_CAMERA_EXIT=0
docker exec "${FLASH_CAMERA_TTY[@]}" -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  franka bash -lc '
    source /opt/ros/humble/setup.bash
    exec /usr/bin/python3 "$1/check_flash_fr3_camera_transport.py" --output "$2" "${@:3}"
  ' bash "$FLASH_CAMERA_CODE" "$FLASH_CAMERA_OUT" "$@" || FLASH_CAMERA_EXIT=$?
if docker exec franka test -d "$FLASH_CAMERA_OUT"; then
  docker cp "franka:$FLASH_CAMERA_OUT" "$FLASH_CAMERA_ARCHIVE/" >/dev/null
  printf '相机传输诊断：%s/%s\n' "$FLASH_CAMERA_ARCHIVE" "$FLASH_CAMERA_NAME"
fi
exit "$FLASH_CAMERA_EXIT"
