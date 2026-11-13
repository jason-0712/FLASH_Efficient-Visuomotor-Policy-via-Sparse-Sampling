#!/usr/bin/env bash
# ThinkPad host entry point. Copies code and runs subscriptions only.
set -euo pipefail
FLASHFR3_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FLASHFR3_RUN="flash_fr3_guard_shadow_$(date +%Y%m%d_%H%M%S_%N)"
FLASHFR3_CODE="/tmp/${FLASHFR3_RUN}_code"
FLASHFR3_OUTPUT="/tmp/${FLASHFR3_RUN}"
FLASHFR3_ARCHIVE="$FLASHFR3_WORKSPACE/artifacts/flash_fr3/guard_shadow_runs"
mkdir -p "$FLASHFR3_ARCHIVE"
docker exec franka mkdir -p "$FLASHFR3_CODE"
for FLASHFR3_FILE in capture_flash_fr3_observation.py run_flash_fr3_shadow_client.py \
    flash_fr3_shadow_math.py flash_fr3_reference.py flash_fr3_guard.py run_flash_fr3_guard_shadow.py; do
    docker cp "$FLASHFR3_WORKSPACE/scripts/$FLASHFR3_FILE" "franka:$FLASHFR3_CODE/"
done
FLASHFR3_EXIT=0
docker exec -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp franka bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ros/ros2_ws/install/setup.bash
    exec /usr/bin/python3 "$1" --output "$2" --seconds 8
' bash "$FLASHFR3_CODE/run_flash_fr3_guard_shadow.py" "$FLASHFR3_OUTPUT" || FLASHFR3_EXIT=$?
if docker exec franka test -d "$FLASHFR3_OUTPUT"; then
    docker cp "franka:$FLASHFR3_OUTPUT" "$FLASHFR3_ARCHIVE/"
    printf '只读诊断目录：%s/%s\n' "$FLASHFR3_ARCHIVE" "$FLASHFR3_RUN"
fi
exit "$FLASHFR3_EXIT"
