#!/usr/bin/env bash
# Run on the ThinkPad host. Default --check performs no activation/heartbeat.
set -euo pipefail
FLASHFR3_MODE="${1:---check}"
if [[ $# -gt 0 ]]; then shift; fi
if [[ "$FLASHFR3_MODE" != --check && "$FLASHFR3_MODE" != --execute && "$FLASHFR3_MODE" != --execute-watchdog ]]; then
    printf '用法：bash %s [--check | --execute | --execute-watchdog] [--plan PLAN_JSON]\n' "$0" >&2
    exit 2
fi
FLASHFR3_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FLASHFR3_CODE=/tmp/flash_fr3_pilot_client_code
FLASHFR3_PLAN="$FLASHFR3_ROOT/artifacts/flash_fr3/pilot_plans/flash_fr3_pilot_plan_20261023_01/plan.json"
if [[ $# -gt 0 ]]; then
    if [[ $# -ne 2 || "$1" != --plan ]]; then
        printf '计划参数应为 --plan PLAN_JSON\n' >&2
        exit 2
    fi
    FLASHFR3_PLAN="$2"
fi
FLASHFR3_RUN="flash_fr3_pilot_${FLASHFR3_MODE#--}_$(date +%Y%m%d_%H%M%S_%N)"
FLASHFR3_OUT="/tmp/$FLASHFR3_RUN"
FLASHFR3_ARCHIVE="$FLASHFR3_ROOT/artifacts/flash_fr3/pilot_trials"
test -f "$FLASHFR3_PLAN"
mkdir -p "$FLASHFR3_ARCHIVE"
docker exec franka mkdir -p "$FLASHFR3_CODE"
for FLASHFR3_FILE in run_flash_fr3_pilot.py capture_flash_fr3_observation.py flash_fr3_guard.py \
    flash_fr3_reference.py flash_fr3_shadow_math.py; do
    docker cp "$FLASHFR3_ROOT/scripts/$FLASHFR3_FILE" "franka:$FLASHFR3_CODE/"
done
docker cp "$FLASHFR3_PLAN" "franka:$FLASHFR3_CODE/plan.json"
FLASHFR3_DOCKER_ARGS=()
FLASHFR3_PYTHON_ARGS=()
if [[ "$FLASHFR3_MODE" != --check ]]; then
    # Interactive PTY forwards Ctrl-C to the Python client inside the container.
    FLASHFR3_DOCKER_ARGS=(-it)
    FLASHFR3_PYTHON_ARGS=(--execute)
    if [[ "$FLASHFR3_MODE" == --execute-watchdog ]]; then
        FLASHFR3_PYTHON_ARGS+=(--watchdog-test)
    fi
fi
FLASHFR3_EXIT=0
docker exec "${FLASHFR3_DOCKER_ARGS[@]}" -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    franka bash -lc '
      source /opt/ros/humble/setup.bash
      source /home/ros/ros2_ws/install/setup.bash
      exec /usr/bin/python3 "$1/run_flash_fr3_pilot.py" --plan "$1/plan.json" --output "$2" "${@:3}"
    ' bash "$FLASHFR3_CODE" "$FLASHFR3_OUT" "${FLASHFR3_PYTHON_ARGS[@]}" || FLASHFR3_EXIT=$?
if docker exec franka test -d "$FLASHFR3_OUT"; then
    docker cp "franka:$FLASHFR3_OUT" "$FLASHFR3_ARCHIVE/"
    printf '试运行记录：%s/%s\n' "$FLASHFR3_ARCHIVE" "$FLASHFR3_RUN"
fi
exit "$FLASHFR3_EXIT"
