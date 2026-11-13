#!/usr/bin/env bash
# ThinkPad host. --prepare and --check never activate the controller.
set -euo pipefail
FLASHFR3_MODE="${1:---check}"
if [[ $# -gt 0 ]]; then shift; fi
case "$FLASHFR3_MODE" in --prepare|--check|--execute|--execute-watchdog) ;; *)
  printf '用法：bash %s [--prepare | --check | --execute | --execute-watchdog] [--plan PLAN_JSON]\n' "$0" >&2
  exit 2;; esac
FLASHFR3_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FLASHFR3_CODE=/tmp/flash_fr3_model_code
FLASHFR3_ARCHIVE="$FLASHFR3_ROOT/artifacts/flash_fr3/model_prefix_trials"
FLASHFR3_LATEST="$FLASHFR3_ROOT/artifacts/flash_fr3/model_prefix_latest.txt"
FLASHFR3_RUN="flash_fr3_model_${FLASHFR3_MODE#--}_$(date +%Y%m%d_%H%M%S_%N)"
FLASHFR3_OUT="/tmp/$FLASHFR3_RUN"
FLASHFR3_PLAN=''
if [[ $# -gt 0 ]]; then
  if [[ $# -ne 2 || "$1" != --plan || "$FLASHFR3_MODE" == --prepare ]]; then
    printf '只有检查或执行模式可附加 --plan PLAN_JSON\n' >&2; exit 2
  fi
  FLASHFR3_PLAN="$2"
fi
mkdir -p "$FLASHFR3_ARCHIVE"
docker exec franka mkdir -p "$FLASHFR3_CODE"
for FLASHFR3_FILE in run_flash_fr3_model_prefix.py prepare_flash_fr3_model_prefix.py \
    configure_flash_fr3_model_prefix.py flash_fr3_model_plan.py run_flash_fr3_pilot.py \
    capture_flash_fr3_observation.py flash_fr3_guard.py flash_fr3_reference.py \
    flash_fr3_shadow_math.py run_flash_fr3_shadow_client.py run_flash_fr3_guard_shadow.py; do
  docker cp "$FLASHFR3_ROOT/scripts/$FLASHFR3_FILE" "franka:$FLASHFR3_CODE/"
done
FLASHFR3_EXIT=0
if [[ "$FLASHFR3_MODE" == --prepare ]]; then
  docker exec -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp franka bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ros/ros2_ws/install/setup.bash
    /usr/bin/python3 "$1/prepare_flash_fr3_model_prefix.py" --output "$2" &&
    /usr/bin/python3 "$1/configure_flash_fr3_model_prefix.py" --plan "$2/plan.json" &&
    /usr/bin/python3 "$1/run_flash_fr3_model_prefix.py" --plan "$2/plan.json" --output "$2/readonly_check"
  ' bash "$FLASHFR3_CODE" "$FLASHFR3_OUT" || FLASHFR3_EXIT=$?
else
  if [[ -z "$FLASHFR3_PLAN" ]]; then
    if [[ ! -f "$FLASHFR3_LATEST" ]]; then printf '请先运行 --prepare\n' >&2; exit 2; fi
    IFS= read -r FLASHFR3_PLAN < "$FLASHFR3_LATEST"
  fi
  test -f "$FLASHFR3_PLAN"
  docker cp "$FLASHFR3_PLAN" "franka:$FLASHFR3_CODE/plan.json"
  FLASHFR3_DOCKER_ARGS=()
  FLASHFR3_PYTHON_ARGS=()
  if [[ "$FLASHFR3_MODE" != --check ]]; then
    FLASHFR3_DOCKER_ARGS=(-it)
    FLASHFR3_PYTHON_ARGS=(--execute)
    if [[ "$FLASHFR3_MODE" == --execute-watchdog ]]; then FLASHFR3_PYTHON_ARGS+=(--watchdog-test); fi
  fi
  docker exec "${FLASHFR3_DOCKER_ARGS[@]}" -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
      franka bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ros/ros2_ws/install/setup.bash
    exec /usr/bin/python3 "$1/run_flash_fr3_model_prefix.py" --plan "$1/plan.json" --output "$2" "${@:3}"
  ' bash "$FLASHFR3_CODE" "$FLASHFR3_OUT" "${FLASHFR3_PYTHON_ARGS[@]}" || FLASHFR3_EXIT=$?
fi
if docker exec franka test -d "$FLASHFR3_OUT"; then
  docker cp "franka:$FLASHFR3_OUT" "$FLASHFR3_ARCHIVE/"
  printf '模型短段记录：%s/%s\n' "$FLASHFR3_ARCHIVE" "$FLASHFR3_RUN"
fi
if [[ "$FLASHFR3_MODE" == --prepare && "$FLASHFR3_EXIT" -eq 0 ]]; then
  printf '%s\n' "$FLASHFR3_ARCHIVE/$FLASHFR3_RUN/plan.json" > "$FLASHFR3_LATEST"
fi
exit "$FLASHFR3_EXIT"
