#!/usr/bin/env bash
# --prepare / --check / --shadow do not activate or send heartbeat messages.
set -euo pipefail
FLASHFR3_MODE="${1:---check}"
if [[ $# -gt 0 ]]; then shift; fi
case "$FLASHFR3_MODE" in --prepare|--check|--execute|--shadow) ;; *)
  printf '用法：bash %s [--prepare | --check | --execute | --shadow]\n' "$0" >&2;exit 2;; esac
if [[ $# -ne 0 ]]; then printf '不接受额外参数\n' >&2;exit 2;fi
FLASHFR3_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FLASHFR3_CODE=/tmp/flash_fr3_stream_code
FLASHFR3_ARCHIVE="$FLASHFR3_ROOT/artifacts/flash_fr3/stream_trials"
FLASHFR3_LATEST="$FLASHFR3_ROOT/artifacts/flash_fr3/stream_latest.txt"
FLASHFR3_RUN="flash_fr3_stream_${FLASHFR3_MODE#--}_$(date +%Y%m%d_%H%M%S_%N)"
FLASHFR3_OUT="/tmp/$FLASHFR3_RUN"
mkdir -p "$FLASHFR3_ARCHIVE"
docker exec franka mkdir -p "$FLASHFR3_CODE"
for FLASHFR3_FILE in run_flash_fr3_stream.py prepare_flash_fr3_stream.py configure_flash_fr3_stream.py \
    flash_fr3_stream_plan.py flash_fr3_stream_io.py flash_fr3_sensor_process.py check_flash_fr3_rolling.py \
    run_flash_fr3_model_prefix.py prepare_flash_fr3_model_prefix.py flash_fr3_model_plan.py \
    run_flash_fr3_pilot.py capture_flash_fr3_observation.py flash_fr3_guard.py flash_fr3_reference.py \
    flash_fr3_shadow_math.py run_flash_fr3_shadow_client.py run_flash_fr3_guard_shadow.py;do
  docker cp "$FLASHFR3_ROOT/scripts/$FLASHFR3_FILE" "franka:$FLASHFR3_CODE/" >/dev/null
done
FLASHFR3_EXIT=0
if [[ "$FLASHFR3_MODE" == --prepare ]];then
  docker exec -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e OPENBLAS_NUM_THREADS=1 \
      -e OPENCV_FOR_THREADS_NUM=1 franka bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ros/ros2_ws/install/setup.bash
    /usr/bin/python3 "$1/prepare_flash_fr3_stream.py" --output "$2" &&
    /usr/bin/python3 "$1/configure_flash_fr3_stream.py" --plan "$2/plan.json" &&
    /usr/bin/python3 "$1/run_flash_fr3_stream.py" --plan "$2/plan.json" --output "$2/readonly_check"
  ' bash "$FLASHFR3_CODE" "$FLASHFR3_OUT" || FLASHFR3_EXIT=$?
elif [[ "$FLASHFR3_MODE" == --shadow ]];then
  docker exec -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e OPENBLAS_NUM_THREADS=1 \
      -e OPENCV_FOR_THREADS_NUM=1 franka bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ros/ros2_ws/install/setup.bash
    /usr/bin/python3 "$1/check_flash_fr3_rolling.py" --output "$2" --replacements 2
  ' bash "$FLASHFR3_CODE" "$FLASHFR3_OUT" || FLASHFR3_EXIT=$?
else
  if [[ ! -f "$FLASHFR3_LATEST" ]];then printf '请先运行 --prepare\n' >&2;exit 2;fi
  IFS= read -r FLASHFR3_PLAN < "$FLASHFR3_LATEST"
  test -f "$FLASHFR3_PLAN"
  docker cp "$FLASHFR3_PLAN" "franka:$FLASHFR3_CODE/plan.json" >/dev/null
  FLASHFR3_DOCKER_ARGS=();FLASHFR3_PYTHON_ARGS=()
  if [[ "$FLASHFR3_MODE" == --execute ]];then FLASHFR3_DOCKER_ARGS=(-it);FLASHFR3_PYTHON_ARGS=(--execute);fi
  docker exec "${FLASHFR3_DOCKER_ARGS[@]}" -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
      -e OPENBLAS_NUM_THREADS=1 -e OPENCV_FOR_THREADS_NUM=1 franka bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ros/ros2_ws/install/setup.bash
    exec /usr/bin/python3 "$1/run_flash_fr3_stream.py" --plan "$1/plan.json" --output "$2" "${@:3}"
  ' bash "$FLASHFR3_CODE" "$FLASHFR3_OUT" "${FLASHFR3_PYTHON_ARGS[@]}" || FLASHFR3_EXIT=$?
fi
if docker exec franka test -d "$FLASHFR3_OUT";then
  docker cp "franka:$FLASHFR3_OUT" "$FLASHFR3_ARCHIVE/" >/dev/null
  if [[ "$FLASHFR3_MODE" == --prepare && "$FLASHFR3_EXIT" -eq 0 ]];then
    printf '%s\n' "$FLASHFR3_ARCHIVE/$FLASHFR3_RUN/plan.json" > "$FLASHFR3_LATEST"
  fi
  printf '连续预测试验记录：%s\n' "$FLASHFR3_ARCHIVE/$FLASHFR3_RUN"
fi
exit "$FLASHFR3_EXIT"
