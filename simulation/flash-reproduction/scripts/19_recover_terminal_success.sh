#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
cd "$root/FLASH-Policy"
export PATH="$root/.venv/bin:$PATH" PYTHONPATH="$PWD" PYTHONUNBUFFERED=1
export CUDA_VISIBLE_DEVICES=GPU-f16a68c8-8e24-9ac3-0713-b9bdbc3af6f6
export MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=3 PYOPENGL_PLATFORM=egl
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
for idx in 7 25; do
  recovery="$root/data-recovery-source$idx"
  test ! -e "$recovery"
  timeout --signal=TERM --kill-after=30s 10m python -u scripts/advanced/collect_demo.py \
    --task libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate \
    --sim mujoco --robot franka --num_envs 1 --demo_start_idx "$idx" --num_demo_success 1 \
    --custom_save_dir "$recovery" --tot_steps_after_success 0 \
    --level 0 --no-run-all --run_unfinished --headless \
    2>&1 | tee "$root/logs/recover-source${idx}.log"
  demo=$(printf 'demo_%04d' "$idx")
  test -s "$recovery/success/$demo/metadata.json"
  test ! -e "$root/data-formal/success/$demo"
  cp -a "$recovery/success/$demo" "$root/data-formal/success/$demo"
done
cat > "$root/manifests/collection-deviations.json" <<'JSON'
{
  "default_tot_steps_after_success": 20,
  "recovered_reference_ids": [7, 25],
  "recovery_tot_steps_after_success": 0,
  "reason": "Both references hit the unchanged official success checker but exhausted their action sequence before the collector's post-success recording finished. Replayed separately with the public zero-extra-steps option; original failed outputs are preserved.",
  "smoke_reference_ids_reused": [1, 2],
  "training_demo_target": 40
}
JSON
python "$root/scripts/validate_demos.py" --root "$root/data-formal" --expected 40 --output "$root/manifests/formal-raw.json"
