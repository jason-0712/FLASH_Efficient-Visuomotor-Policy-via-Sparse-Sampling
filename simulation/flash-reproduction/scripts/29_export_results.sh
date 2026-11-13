#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
cd "$root/FLASH-Policy"
export PYTHONPATH="$PWD" OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
"$root/.venv/bin/python" "$root/scripts/export_results.py" --root "$root" 2>&1 | tee "$root/logs/export-results.log"
"$root/.venv/bin/python" "$root/scripts/build_report.py" --root "$root" 2>&1 | tee "$root/logs/build-report.log"
date -Is
tmux list-sessions 2>/dev/null || true
nvidia-smi --id=GPU-f16a68c8-8e24-9ac3-0713-b9bdbc3af6f6 --query-gpu=index,uuid,memory.used,utilization.gpu --format=csv > "$root/environment/gpu-after-evaluation.csv"
df -h "$root" > "$root/environment/disk-after-evaluation.txt"
