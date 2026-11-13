#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
out="$root/comparison_20261028"
test ! -e "$out/results/fm-training-start.json"
tmux new-session -d -s flash-comparison-fm-20261028 "bash '$out/scripts/01_prepare_and_train_fm.sh' > '$out/logs/fm-job.log' 2>&1"
tmux new-session -d -s flash-comparison-early-20261028 "bash '$out/scripts/02_flash_early_evals.sh' > '$out/logs/flash-early-job.log' 2>&1"
tmux list-sessions
