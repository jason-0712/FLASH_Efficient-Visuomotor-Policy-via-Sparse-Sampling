#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
if tmux has-session -t flash-bowl-s42-20261028 2>/dev/null; then
  echo 'Refusing duplicate training session' >&2
  exit 1
fi
tmux new-session -d -s flash-bowl-s42-20261028 "bash '$root/scripts/22_train_formal.sh' > '$root/logs/train-formal-job.log' 2>&1"
tmux list-sessions
