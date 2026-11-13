#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
cat > "$root/scripts/wait_train_then_eval.sh" <<'JOB'
#!/usr/bin/env bash
set -euo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
while [ ! -s "$root/results/training-process-exit.json" ]; do
  if ! tmux has-session -t flash-bowl-s42-20261028 2>/dev/null; then
    echo 'Training session disappeared without an exit receipt; evaluation not started.' >&2
    exit 1
  fi
  sleep 15
done
bash "$root/scripts/25_verify_and_evaluate.sh"
JOB
if tmux has-session -t flash-bowl-eval-20261028 2>/dev/null; then
  echo 'Refusing duplicate evaluation session' >&2
  exit 1
fi
tmux new-session -d -s flash-bowl-eval-20261028 "bash '$root/scripts/wait_train_then_eval.sh' > '$root/logs/eval-formal-job.log' 2>&1"
tmux list-sessions
