#!/usr/bin/env bash
set -u -o pipefail
date -u '+%Y-%m-%dT%H:%M:%SZ'
echo '$ ssh -o BatchMode=yes -o ConnectTimeout=12 -o StrictHostKeyChecking=yes cps-server bash -s'
ssh -o BatchMode=yes -o ConnectTimeout=12 -o StrictHostKeyChecking=yes -o UserKnownHostsFile="$(cd "$(dirname "$0")/../environment" && pwd)/ssh_known_hosts" cps-server bash -s <<'REMOTE'
set -u
run() { printf '\n$'; printf ' %q' "$@"; printf '\n'; "$@"; printf 'exit=%s\n' "$?"; }
run date -Is
run id
run hostname
run uname -a
run cat /etc/os-release
run nvidia-smi --query-gpu=index,name,uuid,memory.total,memory.free,utilization.gpu,driver_version --format=csv
run nvidia-smi --query-compute-apps=gpu_uuid,pid,process_name,used_memory --format=csv
run free -h
run df -h
run lscpu
run pwd
run ls -la
for cmd in tmux screen squeue sbatch conda mamba uv python python3 git; do command -v "$cmd" || true; done
for d in /data /workspace /work /scratch /mnt /opt; do if [ -d "$d" ]; then run ls -ld "$d"; fi; done
REMOTE
rc=$?
printf '\nssh_exit=%s\n' "$rc"
exit "$rc"
