#!/usr/bin/env bash
set -u -o pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
script=$1
log=$2
printf 'start_utc=%s\nscript=%s\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$script" | tee "$root/logs/$log"
shasum -a 256 "$script" | tee -a "$root/logs/$log"
ssh -o BatchMode=yes -o ConnectTimeout=12 -o ServerAliveInterval=30 -o ServerAliveCountMax=3 -o StrictHostKeyChecking=yes -o UserKnownHostsFile="$root/environment/ssh_known_hosts" cps-server bash -s < "$script" 2>&1 | tee -a "$root/logs/$log"
rc=${PIPESTATUS[0]}
printf '\nend_utc=%s\nssh_exit=%s\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$rc" | tee -a "$root/logs/$log"
exit "$rc"
