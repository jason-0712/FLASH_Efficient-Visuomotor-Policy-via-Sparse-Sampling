#!/usr/bin/env bash
set -u
run() { printf '\n$'; printf ' %q' "$@"; printf '\n'; "$@"; printf 'exit=%s\n' "$?"; }
run command -v rg
for f in /AGENTS.md /home/AGENTS.md /home/hanyu/AGENTS.md; do if [ -f "$f" ]; then run cat "$f"; fi; done
run ls -la /home/hanyu/flash_fr3
run du -sh /home/hanyu/flash_fr3
if command -v rg >/dev/null; then rg --files --hidden /home/hanyu/flash_fr3 | head -100; else find /home/hanyu/flash_fr3 -maxdepth 3 -type f | head -100; fi
run ls -la /home/hanyu/workspace
run ls -la /data
run ls -la /mnt/Nas-Data1
run /home/hanyu/miniconda3/bin/conda env list
run tmux list-sessions
run ps -u hanyu -o pid,ppid,etimes,pcpu,pmem,comm
for d in /data/hanyu /mnt/data1/hanyu /mnt/data2/hanyu /mnt/data3/hanyu /mnt/Nas-Data1/hanyu; do if [ -d "$d" ]; then run ls -la "$d"; fi; done
