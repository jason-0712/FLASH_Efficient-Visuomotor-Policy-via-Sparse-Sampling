#!/usr/bin/env bash
set -u
run() { printf '\n$'; printf ' %q' "$@"; printf '\n'; "$@"; printf 'exit=%s\n' "$?"; }
repo=/home/hanyu/flash_fr3/FLASH-Policy
run git -C "$repo" rev-parse HEAD
run git -C "$repo" status --short
run git -C "$repo" diff --stat
run git -C "$repo" remote -v
run cat /home/hanyu/flash_fr3/flash-source-commit.txt /home/hanyu/flash_fr3/flash-constraints.txt
run cat /home/hanyu/flash_fr3/flash-environment.txt
run tail -60 /home/hanyu/flash_fr3/flash_fr3_reproduction/runs/train_15k_seed42_20261022_160102.log
run find "$repo" -maxdepth 3 -name AGENTS.md -o -name '*.zarr' -o -name '*.ckpt' -o -name roboverse_data -o -name roboverse_demo -o -name data_policy -o -name il_outputs
run ls -la "$repo/roboverse_data"
run /home/hanyu/miniconda3/envs/flash_fr3/bin/python -c 'import sys, importlib.metadata as m, importlib.util as u; print(sys.executable, sys.version); packages=["torch","torchvision","numpy","mujoco","dm-control","zarr","numcodecs","hydra-core","wandb","torchcfm","tyro","roboverse-py"]; print({p:m.version(p) if m.packages_distributions().get(p.replace("-","_")) or p in {d.metadata["Name"] for d in m.distributions()} else "missing" for p in packages}); import torch; print("CUDA",torch.cuda.is_available()); print("metasim",u.find_spec("metasim")); print("isaacsim",u.find_spec("isaacsim")); print("isaaclab",u.find_spec("isaaclab"))'
run find /home/hanyu/flash_fr3/flash_fr3_reproduction -maxdepth 4 -type f -name '*.json' -o -name '*.pt' -o -name '*.ckpt'
run ls -la /home/hanyu/.cache/huggingface/hub
exit 0
