#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
test ! -e "$root"
mkdir -p "$root"/{environment,scripts,configs,manifests,logs,results,videos}
git clone --no-hardlinks /home/hanyu/flash_fr3/FLASH-Policy "$root/FLASH-Policy"
git -C "$root/FLASH-Policy" checkout --detach f47e490f2b820f835d1b9145da3cdb3df69f50a0
git -C "$root/FLASH-Policy" remote set-url origin https://github.com/NTUMARS/FLASH-Policy.git
/home/hanyu/miniconda3/envs/flash_fr3/bin/python -m pip freeze > "$root/environment/base-flash_fr3-before.txt"
/home/hanyu/miniconda3/envs/flash_fr3/bin/python -m venv --system-site-packages "$root/.venv"
cat > "$root/environment/mujoco-constraints.txt" <<'EOF'
numpy==1.26.4
torch==2.7.0+cu128
torchvision==0.22.0+cu128
zarr==2.18.3
numcodecs==0.13.0
EOF
"$root/.venv/bin/python" -m pip install --disable-pip-version-check --no-input -c "$root/environment/mujoco-constraints.txt" mujoco dm-control 2>&1 | tee "$root/logs/install-mujoco.log"
"$root/.venv/bin/python" -m pip install --disable-pip-version-check --no-input --no-deps -e "$root/FLASH-Policy" 2>&1 | tee "$root/logs/install-editable.log"
"$root/.venv/bin/python" -m pip freeze > "$root/environment/pip-freeze-after.txt"
"$root/.venv/bin/python" -m pip check 2>&1 | tee "$root/environment/pip-check.txt"
