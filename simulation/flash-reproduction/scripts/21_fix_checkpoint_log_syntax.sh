#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
cd "$root/FLASH-Policy"
"$root/.venv/bin/python" - <<'PY'
from pathlib import Path
p=Path('roboverse_learn/il/runners/default_eval_runner.py')
lines=p.read_text().splitlines()
matches=[i for i,l in enumerate(lines) if 'log.info(f"[Checkpoint loaded]' in l]
assert len(matches)==1
lines[matches[0]]='        log.info("[Checkpoint loaded] path={} global_step={} seed={} use_ema={}", Path(kwargs["checkpoint_path"]).resolve(), loaded_step, cfg.train_config.training_params.seed, cfg.train_config.training_params.use_ema)'
p.write_text('\n'.join(lines)+'\n')
compile(p.read_text(),str(p),'exec')
PY
git diff --check
git diff > "$root/environment/source-changes.patch"
git status --short > "$root/environment/repository-status.txt"
