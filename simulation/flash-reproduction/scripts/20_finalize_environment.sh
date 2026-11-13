#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
diff -u "$root/environment/base-flash_fr3-before.txt" "$root/environment/base-flash_fr3-after.txt" || true
cd /home/hanyu/flash_fr3
/home/hanyu/miniconda3/envs/flash_fr3/bin/python -m pip freeze > "$root/environment/base-flash_fr3-after-neutral-cwd.txt"
cmp "$root/environment/base-flash_fr3-before.txt" "$root/environment/base-flash_fr3-after-neutral-cwd.txt"
/home/hanyu/miniconda3/envs/flash_fr3/bin/python -c 'import sys,metasim; print("BASE_INTERPRETER",sys.executable,"BASE_METASIM",metasim.__file__)'
cd "$root/FLASH-Policy"
"$root/.venv/bin/python" - <<'PY'
from pathlib import Path
p=Path('roboverse_learn/il/runners/default_eval_runner.py')
s=p.read_text()
needle='        cfg = payload["cfg"]\n'
assert needle in s
s=s.replace(needle,needle+'        from pathlib import Path\n        loaded_step = dill.loads(payload["pickles"]["global_step"])\n        log.info(f"[Checkpoint loaded] path={Path(kwargs[\"checkpoint_path\"]).resolve()} global_step={loaded_step} seed={cfg.train_config.training_params.seed} use_ema={cfg.train_config.training_params.use_ema}")\n',1)
p.write_text(s)
PY
"$root/.venv/bin/python" -m py_compile roboverse_learn/il/runners/default_eval_runner.py
git diff --check
git rev-parse HEAD > "$root/environment/repository-commit.txt"
git status --short > "$root/environment/repository-status.txt"
git diff > "$root/environment/source-changes.patch"
