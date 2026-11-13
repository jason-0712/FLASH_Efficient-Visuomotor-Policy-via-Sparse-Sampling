#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
cd "$root/FLASH-Policy"
"$root/.venv/bin/python" - <<'PY'
from pathlib import Path
p=Path('roboverse_learn/il/utils/lr_scheduler.py')
old='from diffusers.optimization import (\n    TYPE_TO_SCHEDULER_FUNCTION,\n    Optimizer,\n    Optional,\n    SchedulerType,\n    Union,\n)'
new='from typing import Optional, Union\n\nfrom torch.optim import Optimizer\nfrom diffusers.optimization import TYPE_TO_SCHEDULER_FUNCTION, SchedulerType'
s=p.read_text()
assert old in s
p.write_text(s.replace(old,new,1))
PY
git diff --check
git diff > "$root/environment/source-changes.patch"
cat "$root/environment/source-changes.patch"
