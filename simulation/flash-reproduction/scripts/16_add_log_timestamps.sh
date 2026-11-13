#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
cd "$root/FLASH-Policy"
"$root/.venv/bin/python" - <<'PY'
from pathlib import Path
p=Path('roboverse_learn/il/utils/json_logger.py')
s=p.read_text()
assert 'import time\n' not in s
s=s.replace('import os\n','import os\nimport time\n',1)
s=s.replace('        self.last_log = None\n','        self.last_log = None\n        self._started_at = time.perf_counter()\n',1)
needle='        filtered_data = dict(filter(lambda x: self.filter_fn(*x), data.items()))\n'
assert needle in s
s=s.replace(needle,needle+'        filtered_data["wall_time_unix"] = time.time()\n        filtered_data["elapsed_seconds"] = time.perf_counter() - self._started_at\n',1)
p.write_text(s)
PY
git diff --check
git diff > "$root/environment/source-changes.patch"
cat "$root/environment/source-changes.patch"
