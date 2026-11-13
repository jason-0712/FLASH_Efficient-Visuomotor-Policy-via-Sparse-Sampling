"""Compact read-only progress snapshot; final scores require CPU verification."""
from pathlib import Path
import json,datetime
root=Path('/home/hanyu/flash_fr3/flash_sim_reproduction_20261028/heldout_curve_20261029')
print(datetime.datetime.now(datetime.timezone.utc).isoformat())
for step in [2500,5000,7500]:
 for policy in ['flash','fm_dit']:
  p=root/'runs'/str(step)/policy/'rollouts.jsonl';summary=p.parent/'summary.json'
  rows=[json.loads(x) for x in p.read_text().splitlines()] if p.exists() else []
  print(step,policy,'completed',len(rows),'successes_so_far',sum(x['geometric_success_once'] for x in rows),'final' if summary.exists() else 'running_or_pending')
for p in sorted((root/'results').glob('*validation.json')):
 r=json.loads(p.read_text());print(p.name,r.get('status'),r.get('all_trajectories_bitwise_equal'))
for p in sorted((root/'results').glob('job-gpu*-exit.json')):print(p.name,p.read_text().strip())
