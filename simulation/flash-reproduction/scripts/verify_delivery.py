"""Verify local evidence against the server export manifest using stdlib only."""
import argparse
import hashlib
import json
import re
from datetime import datetime, timezone
from pathlib import Path


def sha256(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


p = argparse.ArgumentParser()
p.add_argument('--root', required=True)
a = p.parse_args()
root = Path(a.root).resolve()
records = json.loads((root / 'manifests/raw-evaluation-files.json').read_text())
assert len(records) == 150
for r in records:
    path = root / r['export_path']
    assert path.stat().st_size == r['bytes']
    assert sha256(path) == r['sha256'], path
e = json.loads((root / 'results/evaluation-summary.json').read_text())
rollouts = [json.loads(s) for s in (root / 'results/rollouts.jsonl').read_text().splitlines()]
assert [r['rollout_id'] for r in rollouts] == list(range(50))
independent_successes = 0
for idx, row in enumerate(rollouts):
    text = (root / f'raw-evaluation/{idx:04d}.txt').read_text()
    fields = dict(line.split(':', 1) for line in text.splitlines() if ':' in line)
    assert int(fields['Demo Index']) == idx
    ok = 'True' in fields['SuccessOnce']
    assert ok == row['success']
    assert int(fields['Total Steps']) == row['steps']
    independent_successes += ok
assert independent_successes == e['successes']
train = [json.loads(s) for s in (root / 'logs/training-steps.jsonl').read_text().splitlines()]
assert [r['global_step'] for r in train] == list(range(15015))
for name in ['README_zh.md', 'METHOD_NOTES_zh.md', 'STATUS.md']:
    for url in re.findall(r'\]\(([^)]+)\)', (root / name).read_text()):
        if not url.startswith(('https://', 'http://')):
            assert (root / url).exists(), (name, url)
summary = {'verified_utc': datetime.now(timezone.utc).isoformat(),
           'raw_files_hashed_against_server_manifest': len(records),
           'raw_evidence_bytes': sum(r['bytes'] for r in records),
           'independently_parsed_successes': independent_successes,
           'rollout_count': len(rollouts), 'contiguous_training_log_updates': len(train),
           'report_local_links_exist': True}
(root / 'results/delivery-verification.json').write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
