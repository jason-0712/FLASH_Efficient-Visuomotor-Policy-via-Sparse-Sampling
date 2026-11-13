"""Export verified evidence after the official run; never fabricates a rollout."""
import argparse
import csv
import hashlib
import json
import shutil
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw


def sha256(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def contact_sheet(video, dest, label):
    cap = cv2.VideoCapture(str(video))
    assert cap.isOpened(), video
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    assert count > 0, video
    frames = []
    for index in np.linspace(0, count - 1, 6).astype(int):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(index))
        ok, bgr = cap.read()
        assert ok, (video, index)
        frames.append((index, Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))))
    cap.release()
    w, h = frames[0][1].size
    canvas = Image.new('RGB', (w * 3, (h + 24) * 2), 'white')
    draw = ImageDraw.Draw(canvas)
    for j, (index, frame) in enumerate(frames):
        x, y = j % 3 * w, j // 3 * (h + 24)
        draw.text((x + 3, y + 4), f'{label}, frame {index}', fill='black')
        canvas.paste(frame, (x, y + 24))
    canvas.save(dest)
    return {'frames': count, 'width': w, 'height': h, 'container_fps': fps}


p = argparse.ArgumentParser()
p.add_argument('--root', required=True)
a = p.parse_args()
root = Path(a.root)
train = json.loads((root / 'results/training-summary.json').read_text())
evaluation = json.loads((root / 'results/evaluation-summary.json').read_text())
assert json.loads((root / 'results/training-process-exit.json').read_text())['exit_code'] == 0
assert json.loads((root / 'results/evaluation-process-exit.json').read_text())['exit_code'] == 0
run = Path(train['run_dir'])
eval_dir = Path(evaluation['evaluation_dir'])
rows = [json.loads(s) for s in (run / 'logs.json.txt').read_text().splitlines() if s]
rollouts = [json.loads(s) for s in (root / 'results/rollouts.jsonl').read_text().splitlines() if s]
assert len(rollouts) == 50
raw = root / 'raw-evaluation'
raw.mkdir(exist_ok=True)
(raw / 'trajectory-diagnostics').mkdir(exist_ok=True)
files = []
for r in rollouts:
    for key, subdir in [('raw_record_path', ''), ('video_path', ''), ('trajectory_diagnostics_path', 'trajectory-diagnostics')]:
        src = Path(r[key])
        dst = raw / subdir / src.name
        shutil.copy2(src, dst)
        digest = sha256(dst)
        if key == 'video_path':
            assert digest == r['video_sha256']
            cap = cv2.VideoCapture(str(dst))
            assert cap.isOpened()
            frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            ok, frame = cap.read()
            cap.release()
            assert ok and frame_count == r['steps'] and frame.std() > 5, (dst, frame_count)
        files.append({'source_path': str(src), 'export_path': str(dst.relative_to(root)),
                      'bytes': dst.stat().st_size, 'sha256': digest})
    with np.load(r['trajectory_diagnostics_path']) as diag:
        for key in ['pos_desired', 'pos_actual', 'vel_desired', 'vel_actual']:
            assert diag[key].shape == (r['steps'], 9), (r['rollout_id'], key, diag[key].shape)
            assert np.isfinite(diag[key]).all()
        assert np.isclose(float(diag['control_dt']), 0.015)
for src in eval_dir.glob('*.txt'):
    if len(src.stem) != 4 or not src.stem.isdigit():
        shutil.copy2(src, raw / src.name)
(root / 'manifests/raw-evaluation-files.json').write_text(json.dumps(files, indent=2))
shutil.copy2(run / 'logs.json.txt', root / 'logs/training-steps.jsonl')

steps = np.array([r['global_step'] for r in rows])
losses = np.array([r['train_loss'] for r in rows])
learning_rates = np.array([r['lr'] for r in rows])
fig, ax = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True, constrained_layout=True)
ax[0].plot(steps, losses, color='#4285b4', alpha=.15, linewidth=.6, label='Raw training loss')
window = 100
ax[0].plot(steps[window-1:], np.convolve(losses, np.ones(window)/window, mode='valid'),
           color='#174c78', linewidth=1.4, label='Trailing 100-update mean')
ax[0].set_yscale('log')
ax[0].set_ylabel('Training loss (log scale)')
ax[0].legend()
ax[0].set_title('FLASH / Pick-Place Bowl / training seed 42')
ax[1].plot(steps, learning_rates, color='#7c4d9e')
ax[1].set_ylabel('Learning rate')
ax[1].set_xlabel('Logged global step (zero based)')
for axis in ax:
    axis.axvline(10000, color='#bb7924', linestyle='--', linewidth=1)
    axis.grid(alpha=.2)
fig.savefig(root / 'results/training-curves.png', dpi=160)
plt.close(fig)

fig, ax = plt.subplots(figsize=(12, 3.4), constrained_layout=True)
colors = ['#2d8d63' if r['success'] else '#bd5656' for r in rollouts]
ax.bar([r['rollout_id'] for r in rollouts], [r['steps'] for r in rollouts], color=colors)
ax.set(xlabel='Official reference initial-state ID', ylabel='Control steps until termination',
       title=f"10000-step EMA checkpoint: {evaluation['successes']}/50 success (green); failure (red)",
       ylim=(0, 320), xlim=(-1, 50))
ax.grid(axis='y', alpha=.2)
fig.savefig(root / 'results/rollout-outcomes.png', dpi=160)
plt.close(fig)

selected = []
for desired in [True, False]:
    match = next((r for r in rollouts if r['success'] == desired), None)
    if match is None:
        continue
    tag = 'success' if desired else 'failure'
    target = root / 'videos' / f"policy-{tag}-rollout{match['rollout_id']:04d}.mp4"
    shutil.copy2(match['video_path'], target)
    info = contact_sheet(target, target.with_suffix('.png'), f"Policy {tag} ID {match['rollout_id']}")
    diagnostic = eval_dir / 'vel_diag' / f"diag_demo{match['rollout_id']}.png"
    if diagnostic.exists():
        shutil.copy2(diagnostic, root / 'results' / f"diagnostic-{tag}-rollout{match['rollout_id']:04d}.png")
    selected.append({'rollout_id': match['rollout_id'], 'success': desired,
                     'video': str(target.relative_to(root)), 'sha256': sha256(target), **info})

gpu = []
with (root / 'logs/gpu-training.csv').open() as f:
    for r in csv.DictReader(f, skipinitialspace=True):
        memory = next(v for k, v in r.items() if k.startswith('memory.used'))
        util = next(v for k, v in r.items() if k.startswith('utilization.gpu'))
        gpu.append((float(memory.split()[0]), float(util.split()[0])))
start = json.loads((root / 'results/training-start.json').read_text())
end = json.loads((root / 'results/training-process-exit.json').read_text())
from datetime import datetime
elapsed = (datetime.fromisoformat(end['ended_utc'].replace('Z', '+00:00')) -
           datetime.fromisoformat(start['started_utc'].replace('Z', '+00:00'))).total_seconds()
export = {'rollout_files_exported': len(files), 'verified_video_count': 50,
          'finite_trajectory_diagnostics_count': 50, 'selected_videos': selected,
          'training_process_wall_seconds': elapsed,
          'gpu_sample_count': len(gpu), 'gpu_sampling_interval_seconds': 5,
          'training_sampled_peak_gpu_memory_mib': max(x[0] for x in gpu),
          'training_sampled_mean_gpu_utilization_percent': float(np.mean([x[1] for x in gpu])),
          'note': 'GPU peak is a sampled device-wide peak, not an allocator peak. Videos precede each control step; success may occur on the final unrecorded post-action observation.'}
(root / 'results/export-summary.json').write_text(json.dumps(export, indent=2))
print(json.dumps(export, indent=2))
