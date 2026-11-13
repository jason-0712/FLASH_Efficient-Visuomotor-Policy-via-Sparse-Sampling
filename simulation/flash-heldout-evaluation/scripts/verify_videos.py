"""CPU-decode every formal video and compare frame counts with control steps."""
import argparse
import json
from pathlib import Path
import imageio_ffmpeg

ap = argparse.ArgumentParser()
ap.add_argument('--root', required=True, type=Path)
root = ap.parse_args().root
records = []
for policy in ['flash', 'fm_dit']:
    run = root / 'runs' / policy
    official, = [p for p in (run / 'official-style/eval').rglob('10000_*.ckpt_*') if p.is_dir()]
    rows = [json.loads(line) for line in (run / 'rollouts.jsonl').read_text().splitlines()]
    assert len(rows) == 50
    for row in rows:
        video = official / f"{row['test_id']}.mp4"
        frames, seconds = imageio_ffmpeg.count_frames_and_secs(str(video))
        assert frames == row['control_steps'], (policy, row['test_id'], frames, row['control_steps'])
        records.append({'policy': policy, 'test_id': row['test_id'], 'frames': frames,
                        'container_seconds': seconds, 'simulation_seconds': row['episode_simulated_seconds'],
                        'video': str(video.relative_to(root))})
report = {'status': 'PASS', 'formal_videos_decoded': len(records),
          'all_frame_counts_match_recorded_control_steps': True,
          'note': 'Container duration is distinct from 15 ms/control-step simulation duration.',
          'ffmpeg_binary': imageio_ffmpeg.get_ffmpeg_exe(), 'records': records}
(root / 'results/video-validation.json').write_text(json.dumps(report, indent=2) + '\n')
print('CPU_VIDEO_VALIDATION_PASS', len(records))
