"""Validate real collector outputs; preserve source IDs, hashes and video statistics."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw


def sha256(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--root', required=True)
    p.add_argument('--expected', type=int, required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--zarr')
    a = p.parse_args()
    root, output = Path(a.root), Path(a.output)
    paths = sorted((root/'success').glob('demo_*'))
    assert len(paths) == a.expected, (len(paths), a.expected)
    report = {'root': str(root), 'expected_successes': a.expected, 'demonstrations': [],
              'physical_control_dt_seconds': 0.015,
              'video_fps_note': 'Video container fps is a playback setting, not simulation control frequency.'}
    tiles = []
    total_sampled = 0
    for d in paths:
        mpath, vpath = d/'metadata.json', d/'rgb.mp4'
        assert mpath.is_file() and vpath.is_file(), d
        assert (d/'status.txt').read_text().strip() == 'success', d
        m = json.loads(mpath.read_text())
        q = np.asarray(m['joint_qpos'], dtype=np.float32)
        act = np.asarray(m['joint_qpos_target'], dtype=np.float32)
        assert q.ndim == 2 and q.shape[1] == 9 and act.shape == q.shape, (d,q.shape,act.shape)
        assert np.isfinite(q).all() and np.isfinite(act).all(), d
        cap = cv2.VideoCapture(str(vpath))
        fps = cap.get(cv2.CAP_PROP_FPS)
        n, sample_frames, frame_std, changes, last = 0, {}, [], [], None
        sampled_rgb = []
        while True:
            ok, bgr = cap.read()
            if not ok:
                break
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            assert rgb.shape == (256,256,3), (d,rgb.shape)
            frame_std.append(float(rgb.std()))
            if last is not None:
                changes.append(float(np.abs(rgb.astype(float)-last.astype(float)).mean()))
            if n in {0,len(q)//2,len(q)-1}:
                sample_frames[n] = rgb.copy()
            if a.zarr and n % 4 == 0:
                sampled_rgb.append(rgb.copy())
            last = rgb
            n += 1
        cap.release()
        assert n == len(q), (d,n,len(q))
        assert min(frame_std) > 5 and max(changes, default=0) > 0.01, d
        item = {'demo_id':d.name, 'source_reference_index':int(d.name.split('_')[1]),
                'frames':n, 'joint_shape':list(q.shape), 'video_container_fps':fps,
                'rgb_std_min':min(frame_std), 'rgb_mean_abs_frame_change_max':max(changes),
                'metadata_sha256':sha256(mpath), 'rgb_sha256':sha256(vpath),
                'task_description':m.get('task_desc')}
        report['demonstrations'].append(item)
        if len(tiles)<6:
            for idx in sorted(sample_frames):
                im = Image.fromarray(sample_frames[idx])
                tile = Image.new('RGB',(256,280),'white')
                tile.paste(im,(0,24))
                ImageDraw.Draw(tile).text((6,6),f'{d.name} frame {idx}',fill='black')
                tiles.append(tile)
        if a.zarr:
            import zarr
            z = zarr.open(a.zarr, mode='r')
            count = len(range(0,n,4))
            np.testing.assert_allclose(z['data/state'][total_sampled:total_sampled+count],q[::4],atol=1e-6)
            np.testing.assert_allclose(z['data/action'][total_sampled:total_sampled+count],act[::4],atol=1e-6)
            stored_rgb = np.moveaxis(z['data/head_camera'][total_sampled:total_sampled+count],1,-1)
            rgb_diff = np.abs(stored_rgb.astype(np.int16)-np.asarray(sampled_rgb,dtype=np.int16))
            item['zarr_rgb_abs_diff_max'] = int(rgb_diff.max())
            item['zarr_rgb_abs_diff_mean'] = float(rgb_diff.mean())
            assert rgb_diff.max() <= 2, (d,'video/Zarr image alignment mismatch',rgb_diff.max())
            total_sampled += count
            assert int(z['meta/episode_ends'][len(report['demonstrations'])-1]) == total_sampled
    if a.zarr:
        assert len(z['meta/episode_ends']) == a.expected
        assert z['data/head_camera'].shape == (total_sampled,3,256,256)
        assert np.isfinite(z['data/state'][:]).all() and np.isfinite(z['data/action'][:]).all()
        report['zarr'] = {'path':a.zarr,'samples':total_sampled,'arrays':{k:list(v.shape) for k,v in z['data'].arrays()},
                          'meta_attributes':dict(z['meta'].attrs),
                          'file_sha256':{str(f.relative_to(a.zarr)):sha256(f) for f in sorted(Path(a.zarr).rglob('*')) if f.is_file()}}
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,indent=2,ensure_ascii=False))
    sheet=Image.new('RGB',(256*3,280*2),'white')
    for i,tile in enumerate(tiles):
        sheet.paste(tile,(i%3*256,i//3*280))
    sheet.save(output.with_suffix('.png'))
    print(json.dumps({'manifest':str(output),'success_count':len(paths),'zarr_samples':total_sampled if a.zarr else None,
                      'demo_ids':[x['demo_id'] for x in report['demonstrations']]},indent=2))


if __name__ == '__main__':
    main()
