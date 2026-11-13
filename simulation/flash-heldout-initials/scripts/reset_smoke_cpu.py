"""Reload frozen inputs and compare three CPU resets/replays with saved traces."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import torch
from cpu_scene import Scene
from load_frozen_initials import load_frozen_initials


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', type=Path, required=True)
    args = ap.parse_args()
    root = args.root
    frozen = root / 'frozen-v1'
    expected = 'ad70b6e7b77873373246002ea0a07e1345312e0e2c10b62e092f939f16b24076'
    rows = load_frozen_initials(frozen / 'test-initials.json', expected_sha256=expected)
    scene = Scene()
    records = []
    for index in (0, 24, 49):
        row = rows[index]
        obs = scene.reset(row['initial_state'])
        samples = [scene.measure(obs)]
        for _ in range(200):
            samples.append(scene.measure(scene.step()))
        arrays = {k: np.asarray([r[k] for r in samples]) for k in samples[0]}
        source = frozen / 'candidate-traces' / f"{row['candidate_id']:04d}.npz"
        with np.load(source, allow_pickle=False) as old:
            for key, values in arrays.items():
                np.testing.assert_array_equal(values, old[key], err_msg=f'test={index}, key={key}')
        records.append({'test_id': index, 'candidate_id': row['candidate_id'], 'all_saved_arrays_bitwise_equal': True,
                        'initial_native_qpos_sha256': hashlib.sha256(arrays['native_qpos'][0].tobytes()).hexdigest(),
                        'samples': len(samples)})
        print(json.dumps(records[-1]), flush=True)
    scene.close()
    report = {'status': 'PASS', 'selection_rule': 'First, middle and last frozen test IDs (0,24,49), selected before replay',
              'test_initials_sha256': expected, 'replays': records, 'policy_executed': False,
              'torch_cuda_available': torch.cuda.is_available(), 'torch_cuda_initialized': torch.cuda.is_initialized()}
    (root / 'results/reset-smoke-cpu.json').write_text(json.dumps(report, indent=2) + '\n')
    print('CPU_FROZEN_RESET_SMOKE_PASS', flush=True)


if __name__ == '__main__':
    main()
