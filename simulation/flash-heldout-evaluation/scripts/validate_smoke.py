"""Gate the full run on paired frozen resets and unchanged native controllers."""
import argparse
import json
from pathlib import Path
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('--root', required=True, type=Path)
root = ap.parse_args().root
policies = ['flash', 'fm_dit']
rows, controllers = {}, {}
for policy in policies:
    run = root / 'smoke' / policy
    summary = json.loads((run / 'summary.json').read_text())
    assert summary['rollouts'] == 3 and summary['all_resets_match_frozen_cpu_traces']
    rows[policy] = [json.loads(line) for line in (run / 'rollouts.jsonl').read_text().splitlines()]
    controllers[policy] = json.loads((run / 'native-controller-and-loaded-policy.json').read_text())
    assert controllers[policy]['loaded_ema_all_checkpoint_tensors_equal']
for index in range(3):
    a, b = rows['flash'][index], rows['fm_dit'][index]
    assert a['initial_state_sha256'] == b['initial_state_sha256']
    assert a['reset_fingerprints'] == b['reset_fingerprints']
    with np.load(root / 'smoke/flash/resets' / f'{index:04d}.npz') as x, np.load(root / 'smoke/fm_dit/resets' / f'{index:04d}.npz') as y:
        for key in x.files:
            np.testing.assert_array_equal(x[key], y[key])
for key in ['joint_order', 'actuator_names', 'actuator_gainprm', 'actuator_biasprm', 'dof_damping', 'physics_dt_s', 'decimation']:
    assert controllers['flash'][key] == controllers['fm_dit'][key], key
report = {'status': 'PASS', 'paired_test_ids': [0, 1, 2],
          'paired_actual_resets_and_rgb_bitwise_equal': True,
          'both_resets_match_frozen_cpu_traces': True, 'both_ema_weights_and_normalizers_verified': True,
          'native_controller_arrays_equal': True, 'task_success_not_a_gate': True}
(root / 'results/smoke-validation.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
