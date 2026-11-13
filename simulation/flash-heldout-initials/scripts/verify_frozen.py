"""Independent offline verification; never imports generation code or a policy."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import numpy as np
from load_frozen_initials import load_frozen_initials, state_hash


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def configuration(state):
    return np.asarray([*state['akita_black_bowl']['pos'][:2], *state['plate']['pos'][:2]])


def inspect_trace(path, protocol):
    """Recompute all acceptance decisions directly from saved native traces."""
    with np.load(path, allow_pickle=False) as loaded:
        z = {k: loaded[k] for k in loaded.files}
    count = protocol['passive_check_control_steps'] + 1
    assert all(len(v) == count and np.isfinite(v).all() for v in z.values())
    np.testing.assert_array_equal(z['time_s'], np.arange(count) * protocol['control_dt_s'])
    bowl, plate = z['akita_black_bowl_pose'], z['plate_pose']
    dz = bowl[:, 2] - plate[:, 2]
    goal = (np.linalg.norm(bowl[:, :2] - plate[:, :2], axis=1) < .06) & (dz > 0) & (dz < .03)
    np.testing.assert_array_equal(goal, z['geometric_success'])
    rules = protocol['acceptance']
    reasons = []
    if goal.any():
        reasons.append('already_satisfies_goal')
    penetration = float(z['minimum_object_contact_distance'].min())
    if penetration < -rules['object_contact_penetration_max_m']:
        reasons.append('object_penetration_exceeds_limit')
    metrics = {'minimum_object_contact_distance_m': penetration}
    tail = slice(-protocol['tail_samples_for_stability'], None)
    for name in ('akita_black_bowl', 'plate'):
        positions = z[name + '_pose'][:, :3]
        m = {
            'initial_support_contacts': int(z[name + '_support'][0]),
            'tail_min_support_contacts': int(z[name + '_support'][tail].min()),
            'max_xy_drift_m': float(np.max(np.sqrt(np.sum((positions[:, :2] - positions[0, :2])**2, axis=1)))),
            'max_xyz_drift_m': float(np.max(np.sqrt(np.sum((positions - positions[0])**2, axis=1)))),
            'tail_max_linear_speed_m_s': float(np.sqrt(np.sum(z[name + '_linear_velocity'][tail]**2, axis=1)).max()),
            'tail_max_angular_speed_rad_s': float(np.sqrt(np.sum(z[name + '_angular_velocity'][tail]**2, axis=1)).max()),
            'tail_z_range_m': float(positions[tail, 2].max() - positions[tail, 2].min()),
            'max_robot_contacts': int(z[name + '_robot'].max()),
            'max_cabinet_contacts': int(z[name + '_cabinet'].max()),
        }
        for field, limit in [('initial_support_contacts', 'initial_support_contacts_per_object_min'),
                             ('tail_min_support_contacts', 'tail_support_contacts_per_object_min')]:
            if m[field] < rules[limit]:
                reasons.append(name + ':' + field)
        for field, limit in [('max_xy_drift_m', 'object_xy_drift_max_m'),
                             ('max_xyz_drift_m', 'object_xyz_drift_max_m'),
                             ('tail_max_linear_speed_m_s', 'tail_linear_speed_max_m_s'),
                             ('tail_max_angular_speed_rad_s', 'tail_angular_speed_max_rad_s'),
                             ('tail_z_range_m', 'tail_z_range_max_m'),
                             ('max_robot_contacts', 'forbidden_object_contacts_max'),
                             ('max_cabinet_contacts', 'forbidden_object_contacts_max')]:
            if m[field] > rules[limit]:
                reasons.append(name + ':' + field)
        metrics[name] = m
    metrics['max_bowl_plate_contacts'] = int(z['bowl_plate'].max())
    if metrics['max_bowl_plate_contacts'] > rules['forbidden_object_contacts_max']:
        reasons.append('bowl_plate_contact')
    return reasons, metrics


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', type=Path, required=True)
    args = ap.parse_args()
    root = args.root
    p = json.loads((root / 'configs/generation-protocol.json').read_text())
    split = json.loads((root / 'configs/reference-split.json').read_text())
    frozen = root / 'frozen-v1'
    generation_environment = json.loads((frozen / 'environment.json').read_text())
    assert np.__version__ == generation_environment['numpy'], (
        'Exact stream reconstruction requires the generation NumPy version; '
        'other versions may round uniform samples differently. Frozen file bytes remain authoritative.'
    )
    d = json.loads((frozen / 'test-initials.json').read_text())
    assert sha(root / 'configs/reference-split.json') == p['source_reference_split_sha256'] == d['source_reference_split_sha256']
    assert sha(root / 'configs/generation-protocol.json') == d['generation_protocol_sha256']
    refs = split['initial_states']
    originals = np.stack([configuration(r['initial_state']) for r in refs])
    original_hashes = {state_hash(r['initial_state']) for r in refs}
    for r in refs:
        assert state_hash(r['initial_state']) == r['initial_state_sha256']
    base = next(r['initial_state'] for r in refs if r['reference_index'] == p['canonical_base_reference_id'])
    training = [r for r in refs if r['reference_index'] in p['bounds_reference_ids']]
    assert len(training) == 39
    training_xy = np.stack([configuration(r['initial_state']) for r in training])
    bounds = np.asarray([p['coordinate_bounds_m'][obj][axis] for obj in ('akita_black_bowl', 'plate') for axis in ('x', 'y')])
    np.testing.assert_array_equal(bounds[:, 0], training_xy.min(axis=0))
    np.testing.assert_array_equal(bounds[:, 1], training_xy.max(axis=0))
    rows = [json.loads(line) for line in (frozen / 'all-candidates.jsonl').read_text().splitlines()]
    rng = np.random.Generator(np.random.PCG64(p['seed']))
    accepted, rejected, metrics = [], [], []
    seen = set()
    for i, r in enumerate(rows):
        assert r['candidate_id'] == i
        reconstructed = copy.deepcopy(base)
        for obj in ('akita_black_bowl', 'plate'):
            for axis, index in [('x', 0), ('y', 1)]:
                reconstructed[obj]['pos'][index] = float(rng.uniform(*p['coordinate_bounds_m'][obj][axis]))
        assert reconstructed == r['initial_state']
        h = state_hash(reconstructed)
        assert h == r['state_sha256']
        distance = np.linalg.norm(originals - configuration(reconstructed), axis=1)
        np.testing.assert_allclose(distance.min(), r['nearest_original_xy4_distance_m'], rtol=1e-14, atol=0)
        assert refs[int(distance.argmin())]['reference_index'] == r['nearest_original_reference_id']
        reasons = []
        if h in original_hashes:
            reasons.append('exact_original_state')
        if h in seen:
            reasons.append('duplicate_test_state')
        if distance.min() < p['minimum_distance_to_all_original_50_m']:
            reasons.append('too_close_to_original_object_configuration')
        if reasons:
            assert r['passive_trace'] is None
        else:
            reasons, recomputed = inspect_trace(frozen / r['passive_trace'], p)
            assert recomputed == r['physical_metrics']
        assert sorted(reasons) == sorted(r['rejection_reasons'])
        assert r['accepted'] == (len(reasons) == 0)
        if reasons:
            rejected.append({'candidate_id': i, 'reasons': reasons})
        else:
            seen.add(h)
            accepted.append(i)
            metrics.append(r['physical_metrics'])
        assert len(accepted) < 50 or i == len(rows) - 1
    assert len(accepted) == d['count'] == 50
    assert accepted == [r['candidate_id'] for r in d['states']]
    assert [rows[i]['state_sha256'] for i in accepted] == [r['state_sha256'] for r in d['states']]
    assert [rows[i]['initial_state'] for i in accepted] == [r['initial_state'] for r in d['states']]
    controls = json.loads((frozen / 'negative-controls.json').read_text())
    negative_results = []
    for c in controls:
        reasons, _ = inspect_trace(frozen / 'negative-controls' / (c['name'] + '.npz'), p)
        if np.linalg.norm(originals - configuration(c['input_state']), axis=1).min() < p['minimum_distance_to_all_original_50_m']:
            reasons.append('too_close_to_original_object_configuration')
        assert sorted(reasons) == sorted(c['rejection_reasons']) and reasons
        expected = {'exact_original_reference_1': 'too_close_to_original_object_configuration',
                    'already_at_goal': 'already_satisfies_goal',
                    'bowl_floating_10cm': 'akita_black_bowl:initial_support_contacts',
                    'bowl_interpenetrating_floor_2cm': 'object_penetration_exceeds_limit'}
        assert expected[c['name']] in reasons
        negative_results.append({'name': c['name'], 'independent_rejection_reasons': reasons})
    loaded = load_frozen_initials(frozen / 'test-initials.json', expected_sha256=sha(frozen / 'test-initials.json'))
    assert loaded == d['states']
    loaded[0]['initial_state']['plate']['pos'][0] += 1
    assert loaded[0] != load_frozen_initials(frozen / 'test-initials.json', expected_sha256=sha(frozen / 'test-initials.json'))[0]
    try:
        load_frozen_initials(frozen / 'test-initials.json', expected_sha256='0' * 64)
    except ValueError:
        pass
    else:
        raise AssertionError('Checksum mismatch was accepted')
    report = {
        'status': 'PASS', 'candidate_count': len(rows), 'selected_count': len(accepted),
        'rejected_candidates': rejected, 'negative_controls': negative_results,
        'independent_pcg64_stream_and_first_50_selection_verified': True,
        'original_50_exact_state_and_configuration_disjoint': True,
        'selected_traces_recomputed': len(metrics), 'samples_per_trace': p['passive_check_control_steps'] + 1,
        'minimum_distance_to_original_50_xy4_m': min(rows[i]['nearest_original_xy4_distance_m'] for i in accepted),
        'maximum_object_xyz_drift_m': max(m[obj]['max_xyz_drift_m'] for m in metrics for obj in ('akita_black_bowl', 'plate')),
        'maximum_tail_linear_speed_m_s': max(m[obj]['tail_max_linear_speed_m_s'] for m in metrics for obj in ('akita_black_bowl', 'plate')),
        'minimum_object_contact_distance_m': min(m['minimum_object_contact_distance_m'] for m in metrics),
        'known_negative_second_finger_inherited': base['franka']['dof_pos']['panda_finger_joint2'][0],
        'test_initials_sha256': sha(frozen / 'test-initials.json'),
        'policy_executed': False, 'verification_script_sha256': sha(__file__),
        'numpy_version': np.__version__,
    }
    (root / 'results').mkdir(exist_ok=True)
    (root / 'results/independent-verification.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
