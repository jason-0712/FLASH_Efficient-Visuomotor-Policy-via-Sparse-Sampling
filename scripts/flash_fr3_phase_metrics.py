"""Full held-out phase metrics. Pure NumPy; no model or hardware execution."""
import numpy as np


def expected_samples(manifest):
    selected = set(manifest['splits']['val'])
    return np.asarray([i for ep in manifest['episodes'] if ep['episode_index'] in selected
                       for i in range(ep['start'], ep['end'])], dtype=np.int64)


def phase_metrics(prediction, current, state, action, manifest):
    p = np.asarray(prediction, dtype=np.float64)
    current = np.asarray(current)
    expected = expected_samples(manifest)
    if (current.dtype.kind not in 'iu' or not np.array_equal(current, expected) or
            p.shape != (len(expected), 8, 8) or not np.isfinite(p).all()):
        raise ValueError('必须按缓存顺序评估全部留出窗口，不能缺失、重复、重排或含非有限值')
    if state.shape != action.shape or action.shape[1:] != (8,):
        raise ValueError('状态和动作缓存形状异常')
    if manifest['sample_dt_seconds'] != 4/15:
        raise ValueError('阶段指标要求本次固定的物理采样间隔')
    lookup = {int(index): row for row, index in enumerate(current)}
    target = np.empty_like(p)
    masks = {name: np.zeros(len(current), dtype=bool)
             for name in ('close_near', 'release_near', 'preclose_2', 'close_anchor', 'release_anchor')}
    events = []; groups = {}
    for ep in manifest['episodes']:
        if ep['episode_index'] not in manifest['splits']['val']:
            continue
        start, end = ep['start'], ep['end']
        indices = np.arange(start, end); rows = np.asarray([lookup[int(i)] for i in indices])
        target[rows] = action[np.minimum(indices[:, None]+np.arange(8), end-1)]
        label = np.asarray(action[start:end, 7]) > .5
        for kind, changes in (('close', (~label[:-1])&label[1:]), ('release', label[:-1]&(~label[1:]))):
            for offset in np.flatnonzero(changes)+1:
                index = start+int(offset); row = lookup[index]
                masks[kind+'_anchor'][row] = True
                masks[kind+'_near'][rows[abs(indices-index) <= 2]] = True
                if kind == 'close' and index-2 >= start and not label[offset-2]:
                    masks['preclose_2'][lookup[index-2]] = True
                events.append({'episode': ep['episode_index'], 'group': ep.get('group', 'unknown'),
                               'kind': kind, 'cache_index': index,
                               'seconds_from_episode_start': offset*manifest['sample_dt_seconds'],
                               'target_closed': bool(label[offset]),
                               'predicted_closure': float(p[row, 0, 7]),
                               'wrong_open_closed': bool((p[row, 0, 7] > .5) != label[offset])})
        groups.setdefault(ep.get('group', 'unknown'), []).extend(rows.tolist())
    error = abs(p-target)
    anchor = abs(p[:, 0, :7]-np.asarray(state[current, :7], float))

    def summarize(mask):
        n = int(np.count_nonzero(mask))
        if not n:
            return {'windows': 0, 'joint_mae_deg': None, 'gripper_mae': None,
                    'wrong_anchor_open_closed': 0, 'anchor_open_closed_error_rate': None}
        wrong = (p[mask, 0, 7] > .5) != (target[mask, 0, 7] > .5)
        return {'windows': n, 'joint_mae_deg': float(np.rad2deg(error[mask, :, :7]).mean()),
                'gripper_mae': float(error[mask, :, 7].mean()),
                'wrong_anchor_open_closed': int(wrong.sum()),
                'anchor_open_closed_error_rate': float(wrong.mean())}

    masks['transition_union'] = masks['close_near']|masks['release_near']
    masks['other'] = ~masks['transition_union']
    return {'format': 'flash_fr3_phase_metrics_v1', 'validation_windows': len(current),
            'all_points_including_end_padding': summarize(np.ones(len(current), dtype=bool)),
            'current_anchor': {'joint_mae_deg': float(np.rad2deg(anchor).mean()),
                               'joint_p95_deg': float(np.rad2deg(np.percentile(anchor, 95))),
                               'joint_max_deg': float(np.rad2deg(anchor.max()))},
            'phase': {name: summarize(mask) for name, mask in masks.items()},
            'per_source_group': {name: summarize(np.isin(np.arange(len(current)), rows))
                                 for name, rows in groups.items()},
            'event_anchors': events, 'classification_threshold': .5,
            'classification_scope': 'raw offline model outputs versus command labels; no execution state machine',
            'selection_scope': 'validation set reused for checkpoint selection; not an independent test set or physical success rate'}


def selection_vector(metrics):
    values = [metrics['all_points_including_end_padding']['joint_mae_deg'],
              metrics['current_anchor']['joint_p95_deg'],
              metrics['phase']['transition_union']['gripper_mae'],
              *(metrics['phase'][name]['anchor_open_closed_error_rate']
                for name in ('preclose_2', 'close_anchor', 'release_anchor'))]
    if any(v is None for v in values) or not np.isfinite(values).all():
        raise ValueError('缺少比较所需的阶段或指标')
    return np.asarray(values, float)


def pareto_frontier(candidates):
    """All metrics minimized. No invented weighted score or auto promotion."""
    values = [selection_vector(c['phase_metrics']) for c in candidates]
    return [i for i, v in enumerate(values) if not any(
        j != i and np.all(other <= v) and np.any(other < v) for j, other in enumerate(values))]
