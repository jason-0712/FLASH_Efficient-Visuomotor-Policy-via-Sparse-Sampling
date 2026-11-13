"""Read-only cache identity checks, independent of PyTorch and machine paths."""
import hashlib
import json


CACHE_FILES = ('action.npy', 'primary_cam.npy', 'raw_frame_index.npy',
               'state.npy', 'timestamp.npy', 'wrist_cam.npy')


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(8*1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def content_manifest(manifest):
    # Only a source machine's absolute location is excluded. Every other field,
    # including source/cache hashes, split, indices, schema and timing, is kept.
    return {key: value for key, value in manifest.items() if key != 'source_dataset'}


def differing_fields(first, second):
    absent = object()
    return sorted(key for key in first.keys() | second.keys()
                  if first.get(key, absent) != second.get(key, absent))


def verify_cache_identity(cache, reference_path, baseline_config):
    reference_raw = reference_path.read_bytes()
    actual_raw = (cache/'manifest.json').read_bytes()
    reference = json.loads(reference_raw); actual = json.loads(actual_raw)
    actual_sha = hashlib.sha256(actual_raw).hexdigest()
    trained = baseline_config.get('data')
    changed = differing_fields(content_manifest(reference), content_manifest(actual))
    historical_changed = (differing_fields(trained, actual) if isinstance(trained, dict)
                          else ['missing_training_data_manifest'])
    failures = []
    if changed:
        failures.append('参考清单内容不同：'+', '.join(changed))
    if actual_sha != baseline_config.get('cache_manifest_sha256'):
        failures.append('当前清单文件 SHA 与原训练记录不同；未修改或绕过历史摘要检查')
    if historical_changed:
        failures.append('当前清单字段与原训练记录不同：'+', '.join(historical_changed))
    expected_files = reference.get('cache_sha256', {})
    declared_files = actual.get('cache_sha256', {})
    if set(expected_files) != set(CACHE_FILES) or set(declared_files) != set(CACHE_FILES):
        failures.append('缓存文件集合必须恰好包含固定的六个 NPY 文件')
    checks = {}
    for name in CACHE_FILES:
        try:
            measured = file_sha256(cache/name)
            matched = measured == expected_files.get(name) == declared_files.get(name)
            checks[name] = {'reference': expected_files.get(name), 'declared': declared_files.get(name),
                            'actual': measured, 'matches': matched}
            if not matched:
                failures.append('缓存文件内容不一致：'+name)
        except OSError as error:
            checks[name] = {'matches': False, 'error': str(error)}
            failures.append('缓存文件无法读取：'+name)
    return {'status': 'PASS' if not failures else 'FAIL',
            'reference_manifest_sha256': hashlib.sha256(reference_raw).hexdigest(),
            'actual_manifest_sha256': actual_sha,
            'training_manifest_sha256': baseline_config.get('cache_manifest_sha256'),
            'reference_source_dataset': reference.get('source_dataset'),
            'actual_source_dataset': actual.get('source_dataset'),
            'reference_content_fields_differing': changed,
            'training_content_fields_differing': historical_changed,
            'train_episodes': len(actual.get('splits', {}).get('train', [])),
            'validation_episodes': len(actual.get('splits', {}).get('val', [])),
            'total_samples': actual.get('total_samples'),
            'ignored_reference_field': 'source_dataset only; JSON whitespace/key order is immaterial',
            'cache_files': checks, 'failures': failures,
            'cache_or_checkpoint_modified': False, 'robot_motion_commanded': False}
