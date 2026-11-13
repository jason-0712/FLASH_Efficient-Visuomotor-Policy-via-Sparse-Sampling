#!/usr/bin/env python3
"""Compare existing EMA checkpoints, then optionally train a fresh seed.

Offline only. No ROS, robot clients, inference servers or package installation.
The original trainer and algorithm are used without modification.
"""
import argparse
import csv
import fcntl
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

from flash_fr3_phase_metrics import pareto_frontier, selection_vector
from flash_fr3_cache_identity import verify_cache_identity


def read(path):
    return json.loads(path.read_text())


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def sha(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for part in iter(lambda: stream.read(8*1024*1024), b''):
            result.update(part)
    return result.hexdigest()


def gpu_preflight(index):
    raw = subprocess.check_output(['nvidia-smi', '-i', str(index), '--query-gpu=uuid,name',
                                   '--format=csv,noheader'], text=True).strip().splitlines()
    if len(raw) != 1:
        raise ValueError('GPU 索引必须唯一对应一张显卡')
    uuid, name = [value.strip() for value in raw[0].split(',', 1)]
    if not uuid.startswith('GPU-') or 'H100' not in name:
        raise ValueError('本次对照使用已验证的 H100；请检查所选 GPU')
    processes = subprocess.check_output(['nvidia-smi', '--query-compute-apps=gpu_uuid,pid,process_name',
                                         '--format=csv,noheader'], text=True)
    busy = [line for line in processes.splitlines() if line.split(',', 1)[0].strip() == uuid]
    if busy:
        raise RuntimeError(f'物理 GPU {index} 已有计算任务，未启动实验：'+repr(busy))
    return {'physical_index': index, 'uuid': uuid, 'name': name}


def run_command(command, log, env, training_steps=None):
    print('开始：'+' '.join(str(arg) for arg in command), flush=True)
    with log.open('x') as stream:
        process = subprocess.Popen([str(arg) for arg in command], stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, bufsize=1, env=env)
        try:
            for line in process.stdout:
                print(line, end='', flush=True); stream.write(line); stream.flush()
                if training_steps:
                    try:
                        row = json.loads(line); step = int(row.get('step', 0))
                        if step >= 500 and step % 500 == 0 and 'elapsed_seconds' in row:
                            remaining = row['elapsed_seconds']/step*(training_steps-step)/60
                            print('FLASH_COMPARE_ETA='+json.dumps({'step': step,
                                  'estimated_remaining_training_minutes': remaining}), flush=True)
                    except (ValueError, TypeError, KeyError):
                        pass
            code = process.wait()
            if code:
                raise RuntimeError(f'子任务退出码 {code}，日志：{log}')
        except BaseException:
            if process.poll() is None:
                process.terminate()
                try: process.wait(timeout=15)
                except subprocess.TimeoutExpired: process.kill(); process.wait()
            raise


def training_arguments(baseline):
    args = baseline['arguments']
    if (args['seed'] != 42 or args['steps'] != 15000 or args.get('smoke_test') or
            baseline['checkpoint_purpose'] != 'offline_training_requires_real_robot_validation' or
            baseline['scheduler'] != {'name': 'cosine', 'warmup_steps': 500, 'total_steps': 15000}):
        raise ValueError('需要已有 seed42、正式 15000 步、相同日程的训练配置')
    fields = ('batch_size', 'workers', 'val_every', 'val_batches', 'checkpoint_every', 'cpu_threads')
    values = {key: args[key] for key in fields}
    if any(not isinstance(v, int) or v < (0 if k == 'workers' else 1) for k, v in values.items()):
        raise ValueError('已有训练参数异常')
    return [item for key, value in values.items() for item in ('--'+key.replace('_', '-'), str(value))]


def write_comparison(output, candidates):
    frontier = pareto_frontier(candidates)
    rows = []
    for i, candidate in enumerate(candidates):
        vector = selection_vector(candidate['phase_metrics'])
        rows.append({'candidate': candidate['candidate'], 'seed': candidate['seed'],
                     'step': candidate['checkpoint_step'], 'joint_mae_deg': float(vector[0]),
                     'anchor_joint_p95_deg': float(vector[1]), 'transition_gripper_mae': float(vector[2]),
                     'preclose_error_rate': float(vector[3]), 'close_error_rate': float(vector[4]),
                     'release_error_rate': float(vector[5]), 'pareto': i in frontier})
    report = {'status': 'OFFLINE_CHECKPOINT_COMPARISON_COMPLETE', 'candidates': candidates,
              'table': rows, 'pareto_candidates': [candidates[i]['candidate'] for i in frontier],
              'selection_metrics_all_minimized': ['joint_mae_deg', 'anchor_joint_p95_deg',
                  'transition_gripper_mae', 'preclose_error_rate', 'close_error_rate', 'release_error_rate'],
              'selection_rule': 'Pareto frontier only; no weighted score, automatic policy replacement or deployment',
              'validation_reused_for_selection': True, 'independent_test_set': False,
              'robot_motion_commanded': False, 'inference_service_modified': False}
    save(output/'comparison.json', report)
    with (output/'comparison.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    lines = ['# FLASH 检查点对照', '',
             '全部使用同一留出集与 EMA；以下是离线指标，不是独立测试集或任务成功率。', '',
             '| 候选 | seed | step | 关节 MAE° | 锚点 P95° | 阶段夹爪 MAE | 提前闭爪错误率 | 闭爪错误率 | 释放错误率 | Pareto |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for row in rows:
        lines.append('| {candidate} | {seed} | {step} | {joint_mae_deg:.5f} | {anchor_joint_p95_deg:.5f} | '
                     '{transition_gripper_mae:.5f} | {preclose_error_rate:.3f} | {close_error_rate:.3f} | '
                     '{release_error_rate:.3f} | {pareto} |'.format(**row))
    lines += ['', '不自动切换推理服务。Pareto 候选仍需要真机跟踪及连续执行验证。', '']
    (output/'comparison.md').write_text('\n'.join(lines))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('check-cache', 'baseline', 'all'), default='baseline')
    parser.add_argument('--gpu-index', type=int, default=2)
    parser.add_argument('--flash-root', type=Path, default=Path.home()/'flash_fr3/FLASH-Policy')
    parser.add_argument('--reproduction-root', type=Path, default=Path.home()/'flash_fr3/flash_fr3_reproduction')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]; scripts = package/'scripts'
    output = args.output or package/'runs'/('phase_compare_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns()%10**9))
    if output.exists() or not 0 <= args.gpu_index <= 3:
        parser.error('输出目录须为新目录，GPU 索引为 0–3')
    manifest = read(package/'experiment.json')
    cache = args.reproduction_root/'data/pickplace_74eps_stride4'
    baseline_dir = args.reproduction_root/'runs/train_15k_seed42_20261022_160102'
    checkpoints = [baseline_dir/'step_010000.pt', baseline_dir/'step_015000.pt']
    baseline_config = read(baseline_dir/'config.json')
    reference = package/'cache_reference_manifest.json'
    if sha(reference) != manifest['cache_manifest_sha256']:
        raise ValueError('包内参考清单 SHA 与实验配置不一致')
    print('只读核对缓存清单及六个 NPY 文件……', flush=True)
    identity = verify_cache_identity(cache, reference, baseline_config)
    locks = package/'runs'; locks.mkdir(exist_ok=True)
    audit_path = locks/('cache_identity_'+str(time.time_ns())+'.json')
    save(audit_path, identity)
    print('FLASH_CACHE_IDENTITY='+json.dumps({
        key: value for key, value in identity.items() if key != 'cache_files'
    } | {'report': str(audit_path)}, ensure_ascii=False), flush=True)
    if identity['status'] != 'PASS':
        raise ValueError('数据一致性检查失败；未启动评估或训练。详情：'+str(audit_path))
    if args.mode == 'check-cache':
        return
    inherited = training_arguments(baseline_config)
    if any(not path.is_file() for path in checkpoints):
        raise FileNotFoundError('缺少原始 step_010000.pt 或 step_015000.pt，未启动训练')
    if baseline_config['policy'] != manifest['policy']:
        raise ValueError('原始策略配置与实验固定版本不一致')
    required = (12 if args.mode == 'all' else 1)*1024**3
    if shutil.disk_usage(package).free < required:
        raise RuntimeError(f'输出分区需至少 {required/1024**3:.0f} GiB 可用空间')
    gpu = gpu_preflight(args.gpu_index)
    with (locks/('.'+gpu['uuid']+'.lock')).open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
        output.mkdir(parents=True); env = dict(os.environ)
        env.update(CUDA_VISIBLE_DEVICES=gpu['uuid'], PYTHONUNBUFFERED='1')
        state = {'status': 'RUNNING', 'mode': args.mode, 'gpu': gpu,
                 'cache_identity_report': str(audit_path),
                 'baseline_config_sha256': sha(baseline_dir/'config.json'), 'robot_motion_commanded': False,
                 'inference_service_modified': False, 'new_training_seed': 43 if args.mode == 'all' else None}
        save(output/'run_status.json', state); candidates = []
        def invoke(script, fields, name, training=False):
            run_command([sys.executable, '-u', scripts/script, *fields], output/(name+'.log'), env,
                        training_steps=15000 if training else None)
        def evaluate(checkpoint, seed, name):
            directory = output/'evaluations'/name
            invoke('eval_flash_fr3.py', ['--flash-root', args.flash_root, '--dataset-cache', cache,
                    '--checkpoint', checkpoint, '--output', directory, '--device', 'cuda:0',
                    '--batch-size', str(baseline_config['arguments']['batch_size']),
                    '--latency-warmup', '2', '--latency-repeats', '5'], name+'_eval')
            invoke('summarize_flash_fr3_phases.py', ['--evaluation', directory, '--cache', cache], name+'_phases')
            candidate = read(directory/'phase_report.json'); candidate.update(candidate=name, seed=seed)
            if candidate['source'] != baseline_config['source']:
                raise ValueError('对照源码摘要不一致')
            candidates.append(candidate); write_comparison(output, candidates)
        try:
            command = ('import sys,numpy,torch; '
                       'assert sys.version_info[:2]==(3,11), "请激活 flash_fr3 Python3.11 环境"; '
                       'assert torch.__version__.split("+")[0]=="2.7.0"; '
                       'assert numpy.__version__=="1.26.4"; '
                       'assert torch.cuda.is_available(); '
                       'print("FLASH_COMPARE_ENV=PASS", torch.cuda.get_device_name(0))')
            run_command([sys.executable, '-c', command], output/'environment.log', env)
            for checkpoint in checkpoints:
                evaluate(checkpoint, 42, 'seed42_'+checkpoint.stem)
            if args.mode == 'all':
                common = ['--flash-root', args.flash_root, '--dataset-cache', cache, '--device', 'cuda:0']
                invoke('train_flash_fr3.py', [*common, '--output', output/'smoke', '--batch-size', '2',
                        '--seed', '43', '--smoke-test'], 'pipeline_smoke')
                if read(output/'smoke/smoke_result.json')['status'] != 'PASS':
                    raise ValueError('训练链路检查未通过')
                training = output/'seed43_training'
                invoke('train_flash_fr3.py', [*common, '--output', training, '--seed', '43',
                        '--steps', '15000', *inherited], 'seed43_training', training=True)
                for checkpoint in sorted(training.glob('step_*.pt')):
                    evaluate(checkpoint, 43, 'seed43_'+checkpoint.stem)
                evaluate(training/'best.pt', 43, 'seed43_best_joint')
            state.update(status='EXPERIMENT_COMPLETE', candidates=len(candidates), comparison=str(output/'comparison.md'))
            save(output/'run_status.json', state)
            print('FLASH_PHASE_COMPARISON=COMPLETE '+json.dumps(state, ensure_ascii=False), flush=True)
        except BaseException as error:
            state.update(status='EXPERIMENT_FAILED', error=str(error) or type(error).__name__)
            save(output/'run_status.json', state)
            raise


if __name__ == '__main__':
    main()
