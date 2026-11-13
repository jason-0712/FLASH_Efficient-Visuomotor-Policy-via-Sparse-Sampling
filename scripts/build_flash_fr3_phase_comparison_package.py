#!/usr/bin/env python3
"""Build a small source-only H100 comparison package; no network or training."""
import hashlib
import json
import shutil
import tarfile
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    root=Path(__file__).resolve().parents[1]
    name='flash_fr3_phase_compare_20261104_v2';destination=root/'artifacts/flash_fr3/packages'
    package=destination/name;archive=destination/(name+'.tar.gz')
    if package.exists() or archive.exists():
        raise FileExistsError('实验包已存在；保留已交付版本，不覆盖')
    (package/'scripts').mkdir(parents=True)
    files=['train_flash_fr3.py','flash_fr3_dataset.py','eval_flash_fr3.py','flash_fr3_phase_metrics.py',
           'summarize_flash_fr3_phases.py','run_flash_fr3_phase_comparison.py','flash_fr3_cache_identity.py',
           'run_flash_fr3_phase_comparison.sh','test_flash_fr3_phase_comparison.py']
    for name in files:shutil.copyfile(root/'scripts'/name,package/'scripts'/name)
    source_config=json.loads((root/'artifacts/flash_fr3/cpu_smoke_verified/config.json').read_text())
    cache=root/'artifacts/flash_fr3/pickplace_74eps_stride4/manifest.json'
    shutil.copyfile(cache,package/'cache_reference_manifest.json')
    baseline=root/'artifacts/flash_fr3/phase_comparison_checks/20261104_01/baseline_http_15000.json'
    experiment={'format':'flash_fr3_phase_comparison_experiment_v2',
                'cache_manifest_sha256':digest(cache),'policy':source_config['policy'],
                'cache_identity_rule':'all reference fields except source_dataset must match, all six NPY SHA must match; original training manifest raw SHA must still match',
                'source_commit':source_config['source']['commit'],
                'fixed_split':'existing 66 train / 8 validation episodes; no repartition',
                'baseline_seed':42,'new_seed':43,'steps':15000,
                'training':'fresh seed43 using unchanged original trainer and inherited seed42 arguments; no resume',
                'selection':'six-metric Pareto comparison, no automatic service replacement',
                'robot_motion_commanded':False}
    (package/'experiment.json').write_text(json.dumps(experiment,ensure_ascii=False,indent=2)+'\n')
    shutil.copyfile(baseline,package/'reference_http_15000.json')
    readme='''# FLASH FR3：训练与检查点对照

此包只做离线评估和训练，无 ROS、机器人接口、仿真或推理服务切换。
不含示教缓存和模型权重，直接使用 server1cps 已有文件。

默认使用物理 GPU 2。启动前检查该 GPU 无计算任务，并用 UUID 设置 CUDA_VISIBLE_DEVICES；
不会停止任何已有任务，不修改 GPU 0/1/3 或已运行的 10132 服务。

## 在 H100 上运行

先激活已验证的 Python 3.11 / PyTorch 2.7.0 / NumPy 1.26.4 环境。脚本不安装或更新依赖。

```bash
conda activate flash_fr3
cd ~/flash_fr3
tar -xzf flash_fr3_phase_compare_20261104_v2.tar.gz
bash flash_fr3_phase_compare_20261104_v2/scripts/run_flash_fr3_phase_comparison.sh --mode all --gpu-index 2
```

`all` 顺序执行：

1. 核对固定缓存、策略配置、GPU、磁盘及环境；需至少 12 GiB 可用空间。
2. 评估 seed42 的 step_010000.pt 和 step_015000.pt，复核训练日志并生成阶段指标。
3. 执行 seed43 两次更新的训练 smoke 与 EMA 重载检查；失败时不启动正式训练。
4. 使用原训练脚本，从头训练 seed43 共 15000 步；继承原 batch、验证和保存间隔，固定原 66/8 划分。
5. 对新训练保存的所有 step 检查点及 best.pt 评估完整 483 窗口，更新对照表。

v2 不再把跨机器清单文件的字节摘要直接等同于数据身份。与包内参考清单比较时，
只忽略 source_dataset 绝对路径和 JSON 排版；其他字段全部相同，且六个 NPY 文件逐个核对 SHA。
当前清单仍须与服务器原训练配置中的文件摘要及完整 data 字段一致；不会修改缓存、
训练配置或检查点来让检查通过。实际差异写入 runs/cache_identity_*.json。
仅诊断缓存可使用 --mode check-cache，不需要 GPU，也不会启动评估或训练。

本轮不改变原 loss、历史处理、数据标签、网络、学习率日程或训练时长。
原训练器没有续训入口，且 15000 步末端学习率为零，因此不直接续接其优化器。

仅比较已有两份检查点，可将 `--mode all` 改成 `--mode baseline`。
输出总是新建在包内 runs/phase_compare_*，不覆盖已有训练目录；每 500 步输出剩余时间估计。
保持运行终端开启。训练时间受 GPU 和 IO 影响，原训练日志可作为估计，不能保证固定完成时刻。

## 文件位置

默认读取：

```text
~/flash_fr3/FLASH-Policy/
~/flash_fr3/flash_fr3_reproduction/data/pickplace_74eps_stride4/
~/flash_fr3/flash_fr3_reproduction/runs/train_15k_seed42_20261022_160102/
```

若路径不同，用 `--flash-root` 和 `--reproduction-root` 指定已有目录；不会自动下载或重建数据。

输出：

```text
runs/phase_compare_*/
  run_status.json
  comparison.md / comparison.csv / comparison.json
  evaluations/*/report.json / predictions.npz / phase_report.json
  smoke/smoke_result.json
  seed43_training/config.json / metrics.jsonl / step_*.pt / best.pt
```

对照指标：全局关节 MAE、当前锚点关节 P95、抓放附近夹爪 MAE、提前闭爪/漏闭爪/延迟释放错误率。
阈值 0.5 只用于离线开闭分类，不是新的机器人状态机。
Pareto 列只表示该组指标中未被其他候选全部压过，不会自动挑选、复制或部署模型。
验证集被重复用于选择，因此不是独立测试集；阶段计数也不代表真机任务成功率。

reference_http_15000.json 来源于已归档的 483 个真实模型 HTTP 回复，
只作指标回归基线；该归档没有独立核对远程检查点 SHA。
本包实际检查点评估会记录所读模型文件的 SHA，且核对缓存、配置和训练日志。

## 本地验证的范围

阶段统计已与归档的 483 个模型回复核对；单元测试包括忙 GPU 拒绝、指标与 Pareto、
进程失败，以及使用替身子进程结果的完整编排和 smoke 失败阻止训练。
这些不代替 H100 上的真实训练 smoke。本包尚未在服务器执行，不包含新训练结果。
'''
    (package/'README.md').write_text(readme)
    lines=[digest(path)+'  '+str(path.relative_to(package)) for path in sorted(package.rglob('*')) if path.is_file()]
    (package/'SHA256SUMS').write_text('\n'.join(lines)+'\n')
    with tarfile.open(archive,'w:gz') as tar:
        for path in sorted(package.rglob('*')):
            if path.is_file():tar.add(path,arcname=str(Path(package.name)/path.relative_to(package)),recursive=False)
    (destination/(archive.name+'.sha256')).write_text(digest(archive)+'  '+archive.name+'\n')
    print(json.dumps({'archive':str(archive),'bytes':archive.stat().st_size,'sha256':digest(archive)},ensure_ascii=False))


if __name__=='__main__':main()
