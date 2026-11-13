# FLASH策略仿真&真机（FR3)复现

基于 [FLASH: Efficient Visuomotor Policy via Sparse Sampling](https://arxiv.org/abs/2605.15492) 的真机视觉抓放项目：从VR示教、策略训练到双相机观测推理，在Franka FR3上执行连续关节力矩控制与异步夹爪动作。

## 真机演示

https://github.com/user-attachments/assets/ec8382e7-575b-4d74-b81f-02ce42ee092e

## 真机结果

| 指标 | 结果 |
| --- | --- |
| 自主抓放成功率 | **40/50（80%）** |
| 七关节采样跟踪MAE | **0.6002 ± 0.0361°** |
| 服务端预测与解码 | **均值23.434 ms，P95 26.402 ms** |

成功率采用现场最终50次尝试计数；误差与时延来自已提供报告中的25条执行记录、840个预测请求。跟踪误差基于约25 Hz及事件触发采样，服务端时延不含相机采集和完整网络往返。完整指标与定义见[真机结果](results/fr3/2026-11-13/README.md)。

## 系统流程

```mermaid
flowchart LR
    O[双相机与关节/夹爪状态] --> H[H100 · FLASH推理]
    H --> P[Legendre轨迹系数]
    P --> C[FR3本地连续力矩控制]
    C --> R[机械臂与异步夹爪]
    R --> O
```

H100根据真实观测历史生成多项式轨迹；机器人端在本地求取位置与解析速度，持续接收新预测并执行。已验证实现使用原始多项式、解析速度前馈、连续力矩更新和独立夹爪接口。

## 获取与使用

```bash
git clone --recurse-submodules \
  https://github.com/jason-0712/FLASH_Efficient-Visuomotor-Policy-via-Sparse-Sampling.git
cd FLASH_Efficient-Visuomotor-Policy-via-Sparse-Sampling
```

[训练与部署说明](docs/DEPLOYMENT.md)集中介绍环境、数据准备、训练、H100服务和FR3启动流程。完成驱动、双相机、任务配置与推理服务准备后，正式任务入口为：

```bash
# 执行真机抓放
bash scripts/run_flash_fr3_pickplace.sh --execute --server http://192.168.1.113:10132

# 返回本机保存的任务起点，成功后开爪
bash scripts/run_flash_fr3_return.sh --execute \
  --pose artifacts/flash_fr3/home_poses/task_start.json
```

## 仓库结构

| 路径 | 用途 |
| --- | --- |
| `scripts/` | 数据准备、训练、评估、推理服务与真机执行入口 |
| `src/` | FR3连续控制器及接口 |
| `configs/` | 已验证配置与本机配置示例 |
| [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) | 训练和真机部署步骤 |
| [results/fr3/](results/fr3/2026-11-13/README.md) | 真机成功率、跟踪误差和时延统计 |
| `third_party/FLASH-Policy` | 固定版本的作者官方策略子模块 |
| [simulation/](simulation/README.md) | 仿真补充验证 |

正式抓放使用`run_flash_fr3_pickplace.sh`，回位使用`run_flash_fr3_return.sh`。`pilot`、`model_prefix`、`retimed`等入口保留开发历史，部署步骤以当前说明为准。

## 仿真补充

LIBERO/MuJoCo单任务用于比较FLASH与FM-DiT：10,000步时，在50组未见物体位置上，原任务几何成功率分别为100%与88%。该结果与真机成功率独立统计，详见[简短报告、曲线及逐回合数据](simulation/README.md)。

上游实现、固定版本和依赖来源见[THIRD_PARTY.md](THIRD_PARTY.md)。示教数据、权重、现场图像及原始机器人日志保留在实验环境中。
