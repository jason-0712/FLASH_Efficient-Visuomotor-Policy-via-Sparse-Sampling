# FLASH 在 FR3 抓取放置数据上的复现

**最新结果（2026-11-13 13:57）：已完成一次完整自主抓放。**
现场操作者确认物体成功放置；归档报告为`TASK_SEQUENCE_FINISHED`，32次连续换段，
grasp和release均成功，结束后运动控制器inactive、命令接口全部释放。
使用原15k EMA与1倍物理时间下的原始多项式、解析速度、连续力矩控制和夹爪执行。
成功配置和源码快照见`artifacts/flash_fr3/successful_runs/20261113_135720/`。
后续保持本次版本、模型与参数，使用相同正式入口重复任务并统计成功率。
下方早期失败和暂停建议为历史记录，当前进度以本段及正式入口说明顶部为准。

**回位更新（2026-11-13）：** 已加速原回位入口，并在成功回位、机械臂控制器停用后自动张开夹爪至80毫米，
已在起点时也开爪。最近归档回位的相同起终点离线计算由32段/120秒降为8段/26秒，
这是名义运动时间，新真机总耗时待现场确认。未提高原生运动或停车限值；保存、只读检查均不动作。
现场确认回位路径无遮挡后，使用下方原回位命令即可；不需要新入口。

**上一步更新（2026-11-12 18:06）：** 后续17:52任务已实际闭爪，约5秒后空抓失败。
已发现正式配置仍使用试验的0.02米/秒，现仅恢复为采集适配器使用的0.1米/秒，
模型和机械臂原始连续轨迹保持原值。FR3重启后的ROS状态驱动已恢复，
三个夹爪接口就绪；完整任务效果待现场通过同一正式入口确认。

**最新现场结果（2026-11-12 17:01）：** 原15k模型已完成15次连续换段并输出闭爪。
一次grasp被驱动接受后返回失败，任务停止；实测夹爪仍约80毫米全开。
本次没有换段拒绝或力矩保护停车，完整抓放仍未完成。先处理这次夹爪动作失败；
不把移动方块、修改闭爪阈值或继续训练当作已证实的修复。
原始记录见`artifacts/flash_fr3/raw_policy_trials/flash_fr3_raw_policy_execute_20261112_170116_743514691/`。

**16:10现场更新：** 正式任务在第3次换段后触发Franka原生`cartesian_reflex`，驱动退出；完整抓放尚未完成。
当前先核对同一真实观测上的已有模型输出；不要直接重复下方历史命令。四份原始系数已经复算一致，
最新接入差约0.1441 rad。没有调整碰撞保护、增益或输入超时，详情见正式入口说明。

**当前入口（2026-11-12）：** 已安装原始多项式位置、解析速度前馈、持续力矩更新及异步夹爪执行。
`run_flash_fr3_pickplace.sh --execute` 与 `run_flash_fr3_raw_policy.sh --execute` 调用同一正式实现，
默认整任务预算120秒，完成抓取与释放后提前结束；不再默认三秒或禁用夹爪。
已移除正式任务中的0.05 rad参考接入门限及两处同值跟踪误差停止条件，误差继续记录。
15:50运行中的`torque_fault=5`来自误复用的微动10/4 Nm预算，正式入口现已解除该复用：
采用FR3厂家推荐幅值87/12 Nm与libfranka标准变化率999.999 Nm/s，原生保护保留；模型与增益不变。
新控制器已安装，只读驱动恢复，命令接口未占用；完整自主抓放尚待现场执行。
继续使用原15k EMA，不需要等待30k补充训练或先运行微动/保持试验。
当前检查分类、保留数值依据与本轮变更见 [正式入口说明](FLASH_FR3_正式入口检查说明.md)。

14:29运行后更新：实际已完成一次换段，随后候选被回调静默丢弃，客户端等待确认直到曲线到期。
现已修复保存文件消耗调度提前量及候选时间拒绝无反馈的问题；数值保护未放宽。
新版本已安装、只读驱动恢复，夹爪接口就绪，命令接口未占用；本轮没有运动。
回位耗时随当前姿态变化，随后直接执行下方同一完整任务入口。

```bash
cd /home/dase-hw101/franka_ws
bash scripts/run_flash_fr3_return.sh --execute \
  --pose artifacts/flash_fr3/home_poses/demo_episode036_start.json && \
bash scripts/run_flash_fr3_pickplace.sh --execute
```

H100保持已有10132推理服务运行。以下为数据、训练及部署过程的历史记录，旧阶段的微动/短段命令不属于当前正式任务的前置条件。

论文：[FLASH](https://arxiv.org/abs/2605.15492)。官方仓库：[NTUMARS/FLASH-Policy](https://github.com/NTUMARS/FLASH-Policy)，固定提交 `f47e490f2b820f835d1b9145da3cdb3df69f50a0`。本工具适配自己的场景和示教，不能把结果直接作为原论文任务成功率的复现。

## 已确认的数据和监督信号

- 原始目录：`/home/dase-hw101/franka_ws/dataset/snkdjn/quest3_franka_dualcam_pickplace_74eps`。
- LeRobot v2.1，74 条成功抓取放置示教，17,328 帧，15 Hz；外部相机和腕部相机均为 RGB 256×256。
- 已完整解码 148 个视频，检查每条视频的帧数、图像尺寸及相对 PTS 与标称时间列的对应关系。采集时未记录独立传感器时间戳，因此这项检查不能证明真实采集同步。
- 每 4 帧取一帧，得到 4,357 帧缓存；步长 4 是本次 FR3 实验的起点，可重新准备其他步长做对比。
- 按采集批次分层、固定 seed=42，整条 episode 划分为 66 条训练、8 条验证。训练窗口 3,874 个，验证窗口 483 个，归一化只拟合训练集。
- 模型观测为 7 个实测关节角与实测夹爪闭合比例。监督为示教中的实测关节轨迹与夹爪闭合命令；原始 6 维末端增量没有转换为关节命令。该数据没有保存原始教师关节目标，这一点属于复现适配。
- 原始夹爪观测是闭合比例，原始动作最后一维是张开比例；训练时采用 `command_closure=1-original_action[-1]`，统一为 0 张开、1 闭合。夹住物体时实测闭合比例可以小于 1。
- 模型的 `agent_pos` 与 `action` 维度均为 8，保留官方 ResNet18、GroupNorm、图像归一化和 FLASH 核心。

缓存为可携带的 NPY 内存映射文件，约 1.6 GiB；manifest 记录原始文件和缓存文件的 SHA256，训练前校验缓存，无需再访问原数据路径。

## 已完成：在 H100 上检查训练链路

将 `flash_fr3_reproduction.tar.gz` 放在服务器的 `~/flash_fr3_reproduction.tar.gz`。在已经激活 `(flash_fr3)` 的终端执行：

```bash
mkdir -p ~/flash_fr3
tar -xzf ~/flash_fr3_reproduction.tar.gz -C ~/flash_fr3
bash ~/flash_fr3/flash_fr3_reproduction/scripts/run_flash_fr3_h100_smoke.sh
```

工具包包含脚本、本文和这 74 条原始示教，首次运行自动构建上述稀疏缓存；无需访问 `/home/dase-hw101`，也无需另找电脑。默认使用已有源码 `~/flash_fr3/FLASH-Policy`、物理 GPU 1。可用 `FLASH_ROOT=/你的/FLASH-Policy FLASH_GPU=2 bash ...` 覆盖。只安装数据适配所需依赖，保留已有 CUDA PyTorch，不导入仿真器或机器人控制程序。

预期末尾出现 `FLASH_TRAINING_SMOKE_TEST=PASS`。它使用真实示教完成两次前向和反向、梯度有限性检查、实际参数变化检查、EMA 更新、检查点保存与重载一致性检查，并校验解析速度的物理时间尺度。500 步学习率预热的第一步学习率是 0，第二步会实际更新参数。smoke 模型仅验证链路，不代表抓取能力。

当前工作区已在隔离的 Python 3.11 / PyTorch 2.7 CPU 环境通过上述检查，H100 上也已完成两次更新和重载检查，结果为 `FLASH_TRAINING_SMOKE_TEST=PASS`。

## 正式训练命令（H100 检查通过后使用）

```bash
cd ~/flash_fr3/flash_fr3_reproduction
CUDA_VISIBLE_DEVICES=1 python scripts/train_flash_fr3.py \
  --flash-root ~/flash_fr3/FLASH-Policy \
  --dataset-cache data/pickplace_74eps_stride4 \
  --output runs/train_15k_seed42 \
  --device cuda:0 --batch-size 32 --steps 15000
```

使用单张 H100 即可起步。默认 FP32、AdamW(lr=1e-4, betas=(0.95,0.999), weight_decay=1e-6)、500 步预热、15,000 步余弦日程及 EMA(power=0.75)。每 250 步遍历验证集；每 1,250 步保存完整检查点，保留第 10,000 步和第 15,000 步以便比较。训练目录约需 12 GiB 以上空间。当前入口从头训练，尚无断点续训选项；短跑 `--steps` 仍使用同一 15k 学习率日程。

保存 `config.json`、`metrics.jsonl`、`step_*.pt` 与 `best.pt`。检查点含在线模型、EMA、优化器、学习率日程、随机状态、源码指纹、数据划分和时间尺度。`best.pt` 按验证关节 MAE 选择；这个离线误差不能替代真机任务成功率。

## 已完成：保存模型的离线评估

用户已在 H100 上完成正式训练，目录为 `/home/hanyu/flash_fr3/flash_fr3_reproduction/runs/train_15k_seed42_20261022_160102`。最佳验证出现在第 15,000 步，关节 MAE 为 `0.026743902182727125 rad`（约 `1.53231°`），夹爪闭合比例 MAE 为 `0.0844064274991768`。第 10,000 步为约 `1.56035°` 和 `0.08577365`。`best.pt`、`step_010000.pt`、`step_015000.pt` 均已确认存在，各约 706.6 MiB。

在服务器取得新增的 `scripts/eval_flash_fr3.py` 后执行：

```bash
cd ~/flash_fr3/flash_fr3_reproduction
CUDA_VISIBLE_DEVICES=1 python -u scripts/eval_flash_fr3.py \
  --flash-root ~/flash_fr3/FLASH-Policy \
  --dataset-cache data/pickplace_74eps_stride4 \
  --checkpoint runs/train_15k_seed42_20261022_160102/best.pt \
  --output "runs/train_15k_seed42_20261022_160102/offline_eval_$(date +%Y%m%d_%H%M%S)" \
  --device cuda:0 --batch-size 32
```

评估器只导入模型与数据代码，不连接相机、ROS 或机器人。它校验训练时的源码和缓存指纹，严格重载 EMA 权重与归一化参数，遍历 483 个验证窗口并核对训练日志中的同一步指标。另给出排除末端 padding 的误差、完整历史与动作窗口误差、逐 episode 和逐预测时间误差、当前曲线起点偏差，以及使用真实示教历史时相邻预测曲线在一个稀疏步和完整动作块后的位置与速度差。

轨迹诊断在当前时刻到下一动作块边界之间采样 161 点，统计关节位置、物理速度和加速度，以及原始夹爪预测范围。这是采样诊断，不能证明连续轨迹的精确极值；训练观测范围也不是硬件关节限位。解析导数另与数值差分核对。batch=1 推理延迟包含模型内部归一化、图像编码和轨迹生成，不含相机、外部图像预处理、网络与控制器。

输出 `report.json`、`per_episode.csv`、`per_horizon.csv`、`predictions.npz`，末尾打印 `FLASH_OFFLINE_EVAL=...`。评估采用真实示教历史而非预测状态反馈，因此不是闭环执行，也不直接给出真机通过资格。暂不启动机器人动作。

用户已返回服务器评估结果：`offline_eval_20261022_175327/report.json`，483 个验证窗口全部完成，EMA 重载与训练日志一致（关节 MAE 差约 `1.36e-10 rad`，夹爪 MAE 差约 `2.66e-9`）。关节速度与加速度的解析导数检查通过，最大数值差约 `2.40e-7 rad/s` 和 `1.02e-6 rad/s²`，重复推理差为 0。batch=1、输入已在 GPU 上的单次模型推理中位数约 `6.467 ms`，P95 约 `6.489 ms`。

执行适配仍需处理以下实际预测特征：曲线起点相对当前关节角的最大差为 `0.15827 rad`（约 `9.07°`）；在真实示教历史下对比两次预测，一个稀疏步后的最大位置差为 `0.07394 rad`（约 `4.24°`），完整动作块后的最大位置差为 `0.50114 rad`（约 `28.71°`）。这些是离线曲线之间的差异，不是已经发生的机器人跳动。夹爪连续预测范围为 `[-0.30791, 1.23948]`，执行层需限定到有效比例范围。161 点采样得到的最大关节速度约 `0.7074 rad/s`、最大加速度约 `2.2941 rad/s²`，不能据此断言实际执行满足全部约束。71 个窗口超出训练观测关节范围，该统计不代表触及 FR3 硬件关节限位。

## 当前下一步：FR3 只读接口核对

已直接检查机器人工作区所在 ThinkPad 的 Docker：实际 `franka` 容器为 Ubuntu 22.04 / ROS 2 Humble，工作区为 `/home/ros/ros2_ws`，源码绑定到宿主机本工作区的 `src`，ROS_DOMAIN_ID 在交互 shell 中设为 30。宿主机的 Jazzy 与容器内的 Humble 需分开使用。容器此前退出码为 255，日志没有明确记录停止原因；已确认入口只加载 ROS 并启动 Bash 后，将原容器启动，验证 Humble CLI 与已安装的 `crisp_controllers` 可用，未启动机械臂控制器。H100 训练和评估仍位于 server1cps 的既有目录。

工作区源码的 `CartesianController::parse_target_joint_` 接收 `sensor_msgs/msg/JointState` 的 position 和 velocity，按数组顺序赋给 q_ref 和 dq_ref；当前实现没有按 msg.name 重排。因此必须先核对实际运行控制器的关节顺序。`crisp_py.robot.Robot.set_target_joint` 的现有发布路径默认发送零速度，不能直接用于保留 FLASH 解析速度的执行适配。当前只读源码检查不代表服务器已部署的二进制版本一定相同。

下一步读取运行环境、控制器状态、硬件接口、关节状态、命令话题端点和双相机话题，继续保持不发布机械臂或夹爪命令。后续执行层需设计曲线起点对齐、预测切换时的位置/速度连续过渡、短时滚动更新以及状态和命令超时处理；这些属于 FR3 适配，需与模型本身的离线指标分别记录。

### 已完成：只读状态入口与真实观测采集

`scripts/flash_fr3_observe.launch.py` 已在 `franka` 容器内启动，仅激活 `joint_state_broadcaster`，所有机械臂命令接口均未被占用。官方夹爪驱动仅读取状态，未启动会在构造时自动打开夹爪的 CRISP hand adapter。机械臂状态话题为 `/franka/joint_states`，夹爪为 `/franka_gripper/joint_states`，不依赖汇总的 `/joint_states`。不需要再启动完整的 VR 或运动控制 launch。

首次只读运行在成功读取之后发生过机械臂与夹爪同时 UDP 接收超时，日志保存在容器 `/tmp/flash_fr3_observe_timeout_20261022.log`。网络及单次状态读取恢复后，已重新启动同一个只读入口，日志为 `/tmp/flash_fr3_observe.log`。不要并行启动第二个控制器管理器。主机当前无实时内核；这不阻碍本次只读输入验证，真机运动前仍需落实底层实时运行条件。

实测机械臂 `JointState.name` 顺序为 `[1,3,6,7,2,4,5]`，输入必须按名称重排至 `[1,2,3,4,5,6,7]`，不能直接使用 `position[:7]`。实测夹爪闭合比例按 `1-(finger_joint1+finger_joint2)/0.08` 计算。

`scripts/capture_flash_fr3_observation.py` 只创建订阅，不发送机械臂或夹爪命令。相机使用与示教采集 CRISP Camera 相同的压缩图像传输、cv_bridge RGB8 解码、保持比例的 INTER_AREA 缩放与中心裁剪至 256×256。已用原 Camera 方法对比验证预处理逐像素一致，并验证乱序关节的重排与缺失关节拒绝行为。为减少大图传输丢帧，采集端使用容器已有的 `rmw_cyclonedds_cpp`，不需要安装软件或更改控制器进程。

采集器先缓存源消息，再按 `4/15 s` 网格选择最近的源时间戳，记录每路真实时间偏差和到达时延，拒绝过期、重复或无法覆盖历史网格的输入。该方法不构成硬件同步。已采集 8 个真实观测时刻，源时间相对目标网格的最大绝对偏差约 `0.04768 s`，最大到达时延约 `0.03365 s`。本地目录：`artifacts/flash_fr3/live_observations/flash_fr3_live_observation_20261022_01`；其中包含 `observation.npz`、元数据与两张最终模型输入预览。

### 已完成：H100 上的单组真实观测推理

把新增推理脚本与该观测目录传至服务器既有工具包目录后，在已激活 `(flash_fr3)` 的终端执行：

```bash
cd ~/flash_fr3/flash_fr3_reproduction
bash scripts/run_flash_fr3_real_observation.sh
```

运行脚本默认只使用物理 GPU 1，严格加载已有 `best.pt` 的 EMA 参数及训练归一化参数，并校验固定源码、缓存和时间尺度。`infer_flash_fr3_observation.py` 不导入 ROS 或机器人驱动，不连接机器人。输出 `runs/real_observation_*/report.json` 和 `prediction.npz`，末尾为 `FLASH_REAL_OBSERVATION_INFERENCE=...`，其中列出预测形状、相对当前实测关节角的起点差、夹爪原始预测范围及超出训练观测范围的状态维度。训练范围不是硬件限位；本步骤只有单组真实历史输入，没有闭环任务成功率。

用户已返回 `real_observation_20261022_183624_1989249/report.json` 的运行结果：第 15,000 步 EMA 权重完成推理，预测形状为 `[8,8]`，未发送机器人动作。起点相对实测关节角的最大偏差约 `9.13056°`，夹爪原始预测范围 `[-0.0325671,0.00234252]`，状态索引 `[4]`（第 5 关节）超出训练观测范围。进一步对比本地训练数据，当前第 5 关节相对训练上界高约 `1.22852°`；该差异不能单独解释整个预测起点偏差。用户随后调整了盒子和方块的位置以接近示教布置，后续影子推理会重新读取当前画面。

### 已完成：连续影子推理

服务端为 `scripts/serve_flash_fr3_shadow.py`，复用已验证的严格 EMA 加载路径，不导入 ROS 或机器人驱动。默认在 H100 的 `192.168.1.113:10131` 提供 HTTP 观测推理，仅接受 ThinkPad `192.168.1.107` 和本地回环地址的请求。取得新增脚本后，在 `(flash_fr3)` 终端执行：

```bash
cd ~/flash_fr3/flash_fr3_reproduction
bash scripts/run_flash_fr3_shadow_server.sh
```

默认继续使用物理 GPU 1。出现 `FLASH_SHADOW_SERVER_READY=...` 后保持终端运行，服务等待真实观测请求；它本身不会连接机器人。

机器人端 `scripts/run_flash_fr3_shadow_client.py` 只创建相机、关节和夹爪订阅，经容器已有的 Cyclone DDS 读取消息，在固定 `4/15 s` 网格上构建滚动历史，将 NPZ 输入发给 H100。相邻请求复用已解码的历史图像。它没有机械臂或夹爪命令发布路径，不需要激活运动控制器。默认进行 40 次尝试，检查源消息时效、传感器时间差、请求编号、模型步数和返回曲线与动作数组是否一致。

输出 `predictions.jsonl`、首尾观测 NPZ 和 `report.json`，末尾打印 `FLASH_SHADOW_DIAGNOSTIC=...`。报告包括成功与跳过次数及原因、观测准备时间、请求往返时间、从当前目标观测时刻到收到响应的时间、服务端推理与解码时间，以及相邻预测曲线在同一物理时刻的位置和速度差。上述曲线差是假设切换时的诊断，不是已经执行的机器人跳动；完成诊断也不表示通过真机执行资格。

本地已通过 `test_flash_fr3_shadow.py`：使用 CPU 上的未训练官方模型核对物理轨迹及解析导数、HTTP NPZ 往返、无效状态和请求编号不一致的拒绝行为。另使用真实 ROS 观测在本地完成 6/6 次客户端到服务端联调，未发送动作；它使用的是未训练 CPU 模型，不能将该测试的预测误差或延迟当成 H100 训练策略的结果。本地报告保存在 `artifacts/flash_fr3/shadow_checks/local_untrained_cpu_transport_20261023.json`。固定网格的 8 次真实观测检查也通过，相邻请求复用 14/16 张历史图像，观测准备时间中位数约 `44.28 ms`、最大约 `53.16 ms`。

2026-10-23 已确认昨天的只读进程再次因 UDP 超时退出；单次读取恢复后（Idle，无机器人错误），已重新启动同一个只读入口。重新采集的滚动观测通过。随后使用 H100 上正式第 15,000 步 EMA 模型完成 **40/40 次真实跨电脑影子推理**，跳过 0 次，未发送机械臂或夹爪动作。完整记录已从容器保存至宿主机 `artifacts/flash_fr3/shadow_runs/flash_fr3_shadow_20261023_01`。

| 实测项目 | 中位数 | P95 | 最大值 |
| --- | ---: | ---: | ---: |
| 当前目标观测时刻至收到预测 | 364.44 ms | 576.26 ms | 624.95 ms |
| 推理请求往返 | 128.59 ms | 193.51 ms | 238.52 ms |
| 服务端推理与结果解码 | 23.38 ms | 26.53 ms | 27.53 ms |
| 预测起点相对观测姿态的最大关节差 | 6.62° | 7.01° | 7.02° |
| 相邻原始曲线在同一时刻的最大位置差 | 2.60° | 3.41° | 6.23° |

观测至响应时间包含固定网格回看、观测准备和网络传输，不等于 GPU 模型前向时间。起点差主要集中在第 2、6 关节，逐关节诊断为同目录 `per_joint_diagnostics.json`。曲线位置差是未执行的假设切换差，不能解释为机械臂实际跳动。该次观测记录中机械臂保持静止，不是闭环任务试验。

### 已完成：延迟补偿与连续参考的离线验证

新增 `scripts/flash_fr3_reference.py` 与 `scripts/replay_flash_fr3_reference.py`，只使用 NumPy，不导入 ROS，不连接机器人或服务器。它们按每条实际响应的时龄选择物理曲线位置，并在切换时添加五次 Hermite 校正，使新参考的 q、dq、ddq 与上一条参考一致，校正在指定过渡时间后消失。该适配属于本实验的执行层研究，不是论文原有算法，也没有修改模型或重新训练。夹爪不进入此七关节校正。

已完成如下本地验证：

```bash
/tmp/flash-fr3-verify311/bin/python scripts/test_flash_fr3_reference.py -v
/tmp/flash-fr3-verify311/bin/python scripts/replay_flash_fr3_reference.py \
  --shadow-run artifacts/flash_fr3/shadow_runs/flash_fr3_shadow_20261023_01 \
  --output artifacts/flash_fr3/reference_replays/flash_fr3_shadow_20261023_01_c2
```

四项测试通过，覆盖起点和过渡终点的 q/dq/ddq、重规划边界、物理时间导数及过期/非有限输入拒绝。回放使用 `blend=1.2 s`、最大接受时龄 `0.7 s`、采样间隔 `0.001 s`，这是离线实验参数，并非已验证的真机控制参数。40 条真实预测全部覆盖，记录 11,826 个采样点（约 11.77 s，含最后 1.2 s 的参考尾段），切换处 q/dq/ddq 差不超过 `2.23e-16` 的数值量级。七关节采样速度峰值约 `0.53851 rad/s`、加速度峰值约 `1.55067 rad/s²`、jerk 峰值约 `12.5062 rad/s³`。输出为上述目录的 `report.json` 和 `reference.csv`，原始 JSONL 的 SHA256 写入报告。

初始 q 由第一条已记录输入重建，初始 dq、ddq 假设为零，没有当时实测速度记录。后续切换使用上一条假设参考状态；原始录制观测不会随参考变化，因此该回放没有机器人动力学或状态反馈，不能用于评定任务成功。C2 连续也不代表满足所有限制，jerk 仍可能不连续，采样峰值不是连续时间极值证书。超时或超出预测区间会抛出异常，但这尚不是控制器的自动刹停机制。

### 下一步：机器人电脑的实时执行环境

2026-10-23 只读检查确认，ThinkPad 宿主机当前内核为 `6.17.0-1032-oem`，配置是 `CONFIG_PREEMPT_VOLUNTARY=y`，未启用 `CONFIG_PREEMPT_RT`，`/sys/kernel/realtime` 不存在。`franka` 容器已有 `CAP_SYS_NICE`、rtprio 上限 99 及内存锁定额度；这些权限不能替代宿主机实时内核，也不能据此保证 FCI 实时循环。

Franka 官方要求 libfranka 控制程序在 PREEMPT_RT 内核上以实时优先级运行，见[实时内核配置](https://frankarobotics.github.io/docs/doc/libfranka/docs/real_time_kernel.html)。Ubuntu 24.04 的官方安装途径见[Ubuntu Pro 实时内核说明](https://ubuntu.com/pro-client/docs/en/docs/howtoguides/enable_realtime_kernel/)。内核需安装并重启后验证；上述只读检查阶段没有安装内核、重启电脑或激活运动控制器。后续还需实现运行时限幅检查、状态/命令超时处理和夹爪独立适配，之后才能逐步接入真机控制。

用户已选择 Ubuntu Pro 安装方式。再次检查确认，ThinkPad 的 Pro 客户端为 `37.2ubuntu~24.04.1`、尚未绑定账户；Secure Boot 关闭，根分区可用空间约 9.7 GiB。当前工具不能无密码 sudo，账户绑定也需要用户在自己的浏览器完成。

已准备本机交互安装脚本，在 **ThinkPad 宿主机终端**运行：

```bash
bash /home/dase-hw101/franka_ws/scripts/setup_flash_fr3_realtime.sh
```

脚本先确认主机和发行版，提示本机 sudo 认证；若尚未绑定，则执行 `sudo pro attach --no-auto-enable`，由用户按终端给出的链接在浏览器绑定账户。随后用 `pro enable realtime-kernel --access-only` 启用软件源，检查依赖并安装 `linux-realtime-hwe-24.04`。该 HWE 元软件包会随软件源更新，内核版本以本机安装结果为准；本机实际获得的是下述 `7.0.0-38-realtime`，并非前期官方页面描述的 6.17。实际硬件与 FCI 表现仍需重启后验证。安装使用 `--no-remove` 拒绝移除现有软件包，保留已有内核。

安装后会核对 `/boot` 中的 PREEMPT_RT 配置、内核和 initramfs，保存 `artifacts/flash_fr3/runtime_checks/thinkpad_realtime_install_*.json`，输出 `FLASH_REALTIME_INSTALL=COMPLETE_REBOOT_PENDING`。脚本不自动重启。先返回安装结果，再指导重启、在 GRUB 选择实时内核，并核对 `/sys/kernel/realtime` 为 `1`；不能将“软件包已安装”等同于“实时内核正在运行”。

用户已升级 Pro 客户端至 `38ubuntu0~24.04`，重新生成绑定码并完成安装。安装记录为 `artifacts/flash_fr3/runtime_checks/thinkpad_realtime_install_20261023_132140.json`。已直接核对：`linux-image-7.0.0-38-realtime` 与 HWE 元软件包均为已安装，`/boot/vmlinuz-7.0.0-38-realtime`、对应 initramfs 和配置文件齐全，配置含 `CONFIG_PREEMPT_RT=y` 与 `CONFIG_HZ=1000`。安装完成时仍在运行 `6.17.0-1032-oem`，随后已按下述步骤重启验证。

GRUB 当前配置为 `GRUB_TIMEOUT_STYLE=hidden`、`GRUB_TIMEOUT=0`，且 OEM 配置设置了 `GRUB_FLAVOUR_ORDER=oem`。因此先在 ThinkPad 宿主机执行以下命令显示 10 秒菜单，再手动选择实时内核；当前步骤不修改默认内核选择：

```bash
printf '%s\n' 'GRUB_TIMEOUT_STYLE=menu' 'GRUB_TIMEOUT=10' | \
  sudo tee /etc/default/grub.d/99-flash-fr3-menu.cfg
sudo update-grub
```

菜单更新成功、保存当前工作后，停止只读机器人容器并重启 ThinkPad：

```bash
docker stop franka
sudo reboot
```

在启动菜单进入 `Advanced options for Ubuntu`，选择名称含 `7.0.0-38-realtime` 的正常启动项。登录后执行：

```bash
uname -r
cat /sys/kernel/realtime
```

预期依次为 `7.0.0-38-realtime` 和 `1`。如果新内核无法正常进入系统，可在同一菜单选择保留的 `6.17.0-1032-oem` 返回原系统。`franka` 容器的重启策略为 `no`，所以后续需按只读流程恢复，不能把重启后的空话题误当作模型故障。安装与启动验证不等于机器人运动执行验证。

### 已完成：实时内核启动与只读恢复

用户已返回上述两项预期输出。本地再次只读确认，宿主机和 `franka` 容器均使用 `7.0.0-38-realtime`，实时标志为 `1`。重启后机器人有线地址仍为 `172.16.0.1`，Wi-Fi 仍为 `192.168.1.107`；到 FR3 的三次 ping 无丢包。两台相机的容器自动恢复。

已启动原 `franka` 容器。独立进程实际验证 FIFO 优先级 50 与 `mlockall(MCL_CURRENT|MCL_FUTURE)` 均成功；在尚无控制器管理器运行时，单次 FR3 读取为 Idle、当前和上次运动错误列表均为空。随后恢复同一个只读 launch，仅激活 `joint_state_broadcaster`。日志确认管理器配置为 1000 Hz，FIFO 实时线程优先级 50 设置成功；21 个关节命令接口全部 unclaimed。配置频率和调度权限检查不能替代实际运动控制周期的截止时间测试。

新采集的 8 个双相机/机械臂/夹爪观测通过，保存在 `artifacts/flash_fr3/live_observations/flash_fr3_live_observation_rt_20261023_01`。源时间相对历史网格最大偏差约 `34.81 ms`，到达时延最大约 `31.12 ms`，四路源时间最大间隔约 `39.33 ms`。汇总报告为 `artifacts/flash_fr3/runtime_checks/thinkpad_realtime_boot_20261023.json`，同目录保存新启动日志、控制器和硬件接口列表。上述操作未发送机器人或夹爪动作。

恢复时 H100 的 `10131` 端口拒绝连接。机器人端已启动 10 次只读重连检查，在容器 `/tmp/flash_fr3_shadow_rt_20261023_01.log` 等待服务（最长 900 s）。若服务器终端显示 `(base)` 并报 `ModuleNotFoundError: numpy`，先切换回已完成训练与环境验证的环境，再启动服务：

```bash
conda activate flash_fr3
cd ~/flash_fr3/flash_fr3_reproduction
bash scripts/run_flash_fr3_shadow_server.sh
```

此错误首先按环境选择处理，无需向 base 环境安装另一套训练依赖。H100 服务启动后会自动接收机器人端只读请求。重连结果另行保存，不能将旧的 40 次记录当成本次重启后的验证结果。

用户切回 `(flash_fr3)` 后，服务已恢复，健康接口确认第 15,000 步 EMA 权重。重启后的 **10/10 次影子推理全部成功、跳过 0 次**，结果已保存至宿主机 `artifacts/flash_fr3/shadow_runs/flash_fr3_shadow_rt_20261023_01`。观测至响应中位数约 `276.03 ms`、P95 `568.69 ms`、最大 `649.06 ms`；服务端推理与结果解码中位数约 `23.27 ms`。预测起点最大关节差约 `7.01165°`，相邻原始曲线的假设切换位置差最大约 `5.59963°`。这组短测试证明了新内核下的输入和推理链路，不能将与旧测试的延迟差归因于内核，也没有真机任务成功率。

同批 10 条响应已用既有 C2 参考适配器完成纯数值回放，边界连续性检查通过；输出为 `artifacts/flash_fr3/reference_replays/flash_fr3_shadow_rt_20261023_01_c2`。回放使用容器已有的 NumPy，只计算数值参考，不导入 ROS、不连接机器人，不是动力学仿真或闭环试验。

已只读核对容器安装目录中的 FR3 控制器配置，副本为 `artifacts/flash_fr3/runtime_checks/installed_fr3_controllers_20261023.yaml`。`joint_impedance_controller` 与 `cartesian_impedance_controller` 共享 `crisp_controllers/CartesianController` 实现，但前者的六维 task stiffness 全为 0，且 `nullspace.projector_type=none`，按源码的力矩计算路径可用于七关节参考；不能把普通笛卡尔配置下的 `/target_joint` 直接当作全关节轨迹接口。当前该关节控制器没有被加载或激活，仍只有状态广播器 active。后续执行适配需明确选择此关节模式、保持按名称确定的关节顺序，并补齐运行时约束和超时处理。

### 已完成：运行时保护层的真实输入检查

新增 `scripts/flash_fr3_guard.py`、`scripts/run_flash_fr3_guard_shadow.py` 与宿主机入口
`scripts/run_flash_fr3_guard_shadow.sh`。它们继续只创建 ROS 订阅，未加载或激活运动控制器，未创建关节命令发布器或夹爪动作客户端。H100 HTTP 推理在独立线程中运行，主线程以名义 100 Hz 检查状态与参考；这不是机器人驱动的 1000 Hz 实时控制循环。

当前首次小范围试验的**候选阈值**为：每关节相对初始化姿态最多 `0.10 rad`、速度 `0.25 rad/s`、加速度 `1 rad/s²`、jerk `20 rad/s³`。位置范围采用 [Franka 官方建议的 FR3 矩形范围](https://frankarobotics.github.io/docs/robot_specifications.html)，不是把训练观测范围作为硬件限位。上述应用阈值尚未通过真机运动验证。

保护层读取按名称重排的实测 q/dq，仅在 `|dq|≤0.01 rad/s` 的近似静止状态下将初始加速度假设为零；随后使用实际预测时龄与既有 C2 适配器构造候选参考。对整条剩余候选轨迹按 1 ms 网格检查 q/dq/ddq/jerk，含过渡终点和轨迹终点。任何超限都会拒绝整条候选轨迹，不会独立裁剪 q 与 dq。采样不是连续极值、碰撞、力矩或动态稳定性证明。

运行时检查还包括：关节消息源时龄与接收后等待时间合计不超过 `0.10 s`、接受预测时龄不超过 `0.70 s`、接受预测间隔不超过 `0.70 s`、检查线程运行间隔不超过 `0.10 s`、跟踪偏差不超过 `0.035 rad`，以及预测编号与观测时间严格递增。故障进入锁存 `BLOCKED`，后续有效预测也不会自动恢复。夹爪仅记录 `[0,1]` closure 及对应宽度映射，不发送动作。Python 层无法在本进程停顿或退出时继续执行保护，**控制器层的独立看门狗与停止机制尚未实现**。

九项 NumPy 测试已通过：关节名称顺序与缺失速度、整轨迹拒绝且禁止自动恢复、推理线程阻塞时独立超时、过期源消息、迟到检查循环、乱序/过期预测、实测跟踪误差、近似静止初始化条件、响应时间 NaN 与物理时间尺度/动作数组不一致拒绝，以及向量化采样与标量解析导数的一致性。既有 C2 参考的四项测试也通过。增强后的响应校验已对此前 50 条 H100 真实响应完成回归检查，无需再次推理。

2026-10-23 完成两次真实输入检查。首次结果为
`artifacts/flash_fr3/guard_shadow_runs/flash_fr3_guard_shadow_rt_20261023_01`；
补充记录准入时刻的完整实测 q/dq 与单调时钟后，再次检查保存为
`artifacts/flash_fr3/guard_shadow_runs/flash_fr3_guard_shadow_rt_20261023_02`。
两次均在首条候选处锁存 `SHADOW_GUARD_BLOCKED / TRAJECTORY_LIMIT`，接受轨迹 0 条，发送机械臂/夹爪动作 0 次。

最新一条观测到响应约 `354.01 ms`，候选在准入时剩余约 `1.777 s`。第 2 关节采样最大位移 `0.69213 rad`、速度 `0.64273 rad/s`、加速度 `1.48938 rad/s²`，均超过首次小范围试验的阈值。首次超出局部位移、速度和加速度阈值的候选时刻分别约 `0.449 s`、`0.254 s`、`0.110 s`。这些都是未发送的参考轨迹数值，实测机械臂继续保持静止。该结果说明完整抓放候选不能直接用于这次小范围试运行，不能记为运动执行通过，也不要求据此重新训练。

若后续修改执行适配或场景后需要重新检查，在 **ThinkPad 宿主机**运行下列入口；本轮已完成检查，无需立刻重复：

```bash
bash /home/dase-hw101/franka_ws/scripts/run_flash_fr3_guard_shadow.sh
```

入口要求既有只读状态读取、双相机和 H100 服务保持运行，自动将所需脚本复制进 Humble 容器，并把诊断目录保存回宿主机。它不会替用户启动运动控制器。后续应先设计有明确位移和时间边界的首次低速轨迹，补齐控制器侧独立看门狗，再进入真机小动作验证；不能通过直接放宽当前阈值来把本次拒绝改记为通过。

### 已完成：首次小动作的 C++ 控制器与未激活检查

新增独立 ROS 包 `src/flash_fr3_pilot_controller`，未修改既有 VR、CRISP 控制器或原厂硬件驱动源码。首轮改用 FR3 的七关节 `position` 命令接口：原厂硬件驱动通过 `startJointPositionControl(kJointImpedance)` 使用机械臂内置关节阻抗控制器。控制器不接收 FLASH 完整动作块，只执行一次固定的执行链路检查：第 2 关节参考相对起点增加 `0.005 rad`（约 `0.28648°`），3 秒出去、3 秒返回，其余关节参考固定，夹爪不发命令。此阶段不是抓放策略成功率试验。

轨迹采用五次多项式，起点、折返点、终点的参考速度和加速度均为零；使用机器人时间推进轨迹，使用本机单调时钟检查心跳和调度。实际安装的 libfranka 0.19 提供 `q_d/dq_d/ddq_d`；首个控制周期从原生 `q_d` 初始化，检查原生期望速度/加速度近似静止，并要求与实测 q 及保存的起点足够接近。没有将 FLASH 的原始预测起点写入机器人。

计划目录为 `artifacts/flash_fr3/pilot_plans/flash_fr3_pilot_plan_20261023_01`，包含只读采集的 50 条关节状态、`plan.json`、`controller_params.yaml` 和 `reference.csv`。编译后的同一参考核心生成了 6,501 个数值采样点，最大参考位移 `0.005 rad`、速度约 `0.003125 rad/s`、加速度约 `0.00320750 rad/s²`、jerk 约 `0.0111111 rad/s³`。未运行动力学仿真，也未据此宣称真实跟踪或碰撞检查通过。

C++ 控制器内部的心跳检查独立于 Python 推理进程。仅接受计划 ID 匹配、激活后产生、严格递增且源时龄不超过 `0.10 s` 的心跳；超过 `0.20 s` 未收到有效心跳，便从当前参考 q/dq/ddq 开始，用 `0.25 s` 的五次参考结束运动，再锁存保持。恢复心跳不会继续运动。启动后 2 秒没有心跳则不启动轨迹。同一实例只允许一次激活；再次试验需明确卸载、重新准备并加载。

代码还检查机器人时钟不前进/倒退、机器人时间跨度超过 `5 ms`、本机检查跨度超过 `10 ms`、非有限状态、实测速度、跟踪误差和局部位置范围。数值检查通过不代表实际 1 kHz 运动周期已验证。

已核对 [ros2_control 2.52.0 源码](https://github.com/ros-controls/ros2_control/blob/2.52.0/controller_manager/src/controller_manager.cpp)：该 Humble 版本的 `update(ERROR)` 只记录错误，并不自动停用控制器。因此本包另有 C++ 非实时定时器，在心跳停止后的参考结束或硬错误发生时，向管理器明确请求原生停用；此路径不依赖 Python 客户端。硬错误时的原生停用依赖管理器服务可处理，整个控制进程或 FCI 失效时仍需由机器人自身机制处理，实际停止效果尚未真机验证。原厂驱动在启动位置控制遇到 `ControlException` 时自带一次错误恢复/重试路径，本实验没有修改该原厂行为；本包不重启轨迹。

已通过 **9 个 C++ 测试用例**：8 个数值/状态机用例加 1 个控制器与停用服务接口用例。心跳中断覆盖了运动中 120 个停止时刻，核对参考 q/dq/ddq 连续、限值和禁止自动恢复；接口用例在独立 `ROS_DOMAIN_ID=231` 中验证发生错误后确实请求停用，未运行硬件驱动或机器人动力学。CTest 汇总显示 11 tests，是 9 个 GTest 用例加 2 个 CTest 包装项，不是 11 个独立行为测试。

为了让管理器识别新增安装包，已正常停止并恢复既有只读 launch，再通过官方 `spawner --inactive` 加载试运行控制器。最终状态为：`flash_fr3_pilot inactive`、`joint_state_broadcaster active`，21 个关节命令接口全部 unclaimed，未发送机器人/夹爪动作。最终只读启动日志为容器 `/tmp/flash_fr3_observe_pilot_final_20261023.log`，仍为 1000 Hz 配置和 FIFO 50。控制器与原厂硬件解析到同一个工作区 libfranka 0.19。

宿主机入口为：

```bash
# 默认只读；本轮已经完成，无需重复执行。
bash /home/dase-hw101/franka_ws/scripts/run_flash_fr3_pilot.sh --check
```

最新输出 `READONLY_PREFLIGHT_PASS`，记录目录为 `artifacts/flash_fr3/pilot_trials/flash_fr3_pilot_check_20261023_143951_659526447`，最大起点差约 `4.43e-6 rad`。检查包括控制器类型与 inactive 状态、无其他运动控制器、加载参数与计划 ID 一致、实测状态新鲜且近似静止、起点差不超过 `0.002 rad`。最初使用 Cyclone DDS 调用 Fast DDS 管理器服务出现 payload/history 不兼容超时；试运行入口现已使用与管理器一致的 `rmw_fastrtps_cpp`，检查通过。双相机影子推理的既有客户端无需更换 DDS。

下一步由人在 FR3 旁执行首次小动作。先确认机械臂有足够活动空间、人员处于工作范围外且急停可触及，见 [Franka 运行示例前的要求](https://frankarobotics.github.io/docs/doc/libfranka/docs/overview.html)。在 **ThinkPad 宿主机交互终端**执行下列命令；它会激活真实控制器并让机械臂运动，本轮代理没有运行该命令：

```bash
bash /home/dase-hw101/franka_ws/scripts/run_flash_fr3_pilot.sh --execute
```

客户端先重复只读预检查，明确激活后才发送 50 Hz 心跳；正常约 6 秒完成往返。结束后停止心跳并请求原生停用，保存控制器状态与实测关节轨迹。正常结果为 `FLASH_PILOT_RESULT` 中 `status=MOTION_TRIAL_FINISHED` 和 `controller_left_inactive=true`；仍需检查实测位移/返回误差，不能把流程结束直接当成 FLASH 抓放复现成功。使用交互 PTY 把 Ctrl-C 传入容器客户端；发生异常时停止心跳并尝试停用，不自动恢复或再次激活。若起点变化、控制器状态异常或执行失败，先保留完整输出，不直接重复 `--execute`。

### 2026-10-23：修复试运行客户端的 Python QoS 写法

首次人工执行在创建心跳发布器时报告 `'QoSProfile' object has no attribute 'best_effort'`。
错误源于把 C++ 的链式 QoS 写法用于 rclpy；已改为
`QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)`。
失败记录为 `artifacts/flash_fr3/pilot_trials/flash_fr3_pilot_execute_20261023_144954_920559922`，
其中 `activation_requested=false`、`robot_motion_commanded=false`，异常发生在控制器激活请求之前。
本次没有消耗控制器实例的一次激活额度，无需为此重载 C++ 插件。

新增 `scripts/test_flash_fr3_pilot_client.py`，在强制隔离的 `ROS_DOMAIN_ID=232`、
`ROS_LOCALHOST_ONLY=1` 中使用 ROS 消息与服务替身，运行真实客户端子进程。
4 项检查均通过：默认只读时不激活且不发心跳；计划不匹配时即使指定执行也不激活；
执行分支成功发送带计划 ID 和递增系统时间戳的心跳并完成停用；控制器报告错误时停止心跳并停用。
接口替身不加载硬件驱动，不模拟机械臂动力学，其测试完成状态不是物理运动通过。
测试记录为 `artifacts/flash_fr3/pilot_client_checks/flash_fr3_pilot_client_qos_20261023_145600_996320798`。

修复后真实域 30 的 `--check` 再次通过，最大起点差 `3.4421682357788086e-6 rad`，
记录为 `artifacts/flash_fr3/pilot_trials/flash_fr3_pilot_check_20261023_145625_015729172`。
额外只读查询确认 `flash_fr3_pilot inactive`、仅状态广播器 active，21 个命令接口全部未占用。
本次修复没有自动执行真机动作；后续仍由人在 ThinkPad 交互终端使用上述 `--execute` 命令。

### 2026-10-28：恢复状态读取并等待首次小动作实测

继续工作时仅双相机节点仍在运行，机器人管理器不在线；旧只读启动日志显示
机械臂和夹爪驱动均因 UDP 接收超时退出。当前 FR3 有线连接恢复正常，实时内核仍为
`7.0.0-38-realtime`，实时标志为 1。

已恢复仅状态读取的 launch，并用 `spawner --inactive` 重新加载试运行控制器；
未激活运动控制器或发送夹爪目标。新日志确认 1000 Hz 配置及 FIFO 50 设置成功。
既有计划的起点预检查再次通过，最大起点差约 `0.000222564 rad`，小于 `0.002 rad` 阈值。
记录为 `artifacts/flash_fr3/pilot_trials/flash_fr3_pilot_check_20261028_125250_169370895`，
恢复摘要为 `artifacts/flash_fr3/runtime_checks/resume_20261028.json`。

宿主机归档与容器临时目录中仍未找到 QoS 修复后的真机执行结果，首次小动作实测待完成。
H100 的 `10131` 服务端口当前拒绝连接；固定小动作不依赖推理服务，可先由人在 ThinkPad
交互终端执行上述 `--execute` 命令，再检查完整 `FLASH_PILOT_RESULT`。

### 2026-10-28：首次正常小动作实测完成，心跳中断检查已准备

用户在现场执行首次小动作，记录为
`artifacts/flash_fr3/pilot_trials/flash_fr3_pilot_execute_20261028_125427_673223575`。
155 条控制器状态记录显示 `Waiting(0) → Moving(1) → Complete(4)`，reason 均为 0；
Moving 到 Complete 约 `6.0001 s`。以本次第一条实测 q 为基准，第 2 关节最大位移
`0.005005598 rad`（约 `0.286800°`），各关节最大返回偏差约 `0.005532°`；
记录中的最大绝对跟踪误差约 `0.00009704 rad`。原始报告的 excursion/final_gap
是相对历史计划起点的差，因此不能直接当作本次实际位移/返回误差。
重新计算的指标已存为该目录中的 `analysis.json`。

实际往返正常结束且控制器退回 inactive，后续只读查询确认 21 个命令接口均未占用。
上述轨迹指标来自约 25 Hz 状态采样，不是全部 1 kHz 周期极值的证明。
此结果验证固定小动作的正常执行与退出，未验证 FLASH 抓放任务或运动中心跳中断的停止行为。

已为客户端增加 `--watchdog-test`（必须同时指定 `--execute`），宿主入口增加
`--execute-watchdog` 和显式 `--plan PLAN_JSON`。该模式在观察到 Moving 约 1 秒后
永久停止本轮心跳，保持记录，不再发送运动许可。预期 C++ 控制器经过
`Braking(2) → LatchedHold(3)`，reason 为 `HeartbeatTimeout(1)`，随后独立停用。
只有在客户端尚未请求停用时已确认控制器 inactive，才返回 `WATCHDOG_TRIAL_FINISHED`。
若 1.5 秒内未确认独立停用，报告失败并进入既有客户端兜底收尾，不将兜底动作算成独立保护通过。
同时修正收尾检查：未确认 inactive 时明确报告 `STOP_STATE_UNCONFIRMED`。

7 项隔离 ROS 接口测试均通过，覆盖默认只读、计划不匹配、正常退出、控制器错误、
模拟服务端独立停用、服务端未停用时的客户端兜底、以及最终控制器状态缺失。
测试不加载硬件驱动、不模拟机器人动力学。
归档摘要为 `artifacts/flash_fr3/pilot_client_checks/watchdog_preparation_20261028.json`。

已用 50 条当前新鲜静止状态生成新计划
`artifacts/flash_fr3/pilot_plans/flash_fr3_pilot_watchdog_plan_20261028_01`，
卸载已使用的一次性 inactive 实例，并以新计划加载新实例，仍保持 inactive。
真实只读预检查通过，最大起点差约 `5.45e-6 rad`。准备过程中未发送机械臂或夹爪动作。

下一次由人在 ThinkPad 交互终端执行：

```bash
cd /home/dase-hw101/franka_ws
bash scripts/run_flash_fr3_pilot.sh --execute-watchdog \
  --plan artifacts/flash_fr3/pilot_plans/flash_fr3_pilot_watchdog_plan_20261028_01/plan.json
```

该检查会在小动作途中停车并保持停车位置，**不自动返回起点**；夹爪不动。
预期结果需同时包含 `status=WATCHDOG_TRIAL_FINISHED`、
`native_deactivation_observed_without_client_request=true`、`client_deactivation_requested=false`、
`controller_left_inactive=true`。执行后需检查实测轨迹，不能仅凭接口测试推断真实停止效果。
当前心跳中断实测尚未由代理执行。

### 2026-10-28：受控心跳中断实测完成，恢复 FLASH 实时输入检查

现场执行心跳中断检查的记录为
`artifacts/flash_fr3/pilot_trials/flash_fr3_pilot_execute-watchdog_20261028_130547_784314083`。
40 条状态记录确认 `Waiting(0) → Moving(1) → Braking(2) → LatchedHold(3)`，
减速与锁存的 reason 均为 `HeartbeatTimeout(1)`。开始减速时最后一次心跳接收时龄约
`0.20017 s`，Braking 到 LatchedHold 的记录间隔约 `0.24982 s`。
停止发送心跳约 `0.51556 s` 后，客户端查询到控制器已 inactive；
`native_deactivation_observed_without_client_request=true`、`client_deactivation_requested=false`。
这个查询时间包含轮询和服务延迟，不是经认证的物理停车时间。

第 2 关节相对本次第一条实测状态的最大位移约 `0.00200665 rad`（`0.114973°`），
采样状态中的最大绝对跟踪误差约 `8.65e-5 rad`。停车后保持中间位置，未自动返回；夹爪无动作。
后续只读查询确认运动控制器 inactive，21 个命令接口全部未占用。
该特定故障场景已获得真实执行证据，分析保存于上述目录的 `analysis.json`，
不据此宣称已验证所有进程、通信、碰撞或硬件故障。

已重新采集 8 帧完整真实双相机、关节与夹爪观测，输入检查 PASS，保存为
`artifacts/flash_fr3/observations/flash_fr3_live_observation_20261028_after_watchdog_01`。
此采集仅创建 ROS 订阅，无运动指令。当前 H100 `192.168.1.113:10131` 拒绝连接，
下一步在 H100 服务器恢复既有模型服务：

```bash
conda activate flash_fr3
cd ~/flash_fr3/flash_fr3_reproduction
bash scripts/run_flash_fr3_shadow_server.sh
```

看到 `FLASH_SHADOW_SERVER_READY` 后，继续用当前真实观测做只读推理与候选轨迹检查。
该阶段不重复训练，也不直接将原始预测发送到机械臂；既有完整抓放候选曾被首轮保护阈值拒绝，
正常小动作和心跳检查通过不会自动解除该拒绝。

### 2026-10-28：恢复服务后的候选拒绝与场景对齐选择

H100 服务恢复后，以新鲜真实观测运行只读保护层检查，记录为
`artifacts/flash_fr3/guard_shadow_runs/flash_fr3_guard_shadow_20261028_131014_633890240`。
预测有效、观测至响应约 `396.54 ms`，但当前约 `1.731 s` 的剩余候选被
小范围试验阈值拒绝：第 2 关节最大位移约 `0.73145 rad`、速度约 `0.71313 rad/s`、
加速度约 `1.67313 rad/s²`；对应限制为 `0.10 rad`、`0.25 rad/s`、`1.0 rad/s²`。
接受轨迹 0 条，机械臂和夹爪动作指令 0 次。

随后抽查训练 episode 0、36、50 的双相机起始图像，并与当前图像对照。
抽查训练画面背景为实验室标牌，当前为写字白板；可见视野和物体图像位置也存在差异。
当前第 5 关节约 `0.07143 rad`，超出训练实测最大值约 `1.23°`；
夹爪闭合度约 `-0.0004` 是接近全开处的微小测量偏差，单独记录。
本次拒绝不直接证明模型训练失败或由场景变化造成：第 2、6 关节原始预测仍处于训练观测位置范围，
小动作执行窗口不能直接作为完整任务的允许范围。

对照报告位于 `artifacts/flash_fr3/scene_comparison_20261028_01/comparison.html`，
关节分布与候选细节位于同目录 `distribution_comparison.json` 和 `summary.json`。
用户起初选择“可以，先恢复示教布置”，随后指出当前布置已经与训练相似。
重新核对后确认：整体任务布置确实相似，背景与图像位置的差异不能单凭照片认定为预测异常的原因。
将恢复背景作为继续诊断的前提过于严格，已纠正；可保留当前布置继续只读评估。

当前可确定的执行阻碍是：现有保护层沿用首次小动作的 `0.10 rad` 局部范围，
而模型下一段候选约 `0.73 rad`，且速度与加速度超过该试验阈值。
现有真机 C++ 控制器仅实现固定小动作及其心跳停止，不接收 FLASH 完整连续预测。
后续实际需要完成模型轨迹的控制器接入、连续衔接、限速与任务范围评估，
初始关节姿态差异另行核对；不把小动作通过或修改阈值本身记作模型动作已验证。

## 时间尺度与后续真机接口

固定版本的官方配置是 `horizon=16, n_obs_steps=8, n_action_steps=8, poly_prefix=1, poly_suffix=1, fit_pad=1, poly_order=6`，单步 Euler 推理。训练观测只含历史，窗口在 episode 边界重复首尾帧，不跨 episode。

15 Hz 数据每 4 帧采样，`dt=4/15 s`；多项式核心覆盖 10 点，加两端拟合点后共 12 点，因此归一化参数 `s∈[0,1]` 的完整时长是 `T=11*dt=2.933333 s`。当前观测在 `s=2/11`；动作块取当前点起的 8 点，下一块边界位于 `8*dt=2.133333 s`。不能用 `8*dt` 或完整输入 horizon 的时长代替 `T` 来缩放导数。

官方 `predict_action` 的 velocity、acceleration 是对归一化 `s` 的导数：真机接口必须用 `dq_dt=velocity/T`、`ddq_dt2=acceleration/T²`。夹爪维度单独转换成夹爪指令，7 个关节维度用于关节参考。相关数值已保存至检查点的 `config.physical_time`。

官方训练 target 的 KKT 约束不保证模型预测的相邻动作块自动位置和速度连续。实际控制接口已只读核对，原始边界差与延迟已记录，连续参考适配已通过上述离线检查。进入 FR3 前仍需完成运行时约束、实时执行与故障处理；当前脚本不会驱动机械臂。

## 从原始目录重新准备

```bash
python scripts/prepare_flash_fr3_data.py \
  --dataset /你的/quest3_franka_dualcam_pickplace_74eps \
  --output data/pickplace_74eps_stride4 --stride 4
python scripts/test_flash_fr3_adapter.py
```

已有输出目录会被拒绝覆盖。改变采样步长后必须重新训练，真机观测间隔也必须使用对应时间尺度。

## 2026-10-28：FLASH 多项式短段控制器已接入，等待现场实测

新增独立包 `src/flash_fr3_model_controller`，插件
`flash_fr3_model_controller/ModelController`，实例名 `flash_fr3_model`。
原有固定小动作控制器源码未改动。为发现新插件，在确认运动控制器 inactive、
21 个命令接口全部 unclaimed 后正常重启了只读 launch；当前只加载状态广播器和新的 inactive 模型控制器。

本阶段是**冻结真实观测的模型短段试运行**：采集新鲜 8 帧双相机/关节/夹爪历史，
向 H100 第 15000 步 EMA 模型请求连续 Legendre 系数，按观测延迟选择模型时间起点。
在既有 C2 连续修正后，精确转换前 `0.075 s` 模型曲线为幂基多项式，
以 8 倍时间播放 `0.6 s`，再追加 `0.5 s` 的 C2 停车段，总计 `1.1 s`。
转换不是采样点拟合，也没有将各关节单独裁剪。夹爪维度不发送指令。
执行时以 SDK 当前 `q_d` 为原点，启动前核对实测起点偏差不超过 `0.002 rad`、
机器人近似静止且 native `dq_d/ddq_d` 接近零。

这些连续修正、时间缩放、冻结执行和停车段都是 FR3 部署试验适配，
不属于论文原始配置，不代表已经实现在线滚动重规划或完成抓放。
快照计划有效期为生成后 600 秒；每个控制器实例只允许运行一次，
再次试验必须重新采集、生成计划并加载新的 inactive 实例。

控制器在 1000 Hz 原生 update 内解析求值，不做网络 IO；Python 仅执行激活和 50 Hz 心跳。
沿用 `0.2 s` 心跳超时与 `0.25 s` 停车时间，并在超时、错误或正常完成后
从 C++ 非实时回调请求原生停用。所有参考仍受 `0.01 rad` 相对位移、
`0.02 rad/s` 速度、`0.1 rad/s²` 加速度、`5 rad/s³` jerk 的限制。
配置前用 Bernstein 凸包保守界检查完整短段，以及任意名义参考状态启动的看门狗停车；
这属于关节参考数值约束，不是碰撞、硬件动态或全部故障的安全认证。

当前首次真实输入计划最大采样关节位移 `0.11816°`，
名义速度界约 `0.00519 rad/s`；任意时刻看门狗停车位移界约 `0.00283 rad`。
C++ 与 Python 全轨迹 q/dq/ddq/jerk 求值最大差 `2.22e-16`。
7 项 C++ 行为检查、2 项 Python 数值测试、7 项隔离 ROS 客户端场景检查通过；
隔离检查分别使用 domain 231、232 和 localhost only，未加载真机驱动或机器人动力学。
记录：`artifacts/flash_fr3/model_prefix_checks/model_prefix_checks_20261028_01`。
新控制器已配置为 inactive，真实启动/运动/停车仍待现场验证。

在 ThinkPad 宿主执行，H100 模型服务保持运行：

```bash
cd /home/dase-hw101/franka_ws
bash scripts/run_flash_fr3_model_prefix.sh --prepare
```

`--prepare` 只采集、推理、检查并配置 inactive，不激活、不发送心跳。
出现 `READONLY_PREFLIGHT_PASS` 后，在现场确认机械臂周围可进行这个小动作，急停可用，立即执行：

```bash
bash scripts/run_flash_fr3_model_prefix.sh --execute
```

此命令会实际运动，完成后停在短段终点，不自动返回，夹爪不动。
将完整 `FLASH_MODEL_PREFIX_RESULT` 返回后检查实际位移、跟踪误差与 inactive 状态。
`--check` 仅重新核对当前计划，不运动；过期或起点变化时重新 `--prepare`。
后续若检查该新控制器的真实断心跳路径，应先重新 `--prepare`，再使用
`--execute-watchdog`，该模式在 Moving 状态约 `0.25 s` 后停止心跳；
不要在正常执行后直接重用已消耗的实例。

### 2026-10-28 15:49：首次 FLASH 模型短段真机执行通过

用户现场执行 `--prepare` 后立即执行 `--execute`，本次计划
`9538b029c2444ccfa35cb7b9884b8c3e` 返回 `MOTION_TRIAL_FINISHED`。
状态按 Waiting → Moving → Complete（0 → 1 → 4）推进，全部采样 reason 为 0。
这是由真实双相机与状态输入生成的冻结 FLASH 曲线短段，播放及停车合计 `1.1 s`。

本次 30 条控制器状态样本中，相对第一条实测状态的最大关节位移为
`0.00204593 rad`（`0.117223°`），最大绝对跟踪误差为
`0.000177549 rad`（`0.010173°`）。这些是状态采样指标，不是连续时间极值。
最后一条指令位置与本次计划多项式终点完全一致，最后参考速度为零。
控制器由 C++ 自行请求停用，没有 Python 兜底停用请求；后续只读查询确认
`flash_fr3_model` 为 inactive，21 个命令接口全部 unclaimed。夹爪没有动作指令。

完整结果与核对记录：
`artifacts/flash_fr3/model_prefix_trials/flash_fr3_model_execute_20261028_154924_963536173/analysis.json`。
该结果验证了模型短段的正常执行与停用；在线滚动重规划、抓放成功率及新控制器的
真实断心跳路径仍需分别验证。下一步在模型轨迹上检查受控断心跳停车。

在现场可操作急停时，在 ThinkPad 宿主连续执行：

```bash
cd /home/dase-hw101/franka_ws
bash scripts/run_flash_fr3_model_prefix.sh --prepare && \
bash scripts/run_flash_fr3_model_prefix.sh --execute-watchdog
```

新实例运动约 `0.25 s` 后停止心跳，等待 C++ 超时减速、锁存并独立停用，
不返回起点、夹爪不动。检查 `WATCHDOG_TRIAL_FINISHED`、
`native_deactivation_observed_without_client_request=true`、
`client_deactivation_requested=false` 与 `controller_left_inactive=true`。
此现场试验由用户触发；分析与准备脚本未额外执行真机运动。

### 2026-10-28 15:54：模型轨迹的受控断心跳真机检查通过

计划 `e500a89682694afc93af9f6e8711b245` 返回 `WATCHDOG_TRIAL_FINISHED`，
实测状态 0 → 1 → 2 → 3；开始减速时心跳时龄 `0.200438 s`，
减速与锁存状态的记录间隔 `0.249851 s`。客户端停止心跳后约 `0.519186 s`
查询到 native inactive，该数包含服务查询延迟，不是认证的物理停车时间。
`native_deactivation_observed_without_client_request=true`，
`client_deactivation_requested=false`；后续只读查询确认 21 个命令接口全部未占用。
本次最大实测关节位移约 `0.040213°`，最大采样跟踪误差约 `0.008991°`；
夹爪未收到动作指令，没有自动返回。
记录：`artifacts/flash_fr3/model_prefix_trials/flash_fr3_model_execute-watchdog_20261028_155413_135926769/analysis.json`。

随后采集新的真实观测并只读预览一个完整剩余预测块，加 C2 起点修正及名义停车段，
默认时间放慢 8 倍，总计约 `13.05 s`。使用当前运行驱动的 URDF 计算
`fr3_link0` 到 `fr3_link8` 的法兰 FK，并与独立 Orocos KDL 核对。
当前候选沿基座 +X 约 `18.25 cm`，Z 下降约 `20.75 cm`，
最大法兰位移约 `27.64 cm`。相对 66 条训练轨迹 3874 个法兰位置采样，
候选各点到最近训练点的距离中位数约 `0.54 cm`、最大约 `1.66 cm`。
这只是几何位置对照，未验证接触、碰撞、物体位置或任务成功。
本阶段未发送动作，未改变短段控制器限值，也未形成可执行的完整块计划。

预览：`artifacts/flash_fr3/workspace_previews/flash_fr3_workspace_preview_20261028_01/preview.html`。
独立计算脚本为 `scripts/preview_flash_fr3_workspace.py` 和 `scripts/flash_fr3_kinematics.py`。
这些脚本不创建 ROS 控制客户端，仅计算数值曲线与前向运动学，不含机器人动力学仿真。

用户报告桌面与底座安装面大致同高、略低，具体高差尚未测量。
精确桌面高差不是当前 FLASH 模型的输入，也不是继续只读验证或开发控制接口的硬性前提；
前面过早强调精确测量，已澄清。高度信息仅用于额外的工作空间净空估算，
不能单凭高度或轨迹接近训练分布就宣称无碰撞。
此处 FK 终点是法兰；当前 URDF 的 hand TCP 固定偏移为零，不据此推断 SDK 或实体工具已校准的 TCP。
真实较大动作仍应分段检查，后续重点是在线滚动推理与连续轨迹衔接。

### 2026-10-28：一次在线预测衔接的短程真机接口

新增 `flash_fr3_model_controller/StreamController`，实例名 `flash_fr3_stream`。
首次预加载一段冻结 FLASH 曲线；实际运动开始后，采集新的双相机/关节历史并请求
H100 推理，在首段终点尝试替换一次后续参考。两段模型前缀各取 `0.05` 模型秒，
时间放慢 10 倍，各播放 `0.5 s`，最后追加 `0.5 s` C2 停车，最多运动约 `1.5 s`。
这些限时、慢放、C2 修正与停车均为当前 FR3 联调适配，不能当作论文原配置。

新预测使用的最新七关节状态、外置 RGB 和腕部 RGB 时间戳都必须晚于运动开始时刻；
不是仅在启动后重新请求一遍启动前的同一组图像。首个心跳按固定观测网格安排，
为运动后的新图像与网络传输留出时间。修正曲线使用切换点已有参考的 q/dq/ddq，
慢放前分别将初始速度和加速度乘 10、100，再换算回真实播放时间。
不能在每次新预测处将速度、加速度重置为零。

实时侧通过固定大小的 RealtimeBuffer 接收待切换计划。消息包含本次会话 ID、
序号、源时间戳和明确的机器人时钟切换时刻；最多接受一个替换。
C++ 在同一原生 update 内检查 C2 边界、完整多项式和任意名义时刻看门狗停车的
Bernstein 保守界；到切换时刻才替换，不在 Python 网络回调中直接修改正在求值的数组。
客户端要求新计划在切换前至少 `40 ms` 就绪，原生侧拒绝剩余不足 `20 ms` 的计划。
预测过期、缺帧、迟到、边界不符或越限时保留首段已验证的停车参考，不继续外推。

仍保持相对本次 native q_d 原点的 `0.01 rad` 位置、`0.02 rad/s` 速度、
`0.1 rad/s²` 加速度与 `5 rad/s³` jerk 限值；该位置界不会在切换时重置。
心跳超时 `0.2 s`、看门狗停车 `0.25 s`，正常结束或锁存停车后由 C++ 请求停用。
夹爪不发送动作，不自动返回起点。每个实例只能激活一次。

相机接收放在独立的 Cyclone DDS 进程，控制客户端/管理器继续使用 Fast DDS。
相机进程只有订阅和本地 IPC，没有控制服务、心跳、机械臂发布器或夹爪目标。
这样能单独维护图像历史并减小图像处理对控制心跳的影响。
当前外置相机仍观测到偶发缺帧，尚未宣称解决全部采集抖动：
初版测试出现过约 `0.33 s` 的源帧间隔；后续只读测试有 6/6 衔接通过的批次，
最后加入“最新图像和状态必须晚于运行开始”要求的一批为 4/6 通过，另两次因相机
采样历史缺口返回 STOP_REQUIRED。所有这些批次均没有发送真机动作。
这反映输入链路限制，不能换算成抓放成功率。

6 项 C++ 核心行为、2 项 C++ ROS 原生停用、3 项 Python 数值检查与 7 种真实客户端
子进程场景通过，共 18 项。客户端场景覆盖默认只读、参数不匹配、过期计划、正常衔接、
迟到预测、控制器故障与原生停用未完成时的客户端兜底。
隔离测试 domain 231/232，localhost only，不含驱动或机器人动力学。
记录位于 `artifacts/flash_fr3/stream_checks/stream_checks_20261028_01`。
真实相机只读批次、输入和候选计划位于 `artifacts/flash_fr3/stream_trials`。

新插件已在全部命令接口未占用时重启只读管理器并加载为 inactive；
`--prepare` 和 `--check` 均通过，尚未在真实机器人上激活该在线衔接控制器。
两次前述模型冻结短段的真机成功结果不能代替本控制器的真机验证。

在 ThinkPad 宿主、机械臂现场执行以下命令，H100 推理服务保持运行：

```bash
cd /home/dase-hw101/franka_ws
bash scripts/run_flash_fr3_stream.sh --prepare && \
bash scripts/run_flash_fr3_stream.sh --execute
```

此命令会实际小幅运动。重新 prepare 可以避免旧快照过期或复用已消耗实例。
正常应返回 `ROLLING_TRIAL_FINISHED`、`handoffs_observed=1`、
`native_deactivation_observed_without_client_request=true`、
`client_deactivation_requested=false`、`controller_left_inactive=true` 和
`all_command_interfaces_unclaimed=true`。
输入或预测不满足要求时，可能返回 `STOPPED_WITHOUT_HANDOFF`，表示首段停车完成，
本次未观察到在线衔接；不能将其计为成功衔接或自动增大运动范围。
`--shadow` 可重复只读时序检查；默认 `--check` 不激活、不发心跳。
真机执行结果待用户返回 `FLASH_STREAM_RESULT` 后核对，完整抓放尚未完成。

### 2026-10-28 16:58：首次运动中在线预测衔接真机通过

用户现场执行 fresh `--prepare && --execute`，计划
`47a3e7255a644380ae3f61866a5d5474` 返回 `ROLLING_TRIAL_FINISHED`。
162 条状态记录显示 Waiting → Moving（sequence 0）→ 接收 pending 1 →
Moving（sequence 1）→ Complete，全部 reason 与候选拒绝码均为 0。
这次完成了一次运动中的新预测替换，夹爪未发送动作。

新候选在切换前 `116.119 ms` 就绪，最新关节状态和两张相机图像的源时间戳
均晚于实际运动开始，最小间隔约 `39.055 ms`。
从归档的两段多项式独立求值，衔接处参考位置误差约 `4.48e-17 rad`，
参考速度、加速度误差为 0；这里核对的是参考曲线的 C2 连续性。
相对首条实测状态的最大关节位移为 `0.177086°`，最大采样跟踪误差为
`0.0115853°`；这两个数来自状态采样，不是连续时间极值。

最后参考速度为零，C++ 自行请求停用，Python 未请求兜底停用。
试验后独立只读服务查询再次确认 `flash_fr3_stream` 为 inactive，
仅 `joint_state_broadcaster` active，21 个命令接口全部 unclaimed。
结果与独立核对：
`artifacts/flash_fr3/stream_trials/flash_fr3_stream_execute_20261028_165855_719706832/analysis.json`。

当前已走通实时观测、H100 推理、一次运动中 C2 衔接与正常停车的真机链路。
尚未完成连续多次滚动更新、完整抓放和任务成功率评估。
下一阶段是有界的多次滚动更新；不能直接把本次一次衔接当成完整任务验证，
也不能通过重复执行本次已消耗实例得到连续控制。

### 2026-10-28 17:18：连续两次在线衔接已准备，等待现场真机执行

控制器新增 `max_replacements` 参数，允许 1 或 2，原生硬上限为 2。
默认新计划使用两次替换：三个 `0.5 s` 模型前缀与最后 `0.5 s` 停车，
最长运动 `2 s`。每次候选必须使用当前前缀开始之后的新相机/关节观测，
不能在第二次替换中重复使用第一段开始之前的输入。
候选序号、未来机器人时钟、同一 native q_d 原点以及非零 q/dq/ddq 的 C2 边界
在 Python 与 C++ 两侧核对；原生侧达到上限后拒绝继续延伸。

在归档预测的数值检查中，原先 10 倍慢放连续到第三个前缀时，
任意时刻看门狗停车的加速度保守界可达到约 `0.1345 rad/s²`，
超过现有 `0.1 rad/s²`。新版因此每段取 `0.04` 模型秒、放慢 12.5 倍，
播放时长仍为 `0.5 s`；没有提高位置、速度、加速度或 jerk 限值。
位置范围始终相对本次激活的原点计算，不随预测切换重置。

11 项 C++ 核心检查、4 项 C++ ROS 接口/原生停用检查、4 项 Python 数值检查、
11 种真实 Python 客户端隔离场景通过，共 30 项。新增场景覆盖两次正常切换、
第二次迟到、第二次越限、缺失第二次候选、全程位置范围与原生次数上限。
这些软件检查没有加载机器人驱动或机器人动力学仿真。
记录：`artifacts/flash_fr3/stream_checks/stream_multi_checks_20261028_01/summary.json`。

真实双相机、关节状态与 H100 的只读时序检查 6/6 组通过，每组检查两次，
共 12 次候选边界。候选就绪提前量约 `57.5–225.5 ms`，C2 参考边界误差为零，
全部通过当前完整曲线及看门狗停车界。机械臂在这些检查中保持静止，
结果不能代替连续两次衔接的真机闭环验证，也不代表相机缺帧问题永久消失。
记录：`artifacts/flash_fr3/stream_trials/flash_fr3_stream_shadow_20261028_171656_790496644/report.json`。

在命令接口全部未占用后，正常结束旧只读 launch，编译新库，再恢复只读 launch。
新版已加载为 inactive，fresh prepare 与只读预检查通过，本阶段未发起运动或夹爪动作。
容器只读启动日志为 `/tmp/flash_fr3_observe_multi_20261028.log`。

保持 H100 推理服务运行，在 ThinkPad 现场执行：

```bash
cd /home/dase-hw101/franka_ws
bash scripts/run_flash_fr3_stream.sh --prepare && \
bash scripts/run_flash_fr3_stream.sh --execute
```

预期为 `ROLLING_TRIAL_FINISHED`、`requested_handoffs=2`、`handoffs_observed=2`，
并由 C++ 自行停用、全部命令接口释放。未完成全部衔接时沿已有参考停车；
零次返回 `STOPPED_WITHOUT_HANDOFF`，仅一次返回 `STOPPED_BEFORE_ALL_HANDOFFS`，
不能将这些返回值计为两次衔接成功。每次尝试、提前量和跳过原因保存在
`report.json` 的 `replacements`，已发布候选分别保存为 `replacement_01.json`、
`replacement_02.json`。夹爪仍保持不动，完整抓放尚未完成。

### 2026-10-28 17:20：连续两次在线衔接真机通过

用户现场执行计划 `4d5f59571ef74640bf3c4d5d78074af8`，返回
`ROLLING_TRIAL_FINISHED`，`requested_handoffs=2`、`handoffs_observed=2`。
217 条状态记录确认 sequence 0 → 1 → 2 → Complete，全程 reason 与拒绝码均为 0。
两次候选分别提前 `142.190 ms`、`61.617 ms` 就绪；最新关节和两路相机输入
分别晚于对应前缀开始约 `37.628 ms`、`70.940 ms`。

独立求值确认两次衔接的参考 q/dq/ddq 连续，位置误差均不超过
`2.13e-16 rad`；全部记录的参考 q/dq/ddq 与归档多项式核对通过。
最大实测关节位移约 `0.255060°`，最大采样跟踪误差约 `0.0121607°`。
这些最大值来自状态采样，不是连续时间极值；参考连续也不等于实测导数的连续性证明。

C++ 自行请求停用，客户端没有兜底停用请求，最后参考速度为零。
试验后独立只读查询确认 `flash_fr3_stream` inactive，仅状态广播器 active，
21 个命令接口全部释放。夹爪没有动作指令。
核对记录：
`artifacts/flash_fr3/stream_trials/flash_fr3_stream_execute_20261028_172054_974625413/analysis.json`。

当前完成的是有界短程真机在线控制链路。下一阶段转入任务接近段与持续滚动控制，
再接入夹爪抓取、释放并评估完整抓放成功率。本次结果不能当作完整任务成功，
当前已消耗的实例也不是可直接复用的持续任务控制器。

### 2026-10-28：首次整任务抓放尝试入口

新增 `scripts/run_flash_fr3_pickplace.sh`，默认 `--check` 只读取观测并配置
`flash_fr3_task` 为 inactive；`--execute` 才执行一次整任务尝试。
本入口使用真实双相机、七关节状态和第 15000 步 EMA 模型，采用
“执行有界模型段、C2 停车、读取停车后的新观测、重新规划”的方式。
这是首次真机任务部署适配，不是论文原始连续控制时序或效率测试。

独立 `TaskController` 插件保留 1000 Hz 原生位置求值、静止起点核对、
时钟/跟踪监控、0.2 秒心跳超时与原生停用。任务看门狗停车为 0.5 秒；
每个正常段带 0.75 秒停车，之后确认 inactive、全部命令接口释放再读取新预测。
单段取 0.2–0.6 模型秒、放慢 4/6/8 倍，按完整多项式及任意时刻停车的
Bernstein 保守界选择可执行组合，不裁剪越限轨迹。

本任务插件的部署范围与微动插件不同：每段相对 native q_d 原点的参考位移上限
0.20 rad、速度 0.20 rad/s、加速度 1.5 rad/s²、jerk 30 rad/s³。
同时检查带停车余量的绝对关节范围：
下界 `[-0.40,-0.86,-0.31,-2.57,-0.08,1.48,0.22]`，
上界 `[0.21,0.58,0.23,-1.83,0.11,2.90,1.00]`。
这是围绕训练观测范围设置的任务联调范围，不是机器人硬件限位或碰撞认证。
原模型短段与 stream 微动插件的限值未被改大。

从 66 条训练示教分别提取 66 个抓取、66 个释放事件姿态。
夹爪决策使用当前模型曲线锚点的闭合度，抓取阈值 0.7、释放阈值 0.3，
需两次不同时间戳的新观测确认。实测姿态须接近对应训练事件
（逐关节最大差不超过 0.12 rad），且实测与当前模型目标逐关节最大差不超过 0.06 rad。
抓取成功后至少完成一个搬运段才允许进入释放决策。

夹爪使用已安装驱动的 `/franka_gripper/grasp`、`/franka_gripper/move` 动作接口，
只在机械臂控制器 inactive 且静止时发送目标。目标抓取开度采用训练中抓取指令后
四个稀疏帧的实测开度中位数，约 28.76 mm，允许内外各 12 mm 偏差；
力 20 N、速度 0.02 m/s 是本次部署设置，并非从数据中学习的力控制参数。
空抓、动作失败、过期输入或越限会结束任务；失败时不自动张开或重试。
起点要求空手、夹爪开度至少 70 mm。初始化不自动执行 homing 或开合。

整个尝试最多 120 秒或 30 个机械臂段，完成一次抓取和释放即结束，不返回初始位姿。
终止时优先等待原生心跳停车与停用，必要时客户端兜底停用；
若夹爪动作仍未结束则调用它的 stop 服务，不自动执行释放。

5 项原生核心、2 项原生停用、6 项 Python 数值/任务时序、4 项真实 ROS 夹爪
传输检查和 3 种真实整任务客户端隔离场景通过，共 20 项。
隔离场景覆盖默认只读、抓取→搬运→释放、空抓后停止，未加载机器人驱动或动力学仿真。
真实双相机/H100/FR3 状态与夹爪接口只读预检通过；首段 C++/Python 曲线求值核对通过，
任务控制器保持 inactive，准备过程中没有发起机械臂或夹爪运动。
记录：`artifacts/flash_fr3/task_checks/task_checks_20261028_01/summary.json`。
只读记录：`artifacts/flash_fr3/pickplace_trials/flash_fr3_pickplace_check_20261028_174559_260739696/report.json`。

在 ThinkPad 现场、H100 推理服务保持运行时执行：

```bash
cd /home/dase-hw101/franka_ws
bash scripts/run_flash_fr3_pickplace.sh --execute
```

这条命令会实际移动机械臂并操作夹爪。分段停车是本入口的预期行为。
返回 `FLASH_PICKPLACE_RESULT`；`GRASP_AND_RELEASE_SEQUENCE_FINISHED` 表示机械臂段
以及抓取、释放命令序列执行结束，物体是否真正被放入盒中仍须现场观察确认。
`TASK_STOPPED_LIMIT` 表示达到次数/时间上限且任务尚未完成，`TASK_FAILED` 包含失败原因。
当前只完成软件检查与真实只读预检，尚未执行本入口的完整物理抓放任务。

## 保存起始关节姿态与独立回位

当前抓放起始位已只读保存至
`artifacts/flash_fr3/home_poses/task_start.json`（2026-10-28 18:02 香港时间）。
文件包含按 `fr3_joint1` 到 `fr3_joint7` 排序的七个实测关节角，单位 rad；
保存时机械臂连续静止，运动控制器 inactive，全部命令接口未占用。
回到同一组关节角可恢复机械臂及腕部相机的几何姿态；盒子与方块仍须人工复位。

在 ThinkPad 检查当前位置与保存姿态的差距，默认不运动：

```bash
cd /home/dase-hw101/franka_ws
bash scripts/run_flash_fr3_return.sh --check
```

确认物理返回路径无遮挡后执行实际回位：

```bash
bash scripts/run_flash_fr3_return.sh --execute
```

回位入口不依赖相机或H100；成功执行回位后自动开爪，需要原夹爪驱动在线。
`--save`和`--check`不发送机械臂或夹爪目标。
它沿关节空间插值，采用静止到静止的五次多项式，按最多0.14 rad划分路点，
由新鲜反馈重新生成的单段位移不得超过0.16 rad，时长选择仍满足既有原生导数和心跳停车限值。
逐段复用 `flash_fr3_task` 原生 1 kHz 执行、心跳停车及原生停用机制；
未改动现有控制器共享库或部署限值。
启动前检查整个提出的关节路径，实际每段再用新状态、Python 与 C++ 进行验证。
只支持既有任务部署关节范围，超出范围、其他控制器 active、状态过期、
终点误差超过 0.002 rad 时拒绝或停止；最多 45 段、240 秒。
`RETURN_FINISHED` 表示最终七个关节的最大误差不超过 0.002 rad（约 0.115°）；
`ALREADY_AT_SAVED_POSE`表示已在此容差内，无需运动机械臂；在`--execute`模式下仍自动开爪。
成功回位、机械臂控制器停用后，夹爪以0.1米/秒张开至80毫米。
回位失败不额外开爪；若开爪失败，报告失败并用`arm_return_completed`保留机械臂到位结果。

该入口没有环境碰撞规划。关节与导数边界通过不代表已检查桌面、盒沿或抓持物体；
夹爪仍在盒内或接触桌面时，应先手动引导脱离，不能直接假设关节插值路径畅通。
Ctrl+C 会停止心跳并按既有停车流程退出；不会自动开爪或继续回位。

以后可另存一个起始位，既有文件拒绝覆盖：

```bash
bash scripts/run_flash_fr3_return.sh --save --pose artifacts/flash_fr3/home_poses/another_start.json
bash scripts/run_flash_fr3_return.sh --check --pose artifacts/flash_fr3/home_poses/another_start.json
bash scripts/run_flash_fr3_return.sh --execute --pose artifacts/flash_fr3/home_poses/another_start.json
```

以下为2026-10-28初版验证记录，夹爪行为现以上述2026-11-13更新为准。
新增 5 项数学/原生边界检查及 5 种隔离 ROS 客户端场景通过；
场景包含保存、拒绝覆盖、默认只读、两段返回、拒绝接管 active 控制器。
原有 6 项任务数学/时序检查及 3 种整任务客户端场景回归通过，共 19 项。
隔离客户端使用预设消息与端点，不加载机器人驱动或动力学仿真。
真实只读检查返回 `ALREADY_AT_SAVED_POSE`，误差约 0.000287°，无机械臂或夹爪运动；
尚未实际执行该回位入口。软件记录见 `artifacts/flash_fr3/return_checks/return_checks_20261028_01/summary.json`。

## 首次完整抓放尝试：提前闭爪及抓取几何检查

`flash_fr3_pickplace_execute_20261028_180917_814329818` 实际完成 8 个机械臂段，
最大记录跟踪误差约 0.0323°。模型在第 8、9 次观测分别预测闭合度约
0.816、0.846，适配器随后调用 grasp；夹爪驱动执行了闭合，返回 SDK success=false。
结束后夹爪实测总开度约 0.628 mm，用户观察到夹爪在方块上方闭合。
这是一次失败的物理抓取，不能计为抓放成功。

相同 URDF 的 FK 计算得到闭爪前 `fr3_link8` 法兰高度约 0.18908 m；
最近关节姿态对应的训练抓取事件高度约 0.13491 m，相差约 54.17 mm。
这些是法兰相对机器人基座的高度，不是指尖到桌面/方块的距离。
原逐关节 0.12 rad 接近条件允许该姿态通过：最近事件最大关节差约 0.09653 rad。
模型当时预测的后续轨迹最低法兰高度仍约 0.17 m，单纯延迟夹爪不能保证到达方块。
用户确认当前方块颜色由训练中的蓝色换为绿色，尺寸和形状相同；
颜色变化是待对照的输入差异，尚未证实为唯一或主要原因。

新增 `flash_fr3_grasp_guard.py`，整任务客户端启动时读取当前控制器管理器的
`robot_description`，将 66 个训练抓取事件转换为法兰几何姿态。
闭合请求必须存在同一个训练事件同时满足位置差 ≤20 mm、朝向差 ≤10°、
最大关节差 ≤0.06 rad；法兰高度还须在训练抓取高度范围外扩 10 mm 的高度带内。
本数据集对应高度带约 [0.10796,0.15998] m。
这些阈值是本次部署的保守检查，不是原论文参数，也不是立方体检测、指尖 TCP 标定或碰撞认证。

模型请求闭爪而几何检查不通过时，记录 `grasp_blocked` 并结束尝试，
机械臂保持 inactive，不闭合夹爪，不盲目增加下降位移。
这项修改能阻止本次记录中的空抓条件，但没有修复模型目标偏高或证明抓取成功。
SDK 动作失败时也保留动作尝试、ROS action 状态、success/error 和可获取的实测开度，
避免 `gripper_actions=[]` 被误读为夹爪未运动；空错误文本会明确说明驱动未提供原因。

5 项几何检查、6 项原有任务数值/时序检查、4 项真实 ROS 夹爪传输检查、
5 种整任务客户端隔离场景通过，共 20 项。
隔离场景增加了 SDK success=false 且错误文本为空，以及几何不符时不发送夹爪目标。
只读记录重放确认：原条件通过、本次新条件拒绝，66 个示教抓取姿态自身均通过；
独立 KDL 与 FK 检查差为 0。没有启动真机运动进行这些检查。
记录见 `artifacts/flash_fr3/grasp_diagnostics/grasp_failure_20261028_01`。

当前位置回到保存起始位的只读规划通过，共 33 段，名义运动时间 123.75 秒：
`artifacts/flash_fr3/return_trials/flash_fr3_return_check_20261028_182144_116103980/report.json`。
尚未执行本次回位。现场返回路径畅通时可由用户执行：

```bash
cd /home/dase-hw101/franka_ws
bash scripts/run_flash_fr3_return.sh --execute
```

回位结束、机械臂静止且任务控制器 inactive 后，空手张开夹爪：

```bash
docker exec -it -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp franka bash -lc \
  'source /opt/ros/humble/setup.bash && source /home/ros/ros2_ws/install/setup.bash && ros2 action send_goal /franka_gripper/move franka_msgs/action/Move "{width: 0.08, speed: 0.02}"'
```

张开命令会实际运动夹爪；以上命令未由助手执行。优先用原蓝色方块做对照，
若仍提前请求闭合，新检查会终止尝试，应继续核对模型接近轨迹与训练/执行历史时序，
不能将几何保护通过视为已解决视觉泛化问题。

## 2026-10-29 控制器服务恢复

抓放入口在任何动作之前因 `/controller_manager/list_controllers` 不可用而退出。
检查旧日志确认机械臂控制管理器与独立夹爪进程均遇到 UDP receive timeout 后崩溃，
不是抓放模型已经开始执行。具体断连起因没有足够证据确定。

机器人 172.16.0.2 当前网络和 SDK TCP 端口已恢复，已重新启动
`flash_fr3_observe.launch.py` 状态驱动。确认只有 `joint_state_broadcaster` active，
21 个命令接口全部未占用，夹爪 grasp/move/stop 接口可用，两台相机有新鲜画面，
实时管理器以 1000 Hz、FIFO 优先级 50 运行。恢复过程中没有机械臂或夹爪运动指令。
记录：`artifacts/flash_fr3/runtime_checks/driver_recovery_20261029_01/report.json`。
新驱动日志：容器 `/tmp/flash_fr3_observe_recover_20261029.log`。

检查时 H100 地址 192.168.1.113:10131 仍拒绝连接，需要在 H100 终端重新启动推理服务：

```bash
conda activate flash_fr3
cd ~/flash_fr3/flash_fr3_reproduction
FLASH_GPU=1 bash scripts/run_flash_fr3_shadow_server.sh
```

保持服务器终端运行，出现 `FLASH_SHADOW_SERVER_READY` 后，在 ThinkPad 运行
`bash scripts/run_flash_fr3_pickplace.sh --check`；通过后才能再次 `--execute`。
不要再额外启动完整 VR launch 或手动激活另一套运动控制器。

## 2026-10-29 蓝色方块与历史输入对照

抓放记录 `flash_fr3_pickplace_execute_20261029_134556_295362133` 完成 8 段运动，
最大采样跟踪误差约 0.02668°。模型要求闭爪时法兰高度 0.183285 m，
高于示教抓取高度带上沿 0.159978 m；新增检查阻止闭爪并结束任务。
本次没有夹爪动作，也没有抓放成功。

已保存停车后绿色方块的八帧状态和图像；用户换回蓝色方块后，在同一机械臂姿态
重新只读采集。两次末帧最大关节差约 0.00002456 rad。
绿色静止历史的当前闭合预测为 0.68716，蓝色为 0.77094（闭爪阈值 0.7）。
蓝色预测曲线在 161 点网格上的最低法兰高度约 0.16942 m，仍高于抓取高度带上沿。
换回蓝色未解决偏高请求；一次对照不能排除其他视觉差异，也不能证明颜色完全无影响。

另外对最接近实测关节姿态的三个训练 episode 做了离线输入对照，
每条取接近和抓取两个时刻，分别使用原始八帧历史与末帧重复历史，共 12 次预测。
末帧重复是合成诊断输入，不是一次真实停走轨迹重放，也不是部署修复。
接近阶段的标签均为保持张开：

| Episode | 原始运动历史的当前闭合预测 | 末帧重复历史的当前闭合预测 |
| --- | --- | --- |
| 57 | -0.0053 | 0.9248 |
| 62 | -0.0150 | 0.2620 |
| 70 | -0.0124 | 0.9783 |

负值是未经裁剪的模型输出；按 [0,1] 裁剪后视为张开。
三条示教的真实抓取时刻，在两种历史条件下均预测闭合，约 0.959 至 1.004。
这证明历史输入变化可在部分训练接近帧上造成提前闭爪，
是当前停走部署需要处理的可复现问题；不能据此断言它是实际失败的唯一原因。
原训练使用连续运动、4/15 秒间隔的八帧历史；当前适配器将运动慢放 4 至 8 倍，
每段停车并重新配置，但仍按真实墙钟 4/15 秒取历史，二者的运动时序分布不同。

新增入口仅用于只读采集与预测比较，不创建运动发布器或夹爪动作客户端：

```bash
cd /home/dase-hw101/franka_ws
bash scripts/run_flash_fr3_grasp_probe.sh blue
```

可在标签后提供既有 probe 的 `report.json`，检查是否保持同一关节姿态并比较当前闭合值。
入口会读取控制器、硬件接口与 URDF，要求机械臂 inactive、21 个命令接口未占用，
并检查八帧历史的关节变化不超过 0.002 rad；相机接收沿用独立 Cyclone 进程。
每次保存实际八帧 `observation.npz`、图像、元数据、预测、URDF 和报告。
实际绿色观测回放、蓝色实时采集以及脚本语法检查已通过，没有机械臂/夹爪运动。

证据目录：

- `artifacts/flash_fr3/grasp_diagnostics/grasp_failure_20261029_01/summary.json`
- `artifacts/flash_fr3/grasp_diagnostics/grasp_failure_20261029_01/dataset_probe/report.json`
- `artifacts/flash_fr3/grasp_probes/flash_fr3_grasp_probe_green_20261029_01/report.json`
- `artifacts/flash_fr3/grasp_probes/flash_fr3_grasp_probe_blue_20261029_135321_269961166/report.json`

当前只完成诊断和只读工具，没有修正模型或执行时序，也没有重新训练或更换权重。
下一步需要修正历史输入与慢放/停车执行之间的时间尺度关系，并先离线检验。
不要通过放宽抓取高度检查或自动下降补偿来掩盖这项问题。

## 2026-10-29 执行历史时钟适配与新推理接口

已新增 `flash_fr3_history_clock.py`、`flash_fr3_retimed_buffer.py` 和
`flash_fr3_retimed_schema.py`。模型时间只在已成功执行的机械臂段内推进，
按真实执行时间除以该段慢放倍数计算；配置、推理和停车后的等待时间冻结。
名义减速尾段按同一倍数推进，这是部署适配规则，不是原论文配置。
每段的实际开始/完成时间来自原生状态中的 phase=1/4 时间戳，
与计划时长偏差超过 20 ms 或缺少成功状态时拒绝记录。

旧的墙钟取样通常只看到停车后的画面；新接收进程保留有界任务期间的压缩消息历史，
向前选择对应于模型 4/15 秒间隔的真实相机和状态消息。
最新一帧始终来自当前场景，历史不足、重复帧、时钟变化或源消息过期会报错。
墙钟时间、单调时钟和模型进度分别保存，不将真实传感器时间戳伪装成正常速度。
新格式为 `flash_fr3_retimed_observation_v1`，服务端验证映射并回传本次历史时钟；
客户端拒绝未确认同一时钟的预测。旧 v1 观测接口保持兼容。

新入口仅进行接近实验，夹爪目标被禁用：

```bash
cd /home/dase-hw101/franka_ws
bash scripts/run_flash_fr3_retimed_approach.sh --check
```

默认只检查并配置 inactive 计划。将来用户现场使用 `--execute` 才会运动机械臂；
即使满足抓取几何条件也以 `APPROACH_GEOMETRY_REACHED` 停止，不闭合夹爪。
预测位置不对齐或抓取几何不符时结束实验，不能当作抓取成功。
此入口要求从已保存起始姿态附近开始（最大关节差 ≤0.02 rad），
不能在方块上方重新启动后凭静止画面恢复已经丢失的接近历史。
当前位置不符合时返回 `RETURN_TO_SAVED_START_REQUIRED`，不会自动回位。
模型参考轨迹的时龄使用已冻结的模型时间 0，同时仍检查最新物理观测的墙钟时龄 ≤0.7 秒。
既有原生心跳停车、关节和导数边界、抓取几何检查均保留。

验证结果：

- 数值、消息取样和严格时间格式检查 28 项通过，包含既有任务/抓取/回位数学回归。
- 新 HTTP 格式接收、拒绝和旧格式分派检查 2 项通过；推理使用固定输出来检查接口，未冒充真实模型测试。
- 旧整任务隔离 ROS 客户端 5 种场景通过；新接近入口 7 种场景通过。
- 新接近场景包括只读、单段后停止、旧服务、错误时钟回传、几何不符、非起始位和预测不对齐；均未发送夹爪目标。
- 已记录真机的 8 个成功机械臂段时间戳重放通过，最终 1.8667 秒模型历史跨越约 14.3069 秒真实时间。
- 真实双相机/机械臂/夹爪八帧初始采集返回 `RETIMED_REAL_CAPTURE_PASS`，控制器保持 inactive，无机械臂或夹爪运动。

共 42 项软件/接口/隔离场景检查通过。没有机器人驱动加载到隔离测试域，
没有进行机器人动力学仿真。真实采集检查仅使用订阅与只读服务。
证据：`artifacts/flash_fr3/history_checks/history_update_20261029_01`。
这还没有验证实际 H100 对新格式的推理，也没有证明偏高抓取已经修好；
需要用户启动下面的新服务，再由 ThinkPad 检查真实模型链路。

新 H100 服务包与旧复现目录分开，继续使用同一份第 15000 步 EMA 权重，
默认物理 GPU 1、端口 10132。保留原 10131 服务，不终止其他 GPU 任务。

先在 ThinkPad 上传：

```bash
scp /home/dase-hw101/franka_ws/artifacts/flash_fr3/flash_fr3_history_update_20261029.tar.gz \
  hanyu@192.168.1.113:~/flash_fr3/
```

再在 H100 服务器终端执行：

```bash
conda activate flash_fr3
tar -xzf ~/flash_fr3/flash_fr3_history_update_20261029.tar.gz -C ~/flash_fr3
cd ~/flash_fr3/flash_fr3_history_update_20261029
FLASH_GPU=1 bash scripts/run_flash_fr3_history_server.sh
```

应出现 `FLASH_SHADOW_SERVER_READY`，同时显示 `port: 10132` 和
`retimed_observations: true`。需要保持服务进程运行。本步骤不会执行真机运动。

### 2026-10-29 新服务真实链路检查

H100 的 10132 服务已启动；ThinkPad 已通过真实双相机和关节状态采集八帧，
使用新格式请求第 15000 步 EMA 模型，返回 `RETIMED_REAL_CAPTURE_PASS`、
`inference: PASS`，服务器回传的执行历史时钟与请求一致。
前后均只有关节状态广播器 active，任务控制器 inactive，没有机械臂或夹爪运动。
观测、画面、预测与报告保存在：
`artifacts/flash_fr3/history_checks/history_update_20261029_01/live_inference`。
此次是静止起始历史的真实模型链路检查，尚未验证运动历史适配后的接近效果。

当前姿态距已保存起点的最大关节差约 65.25°。
已完成只读回位检查，结果 `RETURN_PREFLIGHT_PASS`：33 段、名义运动 123.75 秒，
完整路径的原生边界与 C++/Python 曲线核对通过，未激活控制器。
证据：`artifacts/flash_fr3/return_trials/flash_fr3_return_check_20261029_144212_158007446`。
回位入口不进行桌面或物体碰撞规划；现场确认路径畅通后，由用户执行：

```bash
cd /home/dase-hw101/franka_ws
bash scripts/run_flash_fr3_return.sh --execute
```

该命令仅恢复已保存的机械臂关节姿态，夹爪保持不动。
回位结果确认后再检查新版仅机械臂接近入口，不直接重试旧整任务抓放命令。

### 2026-10-29 保存起点确认与新版接近预检查

用户运行回位入口后返回 `ALREADY_AT_SAVED_POSE`，最大关节差 0.00015134 rad
（约 0.00867°），本次没有运动机械臂或夹爪。
证据：`artifacts/flash_fr3/return_trials/flash_fr3_return_execute_20261029_153544_300614062`。

随后在真实 ROS 域运行新版 `--check`，返回 `TASK_PREFLIGHT_PASS`：
10132 服务、新历史格式、真实双相机、当前 URDF 抓取几何配置和首段轨迹检查通过，
首段名义时长 3.15 秒，任务控制器保持 inactive，所有命令接口未占用。
证据：`artifacts/flash_fr3/retimed_trials/flash_fr3_retimed_check_20261029_153633_565462567`。
该预检查未验证后续接近轨迹或实际抓取结果。

用户现场下一步可运行：

```bash
cd /home/dase-hw101/franka_ws
bash scripts/run_flash_fr3_retimed_approach.sh --execute
```

此入口重新采集观测和生成计划，仅运动机械臂，始终不发送夹爪目标。
当模型要求闭爪时检查几何并结束实验；最多 120 秒、30 段。
只有返回 `APPROACH_GEOMETRY_REACHED` 才表示模型闭爪意图处的示教姿态检查通过，
仍不代表已经抓住物体，也不代表完整抓放成功。

### 2026-10-29 接近第 11 段后的时效停止与客户端修复

记录：`artifacts/flash_fr3/retimed_trials/flash_fr3_retimed_execute_20261029_153957_529488371`。
本轮 11 个机械臂段均完成并由原生控制器自行停用，夹爪没有动作。
最后一次观测到响应耗时 898.47 ms，超过既有 700 ms 限制；其中捕获约 262.21 ms、
HTTP 约 557.40 ms，服务端预测与解码约 28.13 ms。原错误将历史时钟缺失与过期合并，
且报告中的时龄仍是上一次通过的值，导致单看尾部不易定位。

已修改客户端：缺少时钟确认或时钟异常仍直接拒绝；有效但过期的预测在作任务决策前丢弃，
保持 inactive，并最多尝试三次新采集。每次记录实际时龄及采集、HTTP 耗时，
归档观测后再次核对时效，计划准备时的原时效检查也保留。
没有放宽 700 ms 限制，也没有放宽几何、关节或导数边界。
传感器子进程采用管道正常退出、幂等 ROS shutdown，父进程等待其清理，
超时后 SIGTERM 可真正终止子进程，避免原来的重复 shutdown 异常。
服务端代码和模型权重未改变。

12 项新版隔离场景（包含偶发过期恢复、连续过期拒绝、运动后过期恢复和两种子进程退出）
与 5 项旧客户端回归均通过。隔离域未加载机器人驱动或运行机器人动力学。
真实相机到 H100 的只读推理复查也返回 `RETIMED_REAL_CAPTURE_PASS`，正常退出。
证据：`artifacts/flash_fr3/history_checks/freshness_fix_20261029_01/summary.json`。

本轮所有决策的闭合值均低于 0.7，尚未触发闭爪意图。最后实测法兰 z 为约 0.21145 m，
最后一条预测在 161 点网格上的未来最低法兰 z 为约 0.15170 m；该值未被实际执行，
不能据此认定已解决抓取问题或已通过实际抓取几何检查。
数值分析保存在本轮记录的 `timing_diagnosis.json`。

当前姿态重新检查的回位计划为 31 段、名义运动 116.25 秒，返回 `RETURN_PREFLIGHT_PASS`。
证据：`artifacts/flash_fr3/return_trials/flash_fr3_return_check_20261029_154816_082010385`。
用户现场可先回位，成功后重新开始仅机械臂接近测试：

```bash
cd /home/dase-hw101/franka_ws
bash scripts/run_flash_fr3_return.sh --execute && \
bash scripts/run_flash_fr3_retimed_approach.sh --execute
```

两个入口均不发送夹爪动作；H100 保持现有 10132 服务运行。

### 2026-10-29 第 11 段后历史网格缺口与接收修复

记录：`artifacts/flash_fr3/retimed_trials/flash_fr3_retimed_execute_20261029_155823_072093011`。
本轮仍有 11 段完成、夹爪未动作，随后观测构建报历史网格未覆盖。
旧版本只留下通用错误，没有归档失败时相邻的原始时间戳，无法确定该轮具体失败来源。

为此新增只读长时取样工具 `check_flash_fr3_history_retention.py`：
使用真实相机、机械臂和夹爪状态，在机械臂静止时，将已记录区间的时间表平移到当前时段，
仅作为软件取样测试，不向模型发送这些测试区间，也不发布机器人命令或运行动力学。
每路完整时间戳记录保存在 `source_timestamps.npz`。
约 67 秒复查中，原读法出现过 0.18799 秒的关节源消息缺口。

发现原读法每次遍历长达数万条的 1 kHz 关节消息，而策略接收器仍采用深度 1 的队列；
这会在 Python 接收线程受阻时丢失历史消息。已改为直接二分查找八个所需时间点，
只复制选中的 32 个消息引用，在接收回调中检查时间戳顺序；策略接收器的关节队列改为 256。
机械臂控制客户端自身的实时状态订阅和原生控制器未改变。
最新时间点也改为四路已接收消息共同覆盖的时间，避免固定 now-80ms 时间点超前于延迟相机。
历史目标时间、80 ms 对齐界限、700 ms 预测时效和全部运动/几何边界保持原值。

失败时会记录样本编号、历史/最新帧类型、来源、目标、所选与相邻时间戳、缓存起止范围。
诊断通过子进程管道返回，写入最终报告的 `sensor_history_coverage_failures`。
真正缺失的历史数据仍拒绝使用，不复制末帧或自动补齐。
报告另存 `completed_history_clock`，避免把上一条成功预测的时钟误当作最后完成的运动进度。

最终版本验证：

- 16 项时间/取样检查通过，包含 10 万条关节历史的有界查找及接收顺序检查。
- 14 项新版隔离场景和 5 项旧客户端回归通过；故意制造的历史缺口被拒绝并正确记录来源。
- 改后约 67.49 秒真实静止采集的 11 组取样均通过，关节最大源消息间隔约 1.58 ms。
- 真实双相机至 H100 推理链路再次通过，控制器始终 inactive，无本轮验证触发的机械臂/夹爪运动。

证据：`artifacts/flash_fr3/history_checks/coverage_fix_20261029_01/summary.json`。
该验证识别并修复了可导致历史缺口的接收问题，没有证明前一轮缺口一定来自同一路，
也未验证修复后的实际抓取结果。

当前回位只读检查通过：33 段、名义运动 123.75 秒。
记录：`artifacts/flash_fr3/return_trials/flash_fr3_return_check_20261029_161804_627864515`。
用户可沿用上面的回位成功后接近命令，保持 H100 10132 服务运行，夹爪不动。

### 2026-10-29 外置相机历史缺口与再次起点检查

16:20 的回位已完成，33 段，最终最大关节误差约 0.000146 rad。
随后 `flash_fr3_retimed_execute_20261029_162427_045648238` 完成两段接近后停止，
诊断明确指向历史样本 6 的 `primary_cam`：相邻真实源时间戳相隔约 0.43197 秒，
最近一帧距目标约 0.15990 秒，超过 0.08 秒界限；腕部、关节、夹爪状态在该目标处均覆盖。
夹爪未动作，控制器最终 inactive。这次不能归因于已修复的长关节历史遍历。

16:27 的再次启动返回 `RETURN_TO_SAVED_START_REQUIRED`，没有发运动命令：
机械臂仍位于两段接近后的位置，与保存起点的最大关节差约 0.17534 rad（10.05°）。
16:39 重新执行只读回位检查，返回 `RETURN_PREFLIGHT_PASS`，6 段、名义运动 22.5 秒。
该检查不包含环境碰撞规划。

另做静止条件下 60 次真实双相机至 10132 服务的只读推理，全部返回有效预测，
无机械臂或夹爪命令；这不能证明运动期间的外置相机缺口已解决。
尚未区分缺口发生在相机发布端还是接收/传输端，未改变相机参数、DDS 配置或对齐界限。
全机累计 UDP 接收溢出计数不能单独证明该相机接收进程丢包。

证据：`artifacts/flash_fr3/history_checks/camera_gap_diagnosis_20261029_01/summary.json`。
下一步先由现场用户执行回位，暂不把接近命令接在回位命令后自动执行：

```bash
cd /home/dase-hw101/franka_ws
bash scripts/run_flash_fr3_return.sh --execute
```
