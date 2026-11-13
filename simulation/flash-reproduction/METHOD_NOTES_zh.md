> 历史实验报告，记录的是2026-10-28—29执行时的状态；于2026-11-13整理发布。当前发布范围见[仿真实验总览](../README.md)。未发布的视频、权重、二进制轨迹等链接已标为“原始留档”；下文的服务器路径和原始文件完整性描述保留历史含义。

# 本次配置如何执行 FLASH

本笔记解释固定源码 `f47e490f2b820f835d1b9145da3cdb3df69f50a0` 的实际路径。成功率以 `results/` 中的原始评测为准。

## 输入与数据

本次输入是一台相机的 256×256 RGB 和 9 维关节状态。关节按名称排序，前两维是两个手指，后七维是机械臂关节。因此前两维位置单位是米，后七维是弧度；不能把九维统一称为 rad。

示范来自官方 RoboVerse 资产中的 50 条参考轨迹回放，重新渲染并保存。它们不是论文原始示范，也不是新遥操作。40 条示范经 stride 4 转成 1,204 帧。训练使用 39 条，验证使用参考 ID 49 的一条；划分随机种子为 42。

原始 MP4 写入 30 FPS，这是播放设置。MuJoCo 的物理步长是 0.001 s，scenario decimation 为 15，所以每次控制步为 0.015 s，stride 4 对应 0.060 s 的训练采样间隔。官方转换器把视频 FPS 写进 `raw_data_freq_hz`；评测的 FLASH 路径实际根据控制周期与 stride 计算时间，不能用容器 FPS 外推控制频率。

对应实现：[采集器](https://github.com/NTUMARS/FLASH-Policy/blob/f47e490f2b820f835d1b9145da3cdb3df69f50a0/scripts/advanced/collect_demo.py)、[转换器](https://github.com/NTUMARS/FLASH-Policy/blob/f47e490f2b820f835d1b9145da3cdb3df69f50a0/roboverse_learn/il/data2zarr_dp.py)、[数据集](https://github.com/NTUMARS/FLASH-Policy/blob/f47e490f2b820f835d1b9145da3cdb3df69f50a0/roboverse_learn/il/datasets/robot_image_dataset.py)。

## 从历史轨迹预测多项式

配置使用 8 帧历史、8 个动作点、horizon 16。ResNet-18 编码图像，GroupNorm 替换 BatchNorm，模型从随机初始化训练。DiT 使用 4 层、hidden size 512、8 个注意力头。

FLASH 在 Legendre 多项式系数空间预测轨迹，阶数为 6。历史关节轨迹拟合出历史系数，作为 flow 的起点；训练期间对历史系数加入标准差 0.5 的噪声，并用系数拟合正则项 0.1 改善数值条件。目标由专家动作轨迹拟合得到。默认 C1 位置/速度约束施加在专家训练目标的系数拟合上，推理时不再投影；因此不能声称相邻预测动作块严格 C1 连续。这也不等同于机器人执行器已采用速度前馈。

具体地，令历史系数为 C_h，专家目标系数为 C_t，随机抽取 t∈[0,1]，网络输入为 (1−t)C_h+tC_t，flow-matching 的监督速度为 C_t−C_h。训练损失还包含权重 1.0 的一步一致性项，要求 C_h+vθ(C_h,0,观测) 接近 C_t。默认一次 Euler 推理正好使用这一形式；loss 衡量系数预测误差，不能直接读作闭环成功率。

`poly_prefix=1`、`poly_suffix=1` 让多项式覆盖 8 个可执行点前后各一个点，即 10 点核心区间；`fit_pad=1` 再扩展首尾拟合点。配置中的 horizon 16 仍控制数据序列，不能把它简单理解为一次执行 16 个控制步。

正式训练保留 AdamW、cosine 调度、500-step 预热、batch 32、EMA、15,000-step 请求预算。runner 按完整 epoch 执行，总更新数可能略高于请求。主结果读取其中的 10,000-step checkpoint，不能把训练预算改成 10,000 后称为同一调度。

对应实现：[FLASH 配置](https://github.com/NTUMARS/FLASH-Policy/blob/f47e490f2b820f835d1b9145da3cdb3df69f50a0/roboverse_learn/il/configs/policy_config/flash.yaml)、[FLASH 策略](https://github.com/NTUMARS/FLASH-Policy/blob/f47e490f2b820f835d1b9145da3cdb3df69f50a0/roboverse_learn/il/policies/flash/flash_policy.py)。

## 闭环执行与控制限制

默认推理用一个 Euler 步，从历史系数预测动作多项式，再在更密的网格上求值得到动作。当前 ds4 配置把 8 个动作点重采样为 32 个控制目标，缓存执行后重新预测。8 个原始点首尾跨度为 7×60 ms=420 ms，重采样后的 32 个点仍覆盖这一多项式参数区间；按 15 ms 控制周期发出时，首尾目标相隔 31×15 ms=465 ms，完整 32 次控制步累计为 480 ms。这里保留发布版行为；后续比较 FM-DiT 时，应分别记录多项式参数覆盖、实际目标时刻和重规划间隔。

MuJoCo handler 实际向原生位置 actuator 写入关节目标。虽然上层配置启用 `send_vel_target` 和 `velocity_pd_gains`，本版本的 handler 只是缓存 `dof_vel_target`，`_simulate()` 没有使用该缓存；`_init_torque_control()` 中将配置增益写入 MJCF 的代码也被注释。因此本次不能报告“速度前馈已生效”，也不能把配置中打印的增益等同于原生 actuator 的有效增益。本轮保留发布版行为，没有为追求某个分数重写控制器。

对应实现：[评测策略执行器](https://github.com/NTUMARS/FLASH-Policy/blob/f47e490f2b820f835d1b9145da3cdb3df69f50a0/roboverse_learn/il/runners/default_eval_runner.py)、[MuJoCo handler](https://github.com/NTUMARS/FLASH-Policy/blob/f47e490f2b820f835d1b9145da3cdb3df69f50a0/metasim/sim/mujoco/mujoco.py)。

## 成功判据与评测场景

任务成功检查的是碗与盘子的中心位置：xy 距离小于 0.06 m，碗相对盘子的高度差大于 0 且小于 0.03 m。它没有单独要求松开夹爪、保持静止或稳定放置。闭环每回合最多 300 个控制步，任一步命中该几何判据即算成功。

50 次评测使用参考 ID 0–49，初始状态可逐条追溯。其中 40 个初始场景也用于生成数据集，剩余 10 个是未纳入示范的场景；这 10 个因回放采集结果而被排除，并非随机选择的独立测试集。报告应同时给出官方池总体结果和这些分组结果，不能称为 50 个未见场景的泛化成绩。

轨迹 7、25 在默认采集中曾命中成功判据，但补录结束前耗尽参考动作。两条分别重放，使用公开参数 `--tot_steps_after_success 0` 保存，原失败记录保留。两条的截断规则是本次复现偏离默认采集命令之处。

对应实现：[任务源码](https://github.com/NTUMARS/FLASH-Policy/blob/f47e490f2b820f835d1b9145da3cdb3df69f50a0/roboverse_pack/tasks/libero_90/libero_kitchen_scene1_put_the_black_bowl_on_the_plate.py)。

## 下一阶段 FM-DiT 对照

尚未执行。应复用同一批 40 条原始示范和相同划分，单独转换 ds1 数据，保留 FM-DiT 的 10 次推理迭代。先记录物理动作覆盖时长、控制器实际行为及相同初始状态，再解释差异。单次 FLASH 网络调用、整个 policy 调用、每环境步摊销时间是不同指标；官方日志包含冷启动调用，本轮不据此给出预热后的网络延迟或加速倍数。
