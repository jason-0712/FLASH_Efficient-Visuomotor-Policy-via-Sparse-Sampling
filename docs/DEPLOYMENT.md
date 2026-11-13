# FLASH训练与Franka FR3部署

H100端使用Python 3.11进行数据处理、训练和推理。机器人端使用已有FR3驱动、双相机和`franka`容器中的ROS 2 Humble / Python 3.10；容器工作区为`/home/ros/ros2_ws`，机器人宿主为Ubuntu 24.04实时内核。

## 1. H100环境与训练

已验证训练环境为PyTorch 2.7.0+cu128、Torchvision 0.22.0+cu128、NumPy 1.26.4和NVIDIA H100。在已有CUDA环境及本仓库根目录中运行：

```bash
python -m pip install -r scripts/flash_fr3_requirements.txt
export FLASH_ROOT="$PWD/third_party/FLASH-Policy"

python scripts/prepare_flash_fr3_data.py \
  --dataset /absolute/path/to/quest3_franka_dualcam_pickplace_74eps \
  --output data/pickplace_74eps_stride4 --stride 4 --seed 42

CUDA_VISIBLE_DEVICES=1 python -u scripts/train_flash_fr3.py \
  --flash-root "$FLASH_ROOT" \
  --dataset-cache data/pickplace_74eps_stride4 \
  --output runs/train_seed42_15k --steps 15000 --schedule-steps 15000 \
  --seed 42 --batch-size 32 --device cuda:0

CUDA_VISIBLE_DEVICES=1 python scripts/eval_flash_fr3.py \
  --flash-root "$FLASH_ROOT" --dataset-cache data/pickplace_74eps_stride4 \
  --checkpoint runs/train_seed42_15k/best.pt \
  --output runs/eval_seed42_15k --device cuda:0
```

以上命令沿用已归档15k训练的74条示教配置：66条训练、8条验证，按整条episode划分。替换为自己的采集目录时，以生成的manifest记录实际episode数与划分。

数据适配要求原采集器的LeRobot v2.1字段和`meta/merge_manifest.json`。两路RGB为256×256，每4帧采样，保留8帧历史并预测8个动作点，采样间隔为4/15秒。七关节监督来自示教实测关节，夹爪动作统一为闭合比例；原始6维末端动作没有直接作为关节命令。

训练调用作者官方FLASH核心，使用EMA检查点。子模块固定为`f47e490f2b820f835d1b9145da3cdb3df69f50a0`；加载已保存检查点时保持源码版本和归一化参数一致。

## 2. 机器人端依赖

需要`controller_manager`、`franka_hardware`、`franka_semantic_components`、`franka_gripper`、`franka_description`、`rclpy`、`cv_bridge`、OpenCV、NumPy和`rmw_cyclonedds_cpp`。成功环境使用libfranka 0.19.0、ros2_control 2.52.0，保留原生信号处理和机器人保护。

基础驱动与Docker来自[jason-0712/franka_ws](https://github.com/jason-0712/franka_ws)。本仓库提供FLASH相关源码；完整驱动、Docker镜像和系统内核由基础工作区提供。

`flash_fr3_observe.launch.py`复用CRISP硬件描述函数，参考提交为`d12b9adb9161a6078b36da3bd93fa8e8422572e4`。[硬件描述补丁](../patches/crisp_fr3_hardware_version.patch)针对该版本设置FR3的`version=0.0.0`。

| 输入 | 默认话题 |
| --- | --- |
| 外部相机RGB压缩图像 | `/right/right_third_person_camera/color/image_raw/compressed` |
| 腕部相机RGB压缩图像 | `/right/right_wrist_camera/color/image_raw/compressed` |
| 机械臂状态 | `/franka/joint_states` |
| 夹爪状态 | `/franka_gripper/joint_states` |

相机SDK时钟配置见[flash_fr3_camera_clock.py](../scripts/flash_fr3_camera_clock.py)和[观测客户端](../scripts/run_flash_fr3_shadow_client.py)。机械臂客户端使用Fast DDS，独立图像接收进程使用Cyclone DDS；ROS_DOMAIN_ID为30。

## 3. 构建控制器与恢复驱动

首次部署或维护时，在已有容器工作区中构建：

```bash
docker cp src/flash_fr3_model_controller franka:/home/ros/ros2_ws/src/
docker cp src/flash_fr3_pilot_controller franka:/home/ros/ros2_ws/src/
docker exec -it franka bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/ros/ros2_ws/install/setup.bash
  cd /home/ros/ros2_ws
  colcon build --packages-select flash_fr3_model_controller flash_fr3_pilot_controller
'
```

正式脚本使用`preview_continuous_effort`及持续力矩插件。已在运行的实验设备不需要因为仓库文档更新而重新构建。

机器人重启后，在Desk完成Hand初始化并激活FCI，再启动驱动：

```bash
docker cp scripts/flash_fr3_observe.launch.py franka:/tmp/flash_fr3_observe.launch.py
docker exec -it franka bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/ros/ros2_ws/install/setup.bash
  export ROS_DOMAIN_ID=30
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ros2 launch /tmp/flash_fr3_observe.launch.py robot_ip:=172.16.0.2
'
```

保持终端打开。该launch激活状态广播器与标准夹爪驱动；完整抓放还需要相机节点与H100服务。回位不依赖H100或相机，成功后的开爪需要Hand驱动。

## 4. 观测预热与H100推理服务

在机器人端已启动相机与状态驱动的ROS终端中采集：

```bash
python3 scripts/capture_flash_fr3_observation.py --output observations/warmup
```

该命令读取真实图像与状态。将生成的`observation.npz`传到H100，在服务器仓库根目录运行：

```bash
export FLASH_ROOT="$PWD/third_party/FLASH-Policy"
CUDA_VISIBLE_DEVICES=1 python -u scripts/serve_flash_fr3_shadow.py \
  --flash-root "$FLASH_ROOT" --dataset-cache data/pickplace_74eps_stride4 \
  --checkpoint runs/train_seed42_15k/best.pt \
  --warmup-observation /absolute/path/to/observation.npz \
  --bind 192.168.1.113 --port 10132 --allowed-client 192.168.1.107 \
  --device cuda:0 --allow-retimed-observations
```

IP、路径和相机话题替换为实际环境。正式客户端发送真实物理时间历史；`--allow-retimed-observations`保留旧接口兼容能力。`run_flash_fr3_history_server.sh`沿用原实验机目录，新的克隆优先使用上述显式参数。

## 5. 任务配置与正式入口

首次部署时创建本地配置目录：

```bash
mkdir -p artifacts/flash_fr3/task_setup artifacts/flash_fr3/home_poses
cp configs/gripper.example.json artifacts/flash_fr3/task_setup/events.json
```

任务起点来自本机实际示教或已验证的保存姿态。在正确起点静止时可保存：

```bash
bash scripts/run_flash_fr3_return.sh --save \
  --pose artifacts/flash_fr3/home_poses/task_start.json
```

入口拒绝覆盖已有目标。`configs/task_start.example.json`仅提供本次场景的参考姿态。

保持驱动、相机和推理服务在线，执行完整任务：

```bash
bash scripts/run_flash_fr3_pickplace.sh --execute --server http://192.168.1.113:10132
```

返回本机保存姿态：

```bash
bash scripts/run_flash_fr3_return.sh --execute \
  --pose artifacts/flash_fr3/home_poses/task_start.json
```

回位成功且控制器停用后，夹爪自动张开到80毫米；回位失败不会额外开爪。回位采用关节插值，需要现场确认路径无遮挡。

机器报告`TASK_SEQUENCE_FINISHED`与grasp/release成功表示执行序列完成。按物体是否实际抓起并放入目标盒内记录任务结果。原始运行记录默认保存到`artifacts/flash_fr3/raw_policy_trials/`；公开指标见[真机结果](../results/fr3/2026-11-13/README.md)。[配置摘要](../configs/fr3_successful_rollout.json)记录一条已确认成功运行的控制参数。

## 6. 常见运行问题

| 现象 | 检查位置 |
| --- | --- |
| `Connection refused` | H100服务、地址和10132端口 |
| 控制器管理服务不可用 | ROS驱动进程；Desk激活FCI后仍需启动ROS |
| `cartesian_reflex` | 原生驱动错误及本次换段记录，保留原有保护 |
| 推理时龄异常 | 观测源时钟、网络与处理时延 |
| `no space left on device` | 宿主和容器各自的磁盘空间 |

完整首次源码导出清单保留在[精简前历史记录](https://github.com/jason-0712/FLASH_Efficient-Visuomotor-Policy-via-Sparse-Sampling/blob/fa953cde97ac55e3f4fe9567ced8dce3f0dc7869/docs/source_export_manifest.json)。该次基础工作区HEAD为`e4ae615688d3f7b69748fb866b220c354fdf0ea8`，当时还包含未提交的开发改动。
