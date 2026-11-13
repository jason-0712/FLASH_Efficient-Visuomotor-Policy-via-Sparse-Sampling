# FR3机器人端部署说明

本文件针对已有可工作的FR3驱动环境。当前成功实验的ROS节点运行在`franka`容器内，ROS 2 Humble、Python 3.10，工作区为`/home/ros/ros2_ws`；机器人电脑宿主为Ubuntu 24.04实时内核。H100训练使用另一个Python 3.11环境。

## 依赖和已有硬件接口

需要`controller_manager`、`franka_hardware`、`franka_semantic_components`、`franka_gripper`、`franka_description`，以及`rclpy`、`cv_bridge`、OpenCV、NumPy和`rmw_cyclonedds_cpp`。成功环境使用libfranka 0.19.0和ros2_control 2.52.0；原生信号处理和机器人保护保持启用。

本项目的`flash_fr3_observe.launch.py`复用已安装的`crisp_controllers_robot_demos/launch/franka.launch.py`硬件描述函数。参考CRISP提交为`d12b9adb9161a6078b36da3bd93fa8e8422572e4`；`patches/crisp_fr3_hardware_version.patch`保留FR3硬件描述中的`version=0.0.0`适配。该补丁仅针对这个CRISP版本，不需要运行完整VR遥操作launch。

基础机器人驱动和Docker环境的来源工作区为[jason-0712/franka_ws](https://github.com/jason-0712/franka_ws)。本仓库只导出FLASH相关部分，没有打包完整驱动依赖、Docker镜像或系统内核。基础工作区版本见`source_export_manifest.json`，并需保留其依赖补丁。

实际订阅的话题：

| 输入 | 默认话题 |
| --- | --- |
| 外部相机RGB压缩图像 | `/right/right_third_person_camera/color/image_raw/compressed` |
| 腕部相机RGB压缩图像 | `/right/right_wrist_camera/color/image_raw/compressed` |
| 机械臂状态 | `/franka/joint_states` |
| 夹爪状态 | `/franka_gripper/joint_states` |

正式接收器还需要相机SDK时间来源；相机话题和metadata配置见`flash_fr3_camera_clock.py`及`run_flash_fr3_shadow_client.py`。机械臂客户端使用Fast DDS，独立图像接收进程使用Cyclone DDS。不得在另一个ROS_DOMAIN_ID中启动后误认为服务已就绪。

## 安装本项目控制器

在首次部署或维护期间，将两个ROS包放入既有容器工作区并构建。已经在测试的设备无需因代码上传重新构建或重启。

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

正式脚本会使用构建出的`preview_continuous_effort`和安装的持续力矩插件；没有运行模拟器。

## 重启机器人后恢复原驱动

在Desk完成Hand初始化并激活FCI。将只读启动文件复制到容器：

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

保持终端打开。该launch只激活状态广播器和标准夹爪驱动，本身不发送开合目标。完整抓放还需要原有相机节点和H100 10132推理服务。仅回位不依赖H100或相机，成功到位后开爪需要Hand驱动。

## 观测、配置和正式入口

真实预热观测可在已启动双相机与状态驱动的ROS终端中采集：

```bash
python3 scripts/capture_flash_fr3_observation.py --output observations/warmup
```

该命令仅读取状态和图像，不发送动作。生成的`observations/warmup/observation.npz`传到H100后可用于服务预热；该文件不属于Git源码。

原回位入口可保存本机当前静止姿态：

```bash
bash scripts/run_flash_fr3_return.sh --save \
  --pose artifacts/flash_fr3/home_poses/task_start.json
```

已有目标文件拒绝覆盖。重新部署到相同机器可保留本机原目标；不要用中途停车姿态替换任务起点。

完整任务：

```bash
bash scripts/run_flash_fr3_pickplace.sh --execute --server http://192.168.1.113:10132
```

现场根据物体是否实际放入盒内记录成功/失败。机器报告`TASK_SEQUENCE_FINISHED`及两个夹爪动作成功是软件证据，物体落点需要单独观察。所有报告默认写入`artifacts/flash_fr3/raw_policy_trials/`。

返回本机保存姿态：

```bash
bash scripts/run_flash_fr3_return.sh --execute \
  --pose artifacts/flash_fr3/home_poses/task_start.json
```

回位成功、机械臂控制器停用后自动张开到80毫米；已在起点时也开爪。回位失败不额外开爪，开爪失败单独记录。回位是关节插值，没有桌面或物体碰撞规划。

## 已观察到的运行问题

- `Connection refused`：核对H100服务进程、地址和10132端口。
- `/controller_manager/list_controllers`不可用：检查机器人端驱动终端，Desk激活FCI不会自动恢复ROS进程。
- `cartesian_reflex`：查看原生驱动错误及该次模型换段记录；不要通过提高保护阈值掩盖问题。
- `推理时龄异常`：检查观测源时间、端到端传输和处理时间；不能仅凭这条信息认定GPU计算慢。
- `no space left on device`：检查宿主与容器各自磁盘；容器保存成功后，复制回宿主也可能失败。

这些是既有异常的解释，不是新增正式任务前置检查入口。
