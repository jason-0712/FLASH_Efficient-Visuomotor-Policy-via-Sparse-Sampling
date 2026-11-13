```bash
# 终端 1：H100 服务器 server1cps；使用当前已安装环境和原 15k EMA 模型。
# 已有服务显示 FLASH_SHADOW_SERVER_READY 时，保留原终端，无需重复启动。
conda activate flash_fr3
cd ~/flash_fr3/flash_fr3_history_update_20261029
FLASH_GPU=1 bash scripts/run_flash_fr3_history_server.sh
# 出现 FLASH_SHADOW_SERVER_READY、port=10132 后，保持此终端打开。
```

```bash
# 终端 2：ThinkPad 宿主机；启动相机、机械臂状态驱动和夹爪驱动。
# 先在 Franka Desk 完成 Hand 初始化并激活 FCI。
# 原驱动终端仍在运行时保留原终端，不重复执行本段。
cd /home/dase-hw101/franka_ws
docker start franka franka_dual_realsense
docker cp scripts/flash_fr3_observe.launch.py franka:/tmp/flash_fr3_observe.launch.py
docker exec -it franka bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/ros/ros2_ws/install/setup.bash
  export ROS_DOMAIN_ID=30
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  exec ros2 launch /tmp/flash_fr3_observe.launch.py robot_ip:=172.16.0.2
'
# 保持此终端打开；等待驱动启动完成后使用终端 3。
```

```bash
# 终端 3：ThinkPad 宿主机；每次试验前回到已跑通的示教起始姿态。
# 在上一任务结束、回位路径无遮挡时执行；成功到位后自动开爪。
cd /home/dase-hw101/franka_ws
bash scripts/run_flash_fr3_return.sh --execute \
  --pose artifacts/flash_fr3/home_poses/demo_episode036_start.json
```

```bash
# 同一终端 3：回位成功并摆好蓝色方块和盒子后，执行一次完整自主抓放。
cd /home/dase-hw101/franka_ws
bash scripts/run_flash_fr3_pickplace.sh --execute \
  --server http://192.168.1.113:10132
# 任务结束后，在同一终端重复上一段回位命令，再进行下一次抓放。
```
