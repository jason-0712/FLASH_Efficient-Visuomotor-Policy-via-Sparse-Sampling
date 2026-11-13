# 上游与依赖来源

本项目将FLASH适配到Franka FR3真机视觉抓放，作者官方策略通过Git子模块引用。

- 论文：[FLASH: Efficient Visuomotor Policy via Sparse Sampling](https://arxiv.org/abs/2605.15492)。
- 官方策略：[NTUMARS/FLASH-Policy](https://github.com/NTUMARS/FLASH-Policy)，固定提交`f47e490f2b820f835d1b9145da3cdb3df69f50a0`，保留上游Apache-2.0许可证与第三方声明。
- 机器人驱动：[Franka Robotics franka_ros2](https://github.com/frankarobotics/franka_ros2)与[libfranka](https://github.com/frankarobotics/libfranka)。
- ROS控制框架：[ros-controls/ros2_control](https://github.com/ros-controls/ros2_control)。
- 硬件描述：[utiasDSL/crisp_controllers_demos](https://github.com/utiasDSL/crisp_controllers_demos)，参考提交`d12b9adb9161a6078b36da3bd93fa8e8422572e4`；本项目的[硬件描述补丁](patches/crisp_fr3_hardware_version.patch)针对该版本。
- 基础驱动与容器工作区：[jason-0712/franka_ws](https://github.com/jason-0712/franka_ws)，部署配置见[部署说明](docs/DEPLOYMENT.md)。
- [仿真补充验证](simulation/README.md)基于官方FLASH/RoboVerse的LIBERO/MuJoCo任务；完整实验脚本、补丁与来源记录保留在[历史归档](https://github.com/jason-0712/FLASH_Efficient-Visuomotor-Policy-via-Sparse-Sampling/tree/fa953cde97ac55e3f4fe9567ced8dce3f0dc7869/simulation)。

各依赖适用其上游许可证，ROS包的原有许可证声明保持不变。示教数据、训练权重与原始现场记录不随源码分发。
