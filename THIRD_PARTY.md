# 上游与依赖来源

本仓库为FLASH的单任务仿真复现和自有FR3任务训练与部署适配，并非论文作者官方仓库。

- 论文：[FLASH: Efficient Visuomotor Policy via Sparse Sampling](https://arxiv.org/abs/2605.15492)。
- 作者官方代码：[NTUMARS/FLASH-Policy](https://github.com/NTUMARS/FLASH-Policy)。通过`third_party/FLASH-Policy`子模块引用固定提交`f47e490f2b820f835d1b9145da3cdb3df69f50a0`，保留其Apache-2.0许可证与第三方声明。
- 机器人硬件与Hand驱动：[Franka Robotics franka_ros2](https://github.com/frankarobotics/franka_ros2)、[libfranka](https://github.com/frankarobotics/libfranka)，分别适用上游许可证，本仓库不复制完整驱动源码。
- ROS控制基础：[ros-controls/ros2_control](https://github.com/ros-controls/ros2_control)。
- 硬件描述函数：[utiasDSL/crisp_controllers_demos](https://github.com/utiasDSL/crisp_controllers_demos)，参考提交`d12b9adb9161a6078b36da3bd93fa8e8422572e4`。本仓库保留一个本地硬件描述补丁，其余依赖由现有机器人工作区提供。
- 基础工作区：[jason-0712/franka_ws](https://github.com/jason-0712/franka_ws)。此次仅导出FLASH相关源码，没有修改原工作区、其远端或已安装驱动。
- 仿真结果：`simulation/` 收录2026-10-28—29在 LIBERO/MuJoCo 单任务上的本地实验记录。任务、模型和历史评测适配基于上述固定版本的官方 FLASH/RoboVerse 实现；部分补丁包含上游代码上下文，沿用其许可证。该目录中的采集脚本使用官方 RoboVerse 资产和参考轨迹；资产、示范数据、模型权重及二进制轨迹未随本次结果整理分发。详细来源、偏离和适用范围见各阶段报告及 `simulation/export-manifest.json`。

ROS包`package.xml`中原有许可证声明保持不变。具体源码、模型和数据的权利归各自作者；数据集和训练检查点不随本次源码推送分发。
