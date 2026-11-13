"""Read FR3 arm/hand states without activating command controllers.

Uses the installed CRISP hardware-description helper, but does not invoke its
full launch description: that would activate a command controller and start an
adapter that automatically opens the gripper. This entry point starts only the
state broadcaster and the standard gripper driver (which reads state at startup).
Raw observations are /franka/joint_states and /franka_gripper/joint_states.
"""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("crisp_controllers_robot_demos"))
    path = share / "launch/franka.launch.py"
    spec = importlib.util.spec_from_file_location("flash_fr3_installed_hardware_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    hardware = OpaqueFunction(
        function=module.robot_description_dependent_nodes_spawner,
        args=[
            LaunchConfiguration("robot_ip"),
            TextSubstitution(text="fr3"),
            TextSubstitution(text="false"),  # use real hardware
            TextSubstitution(text="false"),  # no fake sensor commands
            TextSubstitution(text="true"),   # keep the installed physical hand in URDF
            TextSubstitution(text=""),
            TextSubstitution(text="false"),  # no TF publisher needed for observation
        ],
    )
    gripper_config = str(Path(get_package_share_directory("franka_gripper")) /
                         "config/franka_gripper_node.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="172.16.0.2"),
        LogInfo(msg="FLASH observation: state broadcaster only; no arm commands or gripper goals."),
        hardware,
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster"], output="screen"),
        Node(package="franka_gripper", executable="franka_gripper_node",
             name="franka_gripper", output="screen",
             parameters=[{"robot_ip": LaunchConfiguration("robot_ip"),
                          "joint_names": ["fr3_finger_joint1", "fr3_finger_joint2"]},
                         gripper_config]),
    ])
