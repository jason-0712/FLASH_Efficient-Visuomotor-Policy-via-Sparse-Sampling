#!/usr/bin/env python3
"""Load/configure the bounded model plugin INACTIVE. Never activate or heartbeat."""
import argparse
import json
import subprocess
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import ListControllers, ListControllerTypes, ListHardwareInterfaces, UnloadController
from run_flash_fr3_pilot import call


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan",required=True,type=Path)
    args=parser.parse_args()
    plan=json.loads(args.plan.read_text())
    if (plan.get("format") != "flash_fr3_stream_plan_v1" or plan.get("controller") != "flash_fr3_stream" or
            type(plan.get("max_replacements")) is not int or plan["max_replacements"] not in (1,2)):
        raise ValueError("计划格式异常")
    rclpy.init()
    node=rclpy.create_node("flash_fr3_configure_stream_inactive")
    try:
        listing=node.create_client(ListControllers,"/controller_manager/list_controllers")
        available=call(node,listing,ListControllers.Request()).controller
        if any(c.state == "active" and c.name != "joint_state_broadcaster" for c in available):
            raise RuntimeError("存在运行中的运动控制器；不切换或卸载")
        hardware=node.create_client(ListHardwareInterfaces,"/controller_manager/list_hardware_interfaces")
        interfaces=call(node,hardware,ListHardwareInterfaces.Request())
        if any(i.is_claimed for i in interfaces.command_interfaces):
            raise RuntimeError("硬件命令接口仍被占用")
        types=node.create_client(ListControllerTypes,"/controller_manager/list_controller_types")
        if plan["plugin"] not in call(node,types,ListControllerTypes.Request()).types:
            raise RuntimeError("当前只读管理器尚未识别新插件，需要在全部命令接口未占用时重启只读 launch")
        existing=next((c for c in available if c.name == plan["controller"]),None)
        if existing:
            if existing.state != "inactive" or existing.type != plan["plugin"] or existing.claimed_interfaces:
                raise RuntimeError("既有同名控制器不处于预期的 inactive 状态")
            unloading=node.create_client(UnloadController,"/controller_manager/unload_controller")
            if not call(node,unloading,UnloadController.Request(name=plan["controller"])).ok:
                raise RuntimeError("旧实例卸载失败；不自动重试")
        subprocess.run(["ros2","run","controller_manager","spawner",plan["controller"],
                        "--controller-type",plan["plugin"],"--param-file",str(args.plan.with_name("controller_params.yaml")),
                        "--inactive","--controller-manager-timeout","10"],check=True,timeout=30)
        available=call(node,listing,ListControllers.Request()).controller
        current=next((c for c in available if c.name == plan["controller"]),None)
        interfaces=call(node,hardware,ListHardwareInterfaces.Request())
        if (current is None or current.state != "inactive" or current.claimed_interfaces or
                any(i.is_claimed for i in interfaces.command_interfaces)):
            raise RuntimeError("配置后状态不符合 inactive / 命令接口全部未占用")
        print("FLASH_STREAM_CONTROLLER="+json.dumps({"status":"CONFIGURED_INACTIVE",
              "controller":plan["controller"],"plan_id":plan["plan_id"],"robot_motion_commanded":False}),flush=True)
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
