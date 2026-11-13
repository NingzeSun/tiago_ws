"""Quest bridge + teleop + recorder, all reading one config file.

    ros2 launch tiago_dc teleop.launch.py [config:=/path/to/config.yaml]

The config defaults to $TIAGO_DC_CONFIG; its top-level `use_sim_time` selects sim vs robot time,
and a `sim_scene` section (Gazebo only) adds the scene node.
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    config = LaunchConfiguration("config").perform(context)
    with open(config) as f:
        cfg = yaml.safe_load(f)
    params = [{"config": config, "use_sim_time": bool(cfg.get("use_sim_time", False))}]
    executables = [("quest_webxr_node", "quest_bridge"), ("teleop_node", "teleop"), ("recorder_node", "recorder")]
    if "sim_scene" in cfg:
        executables.append(("scene_node", "scene"))
    return [
        Node(package="tiago_dc", executable=exe, name=f"tiago_dc_{name}", parameters=params,
             output="screen", emulate_tty=True)
        for exe, name in executables
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.environ.get("TIAGO_DC_CONFIG", "")),
        OpaqueFunction(function=nodes),
    ])
