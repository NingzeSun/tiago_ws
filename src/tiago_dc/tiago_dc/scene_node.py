"""Gazebo only: spawn the table and task objects, and re-randomize the objects on request.

The table and objects are not in the world file because PAL's startup tuck/home motions sweep
the arms through the space in front of the robot. They are spawned after the first `ready`
event has finished. `reset_scene` on /teleop/event (right thumbstick click, or the page
button) deletes the objects and respawns them at new random poses from config `sim_scene`.
"""

from __future__ import annotations

import os
import random
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_msgs.msg import String

from .common import EVENT_TOPIC, config_path, load_config, matrix_to_quat_xyzw


def model_sdf(model: str) -> str:
    """SDF text of a model:// directory found on GAZEBO_MODEL_PATH."""
    for root in os.environ.get("GAZEBO_MODEL_PATH", "").split(":"):
        directory = Path(root) / model
        if (directory / "model.config").exists():
            name = ET.parse(directory / "model.config").getroot().findtext("sdf").strip()
            return (directory / name).read_text()
    raise FileNotFoundError(f"model {model} not on GAZEBO_MODEL_PATH")


def yaw_pose(x: float, y: float, z: float, yaw: float) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
    rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    qx, qy, qz, qw = matrix_to_quat_xyzw(rotation)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
    return pose


class SceneNode(Node):
    def __init__(self):
        super().__init__("tiago_dc_scene")
        self.cfg, _ = load_config(config_path(self))
        self.scene = self.cfg["sim_scene"]
        self.spawn = self.create_client(SpawnEntity, "/spawn_entity")
        self.delete = self.create_client(DeleteEntity, "/delete_entity")
        self.table_spawned = False
        self.create_subscription(String, EVENT_TOPIC, self.on_event, 10)
        self.rng = random.Random(self.scene.get("seed"))

    def on_event(self, msg: String):
        if msg.data == "ready" and not self.table_spawned:
            delay = float(self.cfg["ready_pose"]["duration_s"]) + 1.0
            self.get_logger().info(f"spawning table and objects in {delay:.0f} s (after the ready motion)")
            self.table_spawned = True
            self.create_timer_once(delay, self.spawn_all)
        elif msg.data == "reset_scene" and self.table_spawned:
            self.respawn_objects()

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{client.srv_name} unavailable")
            return
        client.call_async(request)

    def spawn_entity(self, name: str, model: str, pose: Pose):
        self.call(self.spawn, SpawnEntity.Request(name=name, xml=model_sdf(model), initial_pose=pose))

    def spawn_all(self):
        table = self.scene["table"]
        self.spawn_entity("table", table["model"], yaw_pose(*table["pose"]))
        self.create_timer_once(1.0, self.spawn_objects)

    def create_timer_once(self, seconds, callback):
        holder = {}

        def fire():
            holder["timer"].cancel()
            callback()

        holder["timer"] = self.create_timer(seconds, fire)

    def spawn_objects(self):
        for name, spec in self.scene["objects"].items():
            x = self.rng.uniform(*spec["x"])
            y = self.rng.uniform(*spec["y"])
            yaw = self.rng.uniform(*spec.get("yaw", [0.0, 0.0]))
            self.spawn_entity(name, spec["model"], yaw_pose(x, y, spec["z"], yaw))
        self.get_logger().info("objects spawned")

    def respawn_objects(self):
        for name in self.scene["objects"]:
            self.call(self.delete, DeleteEntity.Request(name=name))
        self.create_timer_once(1.0, self.spawn_objects)
        self.get_logger().info("resetting objects")


def main():
    rclpy.init()
    node = SceneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
