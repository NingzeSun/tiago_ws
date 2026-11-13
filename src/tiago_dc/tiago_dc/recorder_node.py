"""Fixed-rate episode recorder (RoboTwin HDF5 layout, see episode_writer.py).

Keeps the newest message of every stream and samples them on a timer. A step is dropped, with a
warning, when any stream's newest message is older than recorder.max_staleness_s, so every
saved step is a consistent snapshot. Episodes are controlled by /teleop/event:
start | save | discard.
"""

from __future__ import annotations

import json
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .common import (COMMAND_TOPIC, EVENT_TOPIC, RECORDER_STATUS_TOPIC, SIDES, config_path,
                     gripper_command_name, image_to_rgb, load_config, matrix_to_xyz_wxyz,
                     pose_to_matrix, tcp_command_topic, transform_to_matrix)
from .episode_writer import EpisodeBuffer, camera_matrices, encode_rgb, next_episode_index, write_episode


def stamp_s(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class RecorderNode(Node):
    def __init__(self):
        super().__init__("tiago_dc_recorder")
        self.config_file = config_path(self)
        self.cfg, self.config_sha256 = load_config(self.config_file)
        rec = self.cfg["recorder"]
        self.rec = rec
        self.world_frame = self.cfg["robot"]["world_frame"]
        self.root = Path(rec["data_root"]) / rec["task_name"] / rec["collection"]
        self.cameras = list(self.cfg["cameras"])
        self.arm_joints = {s: self.cfg["arms"][s]["joints"] for s in SIDES}
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.images: dict[str, Image] = {}
        self.infos: dict[str, CameraInfo] = {}
        self.joint_state: JointState | None = None
        self.command: JointState | None = None
        self.tcp_commands: dict[str, PoseStamped] = {}
        for cam, spec in self.cfg["cameras"].items():
            self.create_subscription(Image, spec["image_topic"],
                                     lambda m, c=cam: self.images.__setitem__(c, m), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, spec["info_topic"],
                                     lambda m, c=cam: self.infos.__setitem__(c, m), qos_profile_sensor_data)
        self.create_subscription(JointState, self.cfg["robot"]["joint_states_topic"],
                                 lambda m: setattr(self, "joint_state", m), 10)
        self.create_subscription(JointState, COMMAND_TOPIC, lambda m: setattr(self, "command", m), 10)
        for side in SIDES:
            self.create_subscription(PoseStamped, tcp_command_topic(side),
                                     lambda m, s=side: self.tcp_commands.__setitem__(s, m), 10)
        self.create_subscription(String, EVENT_TOPIC, self.on_event, 10)
        self.status_pub = self.create_publisher(String, RECORDER_STATUS_TOPIC, 10)

        self.buffer: EpisodeBuffer | None = None
        self.joint_names: list[str] | None = None
        self.dropped = 0
        self.drop_reasons: dict[str, int] = {}
        self.started_wall = 0.0
        self.saving = 0
        self.last_saved = ""
        self.last_error = ""
        self.lock = threading.Lock()
        self.save_lock = threading.Lock()
        self.create_timer(1.0 / rec["rate_hz"], self.sample)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(f"recording to {self.root}")

    # ---------------------------------------------------------------- episode control
    def on_event(self, msg: String):
        if msg.data == "start" and self.buffer is None:
            if self.joint_state is None:
                self.last_error = "no /joint_states yet"
                return
            self.joint_names = list(self.joint_state.name)
            self.buffer = EpisodeBuffer(self.cameras, self.joint_names)
            self.dropped, self.drop_reasons, self.last_error = 0, {}, ""
            self.started_wall = time.time()
            self.get_logger().info("recording started")
        elif msg.data == "save" and self.buffer is not None:
            buffer, self.buffer = self.buffer, None
            if len(buffer) < 2:
                self.last_error = f"episode too short ({len(buffer)} steps), not saved"
                self.get_logger().warn(self.last_error)
                return
            meta = self.episode_meta(buffer)
            with self.lock:
                self.saving += 1
            threading.Thread(target=self.save, args=(buffer, meta), daemon=True).start()
        elif msg.data == "discard" and self.buffer is not None:
            self.get_logger().info(f"discarded episode with {len(self.buffer)} steps")
            self.buffer = None

    def episode_meta(self, buffer: EpisodeBuffer) -> dict:
        sim = buffer.stacked("sim_s")
        return {
            "robot_id": self.cfg["robot"]["id"],
            "task_name": self.rec["task_name"],
            "instruction": self.rec["instruction"],
            "collection": self.rec["collection"],
            "teleop": "meta_quest_webxr",
            "operator_marked_success": True,
            "rate_hz": self.rec["rate_hz"],
            "frames": len(buffer),
            "duration_sim_s": float(sim[-1] - sim[0]),
            "dropped_steps": self.dropped,
            "drop_reasons": self.drop_reasons,
            "cameras": {c: self.cfg["cameras"][c] for c in self.cameras},
            "world_frame": self.world_frame,
            "tcp_frames": {s: self.cfg["arms"][s]["tcp_frame"] for s in SIDES},
            "config_file": self.config_file,
            "config_sha256": self.config_sha256,
            "created_utc": datetime.fromtimestamp(self.started_wall, timezone.utc).isoformat(),
        }

    def save(self, buffer: EpisodeBuffer, meta: dict):
        try:
            with self.save_lock:  # one writer at a time, so episode indices never collide
                path = write_episode(self.root, buffer, self.rec["instruction"], meta)
            self.last_saved = str(path)
            self.get_logger().info(f"saved {path} ({len(buffer)} steps, {meta['dropped_steps']} dropped)")
        except Exception as exc:  # report to the operator instead of killing the node
            self.last_error = f"save failed: {exc}"
            self.get_logger().error(self.last_error)
        finally:
            with self.lock:
                self.saving -= 1

    # ---------------------------------------------------------------- sampling
    def drop(self, reason: str):
        self.dropped += 1
        self.drop_reasons[reason] = self.drop_reasons.get(reason, 0) + 1
        self.get_logger().warn(f"dropped step: {reason}", throttle_duration_sec=2.0)

    def lookup(self, frame: str, stamp) -> np.ndarray:
        """world <- frame at `stamp`, falling back to the newest transform."""
        try:
            t = self.tf_buffer.lookup_transform(self.world_frame, frame, Time.from_msg(stamp))
        except TransformException:
            t = self.tf_buffer.lookup_transform(self.world_frame, frame, Time())
        return transform_to_matrix(t.transform)

    def sample(self):
        if self.buffer is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        limit = self.rec["max_staleness_s"]
        streams = {f"image:{c}": self.images.get(c) for c in self.cameras}
        streams.update({f"camera_info:{c}": self.infos.get(c) for c in self.cameras})
        streams.update({"joint_states": self.joint_state, "command": self.command})
        streams.update({f"tcp_command:{s}": self.tcp_commands.get(s) for s in SIDES})
        for name, msg in streams.items():
            if msg is None:
                return self.drop(f"{name} missing")
            if not name.startswith("camera_info") and now - stamp_s(msg.header) > limit:
                return self.drop(f"{name} stale")

        js, cmd = self.joint_state, self.command
        positions = dict(zip(js.name, js.position))
        velocities = dict(zip(js.name, js.velocity)) if js.velocity else {}
        commanded = dict(zip(cmd.name, cmd.position))
        step = {"rgb": {}, "intrinsic_cv": {}, "extrinsic_cv": {}, "cam2world_gl": {}}
        try:
            for cam in self.cameras:
                image = self.images[cam]
                rgb = image_to_rgb(image)
                world_from_optical = self.lookup(image.header.frame_id, image.header.stamp)
                K, extrinsic, cam2world = camera_matrices(np.array(self.infos[cam].k), world_from_optical)
                step["rgb"][cam] = encode_rgb(rgb, self.rec["jpeg_quality"])
                step["intrinsic_cv"][cam], step["extrinsic_cv"][cam], step["cam2world_gl"][cam] = K, extrinsic, cam2world
            for side in SIDES:
                arm = self.cfg["arms"][side]
                step[f"{side}_arm"] = np.array([positions[j] for j in self.arm_joints[side]])
                step[f"{side}_gripper"] = float(commanded[gripper_command_name(side)])
                step[f"{side}_endpose"] = matrix_to_xyz_wxyz(self.lookup(arm["tcp_frame"], js.header.stamp))
                step[f"{side}_tcp_command"] = matrix_to_xyz_wxyz(pose_to_matrix(self.tcp_commands[side].pose))
        except (TransformException, KeyError) as exc:
            return self.drop(f"{type(exc).__name__}: {exc}")

        step["joint_command"] = np.array(
            [commanded[j] for j in self.arm_joints["left"]] + [commanded[gripper_command_name("left")]]
            + [commanded[j] for j in self.arm_joints["right"]] + [commanded[gripper_command_name("right")]])
        step["joint_position"] = np.array([positions.get(j, np.nan) for j in self.joint_names])
        step["joint_velocity"] = np.array([velocities.get(j, np.nan) for j in self.joint_names])
        grippers = []
        for side in SIDES:
            g = self.cfg["arms"][side]["gripper"]
            grippers.append((positions[g["joint"]] - g["closed_position"]) / (g["open_position"] - g["closed_position"]))
        step["gripper_measured"] = np.array(grippers)
        step["sim_s"] = now
        step["wall_s"] = time.time()
        self.buffer.append(step)

    def publish_status(self):
        status = {
            "recording": self.buffer is not None,
            "frames": len(self.buffer) if self.buffer is not None else 0,
            "dropped": self.dropped,
            "saving": self.saving > 0,
            "last_saved": self.last_saved,
            "next_episode": next_episode_index(self.root),
            "task": self.rec["task_name"],
            "instruction": self.rec["instruction"],
            "error": self.last_error,
        }
        self.status_pub.publish(String(data=json.dumps(status)))


def main():
    rclpy.init()
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
