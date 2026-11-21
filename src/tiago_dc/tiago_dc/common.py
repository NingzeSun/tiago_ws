"""Helpers shared by the tiago_dc nodes: config loading, message conversion, pose math."""

from __future__ import annotations

import hashlib
import os

import numpy as np
import yaml
from geometry_msgs.msg import Pose, Transform
from sensor_msgs.msg import Image

# std_msgs/String: start | stop | success | failure | discard | ready | reset_scene
EVENT_TOPIC = "/teleop/event"
COMMAND_TOPIC = "/teleop/command"      # sensor_msgs/JointState: commanded arm joints + grippers
TELEOP_STATUS_TOPIC = "/teleop/status"  # std_msgs/String (JSON)
RECORDER_STATUS_TOPIC = "/recorder/status"  # std_msgs/String (JSON)
QUEST_JOY_TOPIC = "/quest/joy"         # sensor_msgs/Joy
OPERATOR_TOPIC = "/quest/operator"     # tiago_dc_msgs/OperatorState: head, eyes, face
QUEST_FRAME = "quest_local_floor"      # headset floor frame, WebXR axes (x right, y up, z toward the operator)
SIDES = ("left", "right")


def quest_pose_topic(side: str) -> str:
    return f"/quest/{side}/pose"


def tcp_command_topic(side: str) -> str:
    return f"/teleop/{side}/tcp_command"


def gripper_command_name(side: str) -> str:
    """Name of a side's normalized gripper (1 open, 0 closed) inside COMMAND_TOPIC."""
    return f"gripper_{side}_normalized"


# sensor_msgs/Joy layout published by the Quest bridge.
JOY_AXES = ("left_trigger", "right_trigger", "left_squeeze", "right_squeeze")
JOY_BUTTONS = ("left_squeeze", "right_squeeze", "a", "b", "x", "y")


def config_path(node) -> str:
    node.declare_parameter("config", os.environ.get("TIAGO_DC_CONFIG", ""))
    path = node.get_parameter("config").value
    if not path:
        raise RuntimeError("set the `config` parameter or TIAGO_DC_CONFIG")
    return path


def load_config(path: str) -> tuple[dict, str]:
    """Return the parsed config and the sha256 of its bytes (stored with every episode)."""
    with open(path, "rb") as f:
        raw = f.read()
    return yaml.safe_load(raw), hashlib.sha256(raw).hexdigest()


def image_to_rgb(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> HxWx3 uint8 true-RGB array (the Gazebo head camera is bgr8)."""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(msg.encoding)
    if channels is None:
        raise ValueError(f"unsupported image encoding {msg.encoding}")
    array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    array = array[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    if msg.encoding.startswith("bgr"):
        array = array[..., 2::-1]
    elif msg.encoding == "mono8":
        array = np.repeat(array, 3, axis=2)
    return np.ascontiguousarray(array[..., :3])


def quat_xyzw_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat_xyzw(rotation: np.ndarray) -> np.ndarray:
    m = np.asarray(rotation, dtype=float)
    trace = np.trace(m)
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
        q = [0.0, 0.0, 0.0, (m[k, j] - m[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (m[j, i] + m[i, j]) / s
        q[k] = (m[k, i] + m[i, k]) / s
    q = np.asarray(q)
    return q / np.linalg.norm(q)


def transform_to_matrix(t: Transform) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = quat_xyzw_to_matrix([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
    out[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return out


def pose_to_matrix(p: Pose) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = quat_xyzw_to_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    out[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return out


def matrix_to_pose(m: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in m[:3, 3])
    x, y, z, w = matrix_to_quat_xyzw(m[:3, :3])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = x, y, z, w
    return pose


def matrix_to_xyz_wxyz(m: np.ndarray) -> np.ndarray:
    """RoboTwin endpose layout: [x, y, z, qw, qx, qy, qz]."""
    x, y, z, w = matrix_to_quat_xyzw(m[:3, :3])
    return np.array([*m[:3, 3], w, x, y, z], dtype=np.float64)


def pose_to_xyz_wxyz(p: Pose) -> np.ndarray:
    """A Pose in the RoboTwin endpose layout, without renormalizing."""
    return np.array([p.position.x, p.position.y, p.position.z,
                     p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z], dtype=np.float64)
