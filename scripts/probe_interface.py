#!/usr/bin/env python3
"""Probe the running robot/sim and write a Markdown interface report.

    scripts/probe_interface.py [docs/interface_sim.md]      (inside the container)

Lists nodes, topics with types, controllers and hardware interfaces, then checks every name in
the active config ($TIAGO_DC_CONFIG): topic rates, camera resolutions, joint names and TF frames.
Read-only: it never publishes.
"""

import os
import subprocess
import sys
import time
from datetime import datetime

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CameraInfo, JointState
from tf2_ros import Buffer, TransformListener


def run(*cmd: str) -> str:
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    return (result.stdout or result.stderr).strip()


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.environ["TIAGO_WS"], "docs", "interface_sim.md")
    with open(os.environ["TIAGO_DC_CONFIG"]) as f:
        cfg = yaml.safe_load(f)

    rclpy.init()
    node = Node("tiago_dc_probe")
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    start = time.time()
    while time.time() - start < 2.0:  # let DDS discovery fill the graph first
        rclpy.spin_once(node, timeout_sec=0.05)
    topic_types = dict((name, types[0]) for name, types in node.get_topic_names_and_types())
    watched = {cfg["robot"]["joint_states_topic"]: "joint states"}
    for cam, spec in cfg["cameras"].items():
        watched[spec["image_topic"]] = f"{cam} image"
        watched[spec["info_topic"]] = f"{cam} camera_info"
    counts = {topic: 0 for topic in watched}
    last = {}
    for topic in watched:
        if topic in topic_types:
            def callback(msg, t=topic):
                counts[t] += 1
                last[t] = msg
            node.create_subscription(get_message(topic_types[topic]), topic, callback, qos_profile_sensor_data)
    start = time.time()
    while time.time() - start < 5.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    elapsed = time.time() - start
    topic_types = dict((name, types[0]) for name, types in node.get_topic_names_and_types())

    lines = [f"# Interface report ({cfg['robot']['id']})", "",
             f"Generated {datetime.now().isoformat(timespec='seconds')} by scripts/probe_interface.py "
             f"with config `{os.environ['TIAGO_DC_CONFIG']}`, ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')}.", ""]

    lines += ["## Config check", "", "| stream | topic | type | rate (Hz) | detail |", "|---|---|---|---|---|"]
    for topic, label in watched.items():
        detail = ""
        msg = last.get(topic)
        if isinstance(msg, CameraInfo):
            detail = f"{msg.width}x{msg.height}, frame `{msg.header.frame_id}`, fx {msg.k[0]:.1f} fy {msg.k[4]:.1f}"
        elif msg is not None and hasattr(msg, "encoding"):
            detail = f"encoding {msg.encoding}, frame `{msg.header.frame_id}`"
        lines.append(f"| {label} | `{topic}` | {topic_types.get(topic, '**MISSING**')} | {counts[topic] / elapsed:.1f} | {detail} |")

    joint_state = last.get(cfg["robot"]["joint_states_topic"])
    names = set(joint_state.name) if isinstance(joint_state, JointState) else set()
    lines += ["", "### Joints used by the config", ""]
    for side, arm in cfg["arms"].items():
        for joint in arm["joints"] + [arm["gripper"]["joint"]]:
            lines.append(f"- {side}: `{joint}` {'ok' if joint in names else '**MISSING from /joint_states**'}")
    lines += ["", "### Controller topics", ""]
    controller_topics = [arm["controller_topic"] for arm in cfg["arms"].values()]
    controller_topics += [arm["gripper"]["controller_topic"] for arm in cfg["arms"].values()]
    controller_topics += [spec["controller_topic"] for spec in cfg.get("fixed_joints", {}).values()]
    for topic in controller_topics:
        lines.append(f"- `{topic}`: {topic_types.get(topic, '**MISSING**')}")
    lines += ["", "### TF frames", ""]
    world = cfg["robot"]["world_frame"]
    frames = [arm["tcp_frame"] for arm in cfg["arms"].values()]
    frames += [last[spec["info_topic"]].header.frame_id for spec in cfg["cameras"].values() if spec["info_topic"] in last]
    for frame in frames:
        try:
            t = tf_buffer.lookup_transform(world, frame, rclpy.time.Time()).transform.translation
            lines.append(f"- `{world}` -> `{frame}`: ({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) m")
        except Exception as exc:  # report, keep probing
            lines.append(f"- `{world}` -> `{frame}`: **{type(exc).__name__}**")
    lines += ["", "All joint names (from /joint_states):", "", ", ".join(f"`{n}`" for n in (joint_state.name if joint_state else []))]

    lines += ["", "## Controllers", "", "```", run("ros2", "control", "list_controllers"), "```",
              "", "## Hardware interfaces", "", "```", run("ros2", "control", "list_hardware_interfaces"), "```",
              "", "## Nodes", "", "```", run("ros2", "node", "list"), "```",
              "", "## Topics", "", "```", "\n".join(f"{n} [{t}]" for n, t in sorted(topic_types.items())), "```", ""]
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        f.write("\n".join(lines))
    print(f"wrote {out_path}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
