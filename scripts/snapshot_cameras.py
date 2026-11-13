#!/usr/bin/env python3
"""Save one frame from each configured camera as PNG (quick visual check, no GUI needed).

    scripts/snapshot_cameras.py [output_dir]      (run inside the container)
"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def image_to_rgb(msg: Image) -> np.ndarray:
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}[msg.encoding]
    array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    array = array[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    if msg.encoding.startswith("bgr"):
        array = array[..., 2::-1]
    elif msg.encoding == "mono8":
        array = np.repeat(array, 3, axis=2)
    return np.ascontiguousarray(array[..., :3])


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.environ["TIAGO_WS"], "data", "snapshots")
    os.makedirs(out_dir, exist_ok=True)
    with open(os.environ["TIAGO_DC_CONFIG"]) as f:
        cameras = yaml.safe_load(f)["cameras"]
    rclpy.init()
    node = Node("snapshot_cameras", parameter_overrides=[])
    frames = {}
    for name, spec in cameras.items():
        node.create_subscription(Image, spec["image_topic"],
                                 lambda msg, n=name: frames.setdefault(n, msg), qos_profile_sensor_data)
    deadline = time.time() + 10.0
    while len(frames) < len(cameras) and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    for name, msg in frames.items():
        path = os.path.join(out_dir, f"{name}.png")
        cv2.imwrite(path, cv2.cvtColor(image_to_rgb(msg), cv2.COLOR_RGB2BGR))
        print(f"{name}: {msg.width}x{msg.height} {msg.encoding} -> {path}")
    for name in sorted(set(cameras) - set(frames)):
        print(f"{name}: no image on {cameras[name]['image_topic']}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
