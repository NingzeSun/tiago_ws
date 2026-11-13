#!/usr/bin/env python3
"""Headless grasp episode through the real Quest protocol (for testing recorder + contact labels).

Reads the object and tray poses from Gazebo and the arm's TCP from /teleop/<side>/tcp_command,
then, like a Quest page would, holds that grip and moves the controller: the wrist turns to the
object's yaw (cube only), the TCP comes in along the gripper's approach axis, the trigger closes,
and the object is lifted, carried to the tray and released. Presses A
before and B after, so the recorder saves a normal episode.

    scripts/scripted_grasp_demo.py --side right --object can     (inside the container, sim running)
"""

import argparse
import asyncio
import json
import os
import subprocess
from pathlib import Path

import aiohttp
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

SPEED = 0.12      # m/s of the scripted hand motion
RATE = 72.0       # packets per second, like the Quest Browser


def model_pose(name: str) -> np.ndarray:
    """[x, y, z, roll, pitch, yaw] from Gazebo."""
    out = subprocess.run(["gz", "model", "-m", name, "-p"], capture_output=True, text=True, timeout=10).stdout
    return np.array([float(v) for v in out.split()[:6]])


def rot_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def matrix_to_quat_xyzw(m: np.ndarray) -> list:
    w = np.sqrt(max(1e-12, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    return [(m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w), w]


def arm_tcp(side: str) -> tuple[np.ndarray, np.ndarray]:
    """TCP position and approach axis (TCP x) of one arm in the world frame."""
    rclpy.init()
    node = Node("scripted_grasp_demo_probe")
    box = {}
    node.create_subscription(PoseStamped, f"/teleop/{side}/tcp_command", lambda m: box.setdefault("p", m.pose), 10)
    while "p" not in box:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    p, q = box["p"].position, box["p"].orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    approach = np.array([1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)])
    return np.array([p.x, p.y, p.z]), approach


async def run(args):
    cfg = yaml.safe_load(open(os.environ["TIAGO_DC_CONFIG"]))
    world_from_quest = np.asarray(cfg["teleop"]["quest_to_world"], dtype=float)
    arm = args.side
    tcp0, approach0 = arm_tcp(arm)
    object_pose, tray = model_pose(args.object), model_pose("tray")[:3]
    cube = object_pose[:3]
    # A cube is square: turn the jaws to the nearest face pair, within +/-45 deg. A can is round.
    yaw = (object_pose[5] + np.pi / 4) % (np.pi / 2) - np.pi / 4 if args.object == "cube" else 0.0
    approach = rot_z(yaw) @ approach0
    print(f"{arm} TCP {np.round(tcp0, 3)}  {args.object} {np.round(cube, 3)} yaw {np.degrees(yaw):.0f} deg  tray {np.round(tray, 3)}")
    # The pads sit ~15 mm behind the fingertip centre (TCP) along the approach axis; aim them at
    # the object's centre, but keep the fingertips off the table.
    grasp = cube + 0.015 * approach
    grasp[2] = max(grasp[2], cfg["teleop"]["workspace"]["z"][0])
    side_offset = 0.05 if arm == "left" else -0.05
    # Steps: ("yaw", angle) turns the wrist; ("trigger", v) ramps the trigger; arrays are TCP waypoints.
    plan = [
        ("yaw", yaw),
        (grasp - 0.12 * approach, 0.0),
        (grasp, 0.0),
        ("trigger", 1.0),
        (np.array([cube[0], cube[1], 1.00]), 1.0),
        (np.array([tray[0], tray[1] + side_offset, 0.98]), 1.0),
        (np.array([tray[0], tray[1] + side_offset, 0.93]), 1.0),
        ("trigger", 0.0),
        (np.array([tray[0], tray[1] + side_offset, 1.00]), 0.0),
    ]
    hand_base = {"left": np.array([-0.25, 1.1, -0.35]), "right": np.array([0.25, 1.1, -0.35])}
    url = f"ws://localhost:{cfg['quest']['http_port']}/ws?token={args.token}"
    async with aiohttp.ClientSession() as session, session.ws_connect(url) as ws:
        state = {"tcp": tcp0.copy(), "trigger": 0.0, "yaw": 0.0}

        async def packet(squeeze=1.0, press=None):
            hands = {}
            for side in ("left", "right"):
                buttons = [False] * 6
                if press and press[0] == side:
                    buttons[press[1]] = True
                offset = world_from_quest.T @ (state["tcp"] - tcp0) if side == arm else np.zeros(3)
                # A world yaw of the TCP is the hand rotation R^T Rz R in the Quest frame.
                turn = world_from_quest.T @ rot_z(state["yaw"] if side == arm else 0.0) @ world_from_quest
                hands[side] = {"pose": {"position": (hand_base[side] + offset).tolist(),
                                        "orientation": matrix_to_quat_xyzw(turn)},
                               "trigger": state["trigger"] if side == arm else 0.0,
                               "squeeze": squeeze if side == arm else 0.0, "buttons": buttons}
            await ws.send_str(json.dumps({"type": "pose", "hands": hands, "rtt_ms": None}))
            await asyncio.sleep(1.0 / RATE)

        async def hold(seconds):
            for _ in range(int(seconds * RATE)):
                await packet()

        for pressed in (True, False):
            await packet(squeeze=0.0, press=("right", 4) if pressed else None)
        await hold(0.5)
        close_step = next(i for i, (t, _) in enumerate(plan) if isinstance(t, str) and t == "trigger")
        for target, trigger in plan[:close_step] if args.approach_only else plan:
            if isinstance(target, str):
                key = target
                start = state[key]
                for alpha in np.linspace(0, 1, int((0.8 if key == "trigger" else 1.5) * RATE)):
                    state[key] = start + alpha * (trigger - start)
                    await packet()
                await hold(1.0)
                continue
            start = state["tcp"].copy()
            steps = max(int(np.linalg.norm(target - start) / SPEED * RATE), 1)
            for alpha in np.linspace(0, 1, steps):
                state["tcp"] = start + alpha * (target - start)
                await packet()
            await hold(0.6)
        for _ in range(10):
            await packet(squeeze=0.0)
        for pressed in (True, False):
            await packet(squeeze=0.0, press=("right", 5) if pressed else None)
        await asyncio.sleep(2.0)
    print(f"{args.object} now at {np.round(model_pose(args.object)[:3], 3)} (tray {np.round(model_pose('tray')[:3], 3)})")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    token_file = Path(os.environ.get("TIAGO_WS", "."), "data", ".quest_token")
    parser.add_argument("--token", default=token_file.read_text().strip() if token_file.exists() else "")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--object", choices=("cube", "can"), default="cube")
    parser.add_argument("--approach-only", action="store_true", help="stop with the open jaws around the cube")
    asyncio.run(run(parser.parse_args()))


if __name__ == "__main__":
    main()
