#!/usr/bin/env python3
"""Replay a recorded episode's commanded joints/grippers in Gazebo and report tracking error.

    scripts/replay_episode.py data/<task>/<collection>/data/episode0.hdf5     (inside the container)

Moves both arms to the episode's first command (3 s), then streams action/joint_command at the
recorded rate and compares the measured arm joints with the ones recorded. Simulation only: the
script refuses to run unless the active config has use_sim_time: true (PLAN.md hard rule 1).
Stop the teleop stack first (or keep the Quest grips released) so nothing else commands the arms.
"""

import argparse
import os
import sys
import time

import h5py
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("episode")
    args = parser.parse_args()
    with open(os.environ["TIAGO_DC_CONFIG"]) as f:
        cfg = yaml.safe_load(f)
    if not cfg.get("use_sim_time", False):
        sys.exit("refusing to replay: the active config is not a simulation config")
    with h5py.File(args.episode, "r") as f:
        command = np.asarray(f["action/joint_command"])
        recorded = np.concatenate([f["joint_action/left_arm"], f["joint_action/right_arm"]], axis=1)
        sim_t = np.asarray(f["timestamp/sim_s"])
    period = float(np.mean(np.diff(sim_t)))

    rclpy.init()
    node = Node("tiago_dc_replay", parameter_overrides=[Parameter("use_sim_time", value=True)])
    arms = {s: cfg["arms"][s] for s in ("left", "right")}
    pubs = {s: node.create_publisher(JointTrajectory, arms[s]["controller_topic"], 10) for s in arms}
    grip_pubs = {s: node.create_publisher(JointTrajectory, arms[s]["gripper"]["controller_topic"], 10) for s in arms}
    latest = {}
    node.create_subscription(JointState, cfg["robot"]["joint_states_topic"],
                             lambda m: latest.update(zip(m.name, m.position)), 10)

    def send(row, seconds):
        duration = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        for side, offset in (("left", 0), ("right", 8)):
            joints = arms[side]["joints"]
            pubs[side].publish(JointTrajectory(joint_names=joints, points=[JointTrajectoryPoint(
                positions=[float(v) for v in row[offset:offset + 7]], time_from_start=duration)]))
            g = arms[side]["gripper"]
            position = g["closed_position"] + row[offset + 7] * (g["open_position"] - g["closed_position"])
            grip_pubs[side].publish(JointTrajectory(joint_names=[g["joint"]], points=[JointTrajectoryPoint(
                positions=[float(position)], time_from_start=duration)]))

    def sleep_sim(seconds):
        end = node.get_clock().now().nanoseconds * 1e-9 + seconds
        while node.get_clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(node, timeout_sec=0.005)

    sleep_sim(1.0)
    print(f"moving to the first command, then replaying {len(command)} steps at {1 / period:.1f} Hz")
    send(command[0], 3.0)
    sleep_sim(3.5)
    names = arms["left"]["joints"] + arms["right"]["joints"]
    measured = []
    started = time.time()
    for row in command:
        send(row, 1.5 * period)
        sleep_sim(period)
        measured.append([latest[n] for n in names])
    measured = np.asarray(measured)
    error = np.abs(measured - recorded)
    print(f"replayed in {time.time() - started:.1f} s wall")
    print(f"joint tracking |replay - recorded|: mean {np.degrees(error.mean()):.2f} deg, "
          f"p95 {np.degrees(np.percentile(error, 95)):.2f} deg, max {np.degrees(error.max()):.2f} deg")
    per_joint = np.degrees(error.mean(axis=0))
    print("per joint mean (deg): " + ", ".join(f"{n.replace('arm_', '').replace('_joint', '')} {e:.2f}"
                                              for n, e in zip(names, per_joint)))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
