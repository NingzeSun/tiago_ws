"""Quest controller poses -> TIAGo Pro arm joint commands.

Per arm, the grip (squeeze) button is a clutch and deadman: while held, the TCP target follows
the controller's motion relative to where both were when the grip closed; on release, commands
stop and the arm holds. The trigger closes the gripper. Targets are clipped to the configured
workspace and speed limits, then solved with damped-least-squares IK and sent as single-point
joint trajectories. Every tick also publishes the commanded joints/grippers and commanded TCP
poses, which the recorder stores as the action.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field

import numpy as np
import pinocchio as pin
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .common import (COMMAND_TOPIC, EVENT_TOPIC, JOY_AXES, JOY_BUTTONS, QUEST_JOY_TOPIC, SIDES,
                     TELEOP_STATUS_TOPIC, config_path, gripper_command_name, load_config,
                     matrix_to_pose, pose_to_matrix, quest_pose_topic, tcp_command_topic)
from .ik import ArmIK

# A solve leaving more than this TCP error means the target left the reachable set: the clutch
# re-anchors at the reached pose so the operator never has to "unwind" a dead zone.
REANCHOR_ERROR = 0.05


def duration_msg(seconds: float) -> DurationMsg:
    return DurationMsg(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))


@dataclass
class ArmState:
    joints: list[str]
    ik: ArmIK | None = None
    q_cmd: np.ndarray | None = None
    engaged: bool = False
    controller_anchor: np.ndarray | None = None
    tcp_anchor: np.ndarray | None = None
    target: np.ndarray | None = None
    ik_error: float = 0.0
    gripper: float = 1.0
    gripper_sent: float | None = None
    gripper_sent_time: float = 0.0
    quest_pose: np.ndarray | None = None
    quest_time: float = -1.0
    extra: dict = field(default_factory=dict)


class TeleopNode(Node):
    def __init__(self):
        super().__init__("tiago_dc_teleop")
        self.cfg, _ = load_config(config_path(self))
        teleop = self.cfg["teleop"]
        self.teleop = teleop
        self.dt = 1.0 / teleop["rate_hz"]
        self.world_frame = self.cfg["robot"]["world_frame"]
        self.quest_to_world = np.asarray(teleop["quest_to_world"], dtype=float)
        self.enable_motion = bool(teleop.get("enable_motion", False))
        self.workspace = np.array([teleop["workspace"][axis] for axis in "xyz"], dtype=float)
        self.arms = {side: ArmState(joints=self.cfg["arms"][side]["joints"]) for side in SIDES}
        self.joint_positions: dict[str, float] = {}
        self.joy: Joy | None = None
        self.joy_time = -1.0
        self.ready_until = -1.0
        self.ready_on_start = bool(teleop.get("ready_on_start", False))
        self.status_tick = 0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, self.cfg["robot"]["robot_description_topic"],
                                 self.on_robot_description, latched)
        self.create_subscription(JointState, self.cfg["robot"]["joint_states_topic"], self.on_joint_states, 10)
        self.create_subscription(Joy, QUEST_JOY_TOPIC, self.on_joy, 10)
        self.create_subscription(String, EVENT_TOPIC, self.on_event, 10)
        for side in SIDES:
            self.create_subscription(PoseStamped, quest_pose_topic(side),
                                     lambda msg, s=side: self.on_quest_pose(s, msg), 10)

        self.arm_pubs = {s: self.create_publisher(JointTrajectory, self.cfg["arms"][s]["controller_topic"], 10)
                         for s in SIDES}
        self.gripper_pubs = {s: self.create_publisher(JointTrajectory, self.cfg["arms"][s]["gripper"]["controller_topic"], 10)
                             for s in SIDES}
        self.fixed_pubs = {name: self.create_publisher(JointTrajectory, spec["controller_topic"], 10)
                           for name, spec in self.cfg.get("fixed_joints", {}).items()}
        self.command_pub = self.create_publisher(JointState, COMMAND_TOPIC, 10)
        self.tcp_pubs = {s: self.create_publisher(PoseStamped, tcp_command_topic(s), 10) for s in SIDES}
        self.status_pub = self.create_publisher(String, TELEOP_STATUS_TOPIC, 10)
        self.event_pub = self.create_publisher(String, EVENT_TOPIC, 10)
        self.create_timer(self.dt, self.tick)
        if not self.enable_motion:
            self.get_logger().warn("teleop.enable_motion is false: commands are computed and published "
                                   "on /teleop/command but NOT sent to the controllers")

    # ---------------------------------------------------------------- inputs
    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_robot_description(self, msg: String):
        if self.arms["left"].ik is not None:
            return
        for side in SIDES:
            arm_cfg = self.cfg["arms"][side]
            self.arms[side].ik = ArmIK(msg.data, arm_cfg["joints"], arm_cfg["tcp_frame"], self.world_frame,
                                       self.teleop["joint_limit_margin"], self.teleop["ik_damping"])
        self.get_logger().info("IK models built from robot_description")

    def on_joint_states(self, msg: JointState):
        self.joint_positions.update(zip(msg.name, msg.position))

    def on_joy(self, msg: Joy):
        self.joy, self.joy_time = msg, self.now_s()

    def on_quest_pose(self, side: str, msg: PoseStamped):
        arm = self.arms[side]
        arm.quest_pose, arm.quest_time = pose_to_matrix(msg.pose), self.now_s()

    def on_event(self, msg: String):
        if msg.data == "ready":
            self.go_ready()

    def joy_value(self, kind: str, name: str) -> float:
        if self.joy is None or self.now_s() - self.joy_time > self.teleop["input_timeout_s"]:
            return 0.0
        values, names = (self.joy.axes, JOY_AXES) if kind == "axis" else (self.joy.buttons, JOY_BUTTONS)
        index = names.index(name)
        return float(values[index]) if index < len(values) else 0.0

    # ---------------------------------------------------------------- outputs
    def send_trajectory(self, publisher, joints, positions, seconds: float):
        if not self.enable_motion:
            return
        point = JointTrajectoryPoint(positions=[float(p) for p in positions],
                                     time_from_start=duration_msg(seconds))
        publisher.publish(JointTrajectory(joint_names=list(joints), points=[point]))

    def send_gripper(self, side: str, value: float, seconds: float):
        spec = self.cfg["arms"][side]["gripper"]
        position = spec["closed_position"] + value * (spec["open_position"] - spec["closed_position"])
        self.send_trajectory(self.gripper_pubs[side], [spec["joint"]], [position], seconds)
        arm = self.arms[side]
        arm.gripper_sent, arm.gripper_sent_time = value, self.now_s()

    def go_ready(self):
        """Fixed joints and arms to the configured ready pose; teleop pauses until it is done."""
        duration = float(self.cfg["ready_pose"]["duration_s"])
        for name, spec in self.cfg.get("fixed_joints", {}).items():
            joints, positions = zip(*spec["positions"].items())
            self.send_trajectory(self.fixed_pubs[name], joints, positions, duration)
        for side in SIDES:
            arm = self.arms[side]
            arm.engaged = False
            arm.gripper = 1.0
            self.send_trajectory(self.arm_pubs[side], arm.joints, self.cfg["ready_pose"][side], duration)
            self.send_gripper(side, 1.0, 1.0)
        self.ready_until = self.now_s() + duration + 0.5
        self.get_logger().info(f"moving to ready pose ({duration:.1f} s)")

    # ---------------------------------------------------------------- control loop
    def measured(self, arm: ArmState) -> np.ndarray | None:
        try:
            return np.array([self.joint_positions[j] for j in arm.joints])
        except KeyError:
            return None

    def limit_target(self, arm: ArmState, desired: np.ndarray) -> np.ndarray:
        """Clip to the workspace box, then cap the target's linear and angular speed."""
        desired = desired.copy()
        desired[:3, 3] = np.clip(desired[:3, 3], self.workspace[:, 0], self.workspace[:, 1])
        target = arm.target.copy()
        step = desired[:3, 3] - target[:3, 3]
        max_lin = self.teleop["max_linear_speed"] * self.dt
        norm = np.linalg.norm(step)
        target[:3, 3] += step if norm <= max_lin else step * (max_lin / norm)
        rotvec = pin.log3(desired[:3, :3] @ target[:3, :3].T)
        max_ang = self.teleop["max_angular_speed"] * self.dt
        angle = np.linalg.norm(rotvec)
        if angle > max_ang:
            rotvec *= max_ang / angle
        target[:3, :3] = pin.exp3(rotvec) @ target[:3, :3]
        return target

    def step_arm(self, side: str, now: float):
        arm = self.arms[side]
        measured = self.measured(arm)
        if measured is None:
            return
        arm.ik.set_joint_positions(self.joint_positions)
        fresh = arm.quest_pose is not None and now - arm.quest_time <= self.teleop["input_timeout_s"]
        clutch = self.joy_value("button", f"{side}_squeeze") > 0.5 and fresh and self.enable_motion

        if not clutch:
            arm.engaged = False
            arm.q_cmd = measured
            arm.target = arm.ik.fk(measured).homogeneous
            arm.ik_error = 0.0
        else:
            if not arm.engaged:
                arm.engaged = True
                arm.controller_anchor = arm.quest_pose.copy()
                arm.tcp_anchor = arm.ik.fk(arm.q_cmd).homogeneous
                arm.target = arm.tcp_anchor.copy()
            r = self.quest_to_world
            delta_p = r @ (arm.quest_pose[:3, 3] - arm.controller_anchor[:3, 3]) * self.teleop["translation_scale"]
            delta_r = r @ arm.quest_pose[:3, :3] @ arm.controller_anchor[:3, :3].T @ r.T
            desired = np.eye(4)
            desired[:3, 3] = arm.tcp_anchor[:3, 3] + delta_p
            desired[:3, :3] = delta_r @ arm.tcp_anchor[:3, :3]
            arm.target = self.limit_target(arm, desired)
            q_new, arm.ik_error = arm.ik.solve(pin.SE3(arm.target), arm.q_cmd, self.teleop["ik_iterations"],
                                               max_step=self.teleop["max_joint_speed"] * self.dt)
            arm.q_cmd = q_new
            if arm.ik_error > REANCHOR_ERROR:
                arm.target = arm.ik.fk(q_new).homogeneous
                arm.controller_anchor = arm.quest_pose.copy()
                arm.tcp_anchor = arm.target.copy()
            self.send_trajectory(self.arm_pubs[side], arm.joints, arm.q_cmd, self.teleop["command_horizon_s"])

        # Gripper: trigger pressed = close. Rate-limited; while the Quest is live it is resent on
        # change or every 0.5 s. Without Quest input nothing is sent, so other tools can command it.
        if self.joy is not None and now - self.joy_time <= self.teleop["input_timeout_s"]:
            desired = 1.0 - min(max(self.joy_value("axis", f"{side}_trigger"), 0.0), 1.0)
            max_step = self.teleop["gripper_rate"] * self.dt
            arm.gripper += float(np.clip(desired - arm.gripper, -max_step, max_step))
            if (arm.gripper_sent is None or abs(arm.gripper - arm.gripper_sent) > 0.005
                    or now - arm.gripper_sent_time > 0.5):
                self.send_gripper(side, arm.gripper, max(self.teleop["command_horizon_s"], 0.1))

    def tick(self):
        if self.arms["left"].ik is None or not self.joint_positions:
            return
        if self.ready_on_start:
            # Published as an event (handled by on_event) so the sim scene node sees it too.
            self.ready_on_start = False
            self.event_pub.publish(String(data="ready"))
        now = self.now_s()
        if now < self.ready_until:
            for arm in self.arms.values():
                arm.engaged = False
                arm.q_cmd = self.measured(arm)
                if arm.q_cmd is not None:
                    arm.target = arm.ik.fk(arm.q_cmd).homogeneous
        else:
            for side in SIDES:
                self.step_arm(side, now)
        self.publish_command()

    def publish_command(self):
        if any(arm.q_cmd is None or arm.target is None for arm in self.arms.values()):
            return
        stamp = self.get_clock().now().to_msg()
        names, positions = [], []
        for side in SIDES:
            names += self.arms[side].joints
            positions += [float(v) for v in self.arms[side].q_cmd]
        for side in SIDES:
            names.append(gripper_command_name(side))
            positions.append(float(self.arms[side].gripper))
        msg = JointState(name=names, position=positions)
        msg.header.stamp = stamp
        self.command_pub.publish(msg)
        for side in SIDES:
            pose = PoseStamped(pose=matrix_to_pose(self.arms[side].ik.fk(self.arms[side].q_cmd).homogeneous))
            pose.header.stamp, pose.header.frame_id = stamp, self.world_frame
            self.tcp_pubs[side].publish(pose)
        self.status_tick += 1
        if self.status_tick % 10 == 0:
            status = {
                "engaged": {s: self.arms[s].engaged for s in SIDES},
                "ik_error": {s: round(self.arms[s].ik_error, 4) for s in SIDES},
                "gripper": {s: round(self.arms[s].gripper, 2) for s in SIDES},
                "ready_motion": self.now_s() < self.ready_until,
                "motion_enabled": self.enable_motion,
            }
            self.status_pub.publish(String(data=json.dumps(status)))


def main():
    rclpy.init()
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
