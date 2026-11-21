"""Meta Quest bridge: serves the WebXR page and the WebSocket that both Quest clients use.

Clients: the WebXR page (tiago_dc/web, any Quest Browser) and the Unity app
(quest_unity/QuestTeleopClient.cs, Quest Pro: adds eye gaze and face expressions). The Quest
Browser opens http://localhost:<port>/?token=... ; "localhost" is reached through adb reverse
(and ssh -L), and WebXR treats localhost as a secure origin, so no TLS is needed.

Publishes /quest/{left,right}/pose (PoseStamped, WebXR local-floor frame), /quest/joy (Joy, see
common.JOY_AXES/JOY_BUTTONS), /quest/operator (OperatorState: head, eyes, face) and button
presses as /teleop/event:
    right A = start recording (mark success after B), right B = stop, left X = mark failure,
    left stick click = discard, left Y = ready pose, right stick click = reset_scene (Gazebo only).
Streams a JPEG mosaic of the cameras plus teleop/recorder status back to the client.
"""

from __future__ import annotations

import asyncio
import json
import secrets
import string
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from aiohttp import WSMsgType, web
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import String
from tiago_dc_msgs.msg import OperatorState

from .common import (EVENT_TOPIC, JOY_AXES, JOY_BUTTONS, OPERATOR_TOPIC, QUEST_FRAME, QUEST_JOY_TOPIC,
                     RECORDER_STATUS_TOPIC, SIDES, TELEOP_STATUS_TOPIC, config_path, image_to_rgb,
                     load_config, quest_pose_topic)

# xr-standard gamepad: buttons[4]/[5] are A/B on the right controller and X/Y on the left,
# buttons[3] is the thumbstick click. A marks success instead while the recorder awaits a label.
BUTTON_EVENTS = {("right", 4): "start", ("right", 5): "stop", ("left", 4): "failure", ("left", 5): "ready",
                 ("left", 3): "discard", ("right", 3): "reset_scene"}
WEB_EVENTS = {"start", "stop", "success", "failure", "discard", "ready", "reset_scene"}
# Mosaic tiles as (row, column) in tile units: head camera on top, wrist cameras below.
MOSAIC_TILES = {"head_camera": (0, 0.5), "left_camera": (1, 0), "right_camera": (1, 1)}


def load_token(path: Path) -> str:
    """A short per-workspace token, stable across restarts so the Quest URL can be bookmarked."""
    if path.exists():
        return path.read_text().strip()
    path.parent.mkdir(parents=True, exist_ok=True)
    token = "".join(secrets.choice(string.ascii_lowercase + string.digits) for _ in range(8))
    path.write_text(token + "\n")
    path.chmod(0o600)
    return token


def pose_from_packet(pose: dict | None) -> Pose | None:
    """{"position": [x, y, z], "orientation": [x, y, z, w]} -> Pose, or None if absent or malformed."""
    if not pose:
        return None
    position, orientation = pose.get("position", []), pose.get("orientation", [])
    if len(position) != 3 or len(orientation) != 4 or not np.all(np.isfinite(position + orientation)):
        return None
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = (float(v) for v in position)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = (float(v) for v in orientation)
    return msg


class QuestBridge(Node):
    def __init__(self):
        super().__init__("tiago_dc_quest_bridge")
        self.cfg, _ = load_config(config_path(self))
        self.quest = self.cfg["quest"]
        self.pose_pubs = {s: self.create_publisher(PoseStamped, quest_pose_topic(s), 10) for s in SIDES}
        self.joy_pub = self.create_publisher(Joy, QUEST_JOY_TOPIC, 10)
        self.event_pub = self.create_publisher(String, EVENT_TOPIC, 10)
        self.operator_pub = self.create_publisher(OperatorState, OPERATOR_TOPIC, 10)
        self.client = ""
        self.face_names: list[str] = []
        self.images: dict[str, Image] = {}
        for cam in self.quest["view_cameras"]:
            topic = self.cfg["cameras"][cam]["image_topic"]
            self.create_subscription(Image, topic, lambda m, c=cam: self.images.__setitem__(c, m),
                                     qos_profile_sensor_data)
        self.status = {"teleop": {}, "recorder": {}}
        self.create_subscription(String, TELEOP_STATUS_TOPIC,
                                 lambda m: self.status.__setitem__("teleop", json.loads(m.data)), 10)
        self.create_subscription(String, RECORDER_STATUS_TOPIC,
                                 lambda m: self.status.__setitem__("recorder", json.loads(m.data)), 10)
        self.previous_buttons: dict[tuple[str, int], bool] = {}
        self.rtt_ms: float | None = None
        self.packets = 0
        self.last_rate_log = time.time()

    def publish_event(self, name: str):
        self.event_pub.publish(String(data=name))
        self.get_logger().info(f"event: {name}")

    def handle_hello(self, packet: dict):
        """First packet of a client: its name and, for the Unity app, the face expression names."""
        self.client = str(packet.get("client", ""))
        self.face_names = [str(n) for n in packet.get("face_names") or []]
        self.get_logger().info(f"client: {self.client or '?'}, {len(self.face_names)} face expressions")

    def button_event(self, key: tuple[str, int]) -> str:
        if key == ("right", 4) and self.status["recorder"].get("state") == "labeling":
            return "success"
        return BUTTON_EVENTS[key]

    def gaze_target(self, uv) -> tuple[str, list[float]] | None:
        """Gaze point on the headset panel (u right, v down, 0..1) -> (camera, pixel in its image)."""
        if len(uv) != 2 or not np.all(np.isfinite(uv)):
            return None
        x, y = 2.0 * float(uv[0]), 2.0 * float(uv[1])  # in tile units; the mosaic is 2 x 2 tiles
        for cam, (row, col) in MOSAIC_TILES.items():
            image = self.images.get(cam)
            if image is not None and col <= x < col + 1 and row <= y < row + 1:
                return cam, [(x - col) * image.width, (y - row) * image.height]
        return None

    def publish_operator(self, packet: dict, stamp):
        msg = OperatorState()
        msg.header.stamp, msg.header.frame_id = stamp, QUEST_FRAME
        msg.client, msg.face_names = self.client, self.face_names
        msg.headset_time = float(packet.get("t") or 0.0)
        if (head := pose_from_packet(packet.get("head"))) is not None:
            msg.head_valid, msg.head = True, head
        eyes = packet.get("eyes") or {}
        left, right = (pose_from_packet((eyes.get(s) or {}).get("pose")) for s in SIDES)
        if left is not None and right is not None:
            msg.eyes_valid, msg.left_eye, msg.right_eye = True, left, right
            msg.left_eye_confidence = float(eyes["left"].get("confidence", 0.0))
            msg.right_eye_confidence = float(eyes["right"].get("confidence", 0.0))
        if packet.get("gaze_uv") and (target := self.gaze_target(packet["gaze_uv"])) is not None:
            msg.gaze_valid, (msg.gaze_camera, msg.gaze_pixel) = True, target
        face = packet.get("face")
        if face and len(face) == len(self.face_names) and np.all(np.isfinite(face)):
            msg.face_valid, msg.face_weights = True, [float(v) for v in face]
        self.operator_pub.publish(msg)

    def handle_pose_packet(self, packet: dict):
        stamp = self.get_clock().now().to_msg()
        hands = packet.get("hands") or {}
        axes = {name: 0.0 for name in JOY_AXES}
        pressed = {name: 0 for name in JOY_BUTTONS}
        for side in SIDES:
            hand = hands.get(side)
            if not hand:
                for key in BUTTON_EVENTS:
                    if key[0] == side:
                        self.previous_buttons[key] = False
                continue
            pose = pose_from_packet(hand.get("pose"))
            if pose is None:
                continue
            msg = PoseStamped(pose=pose)
            msg.header.stamp, msg.header.frame_id = stamp, QUEST_FRAME
            self.pose_pubs[side].publish(msg)
            axes[f"{side}_trigger"] = float(hand.get("trigger", 0.0))
            axes[f"{side}_squeeze"] = float(hand.get("squeeze", 0.0))
            pressed[f"{side}_squeeze"] = int(axes[f"{side}_squeeze"] > 0.5)
            buttons = hand.get("buttons", [])
            for key in BUTTON_EVENTS:
                if key[0] != side:
                    continue
                down = bool(buttons[key[1]]) if key[1] < len(buttons) else False
                if down and not self.previous_buttons.get(key, False):
                    self.publish_event(self.button_event(key))
                self.previous_buttons[key] = down
            name = {("right", 4): "a", ("right", 5): "b", ("left", 4): "x", ("left", 5): "y"}
            for index in (4, 5):
                pressed[name[(side, index)]] = int(self.previous_buttons[(side, index)])
        joy = Joy(axes=[axes[n] for n in JOY_AXES], buttons=[pressed[n] for n in JOY_BUTTONS])
        joy.header.stamp = stamp
        self.joy_pub.publish(joy)
        self.publish_operator(packet, stamp)
        if packet.get("rtt_ms") is not None:
            self.rtt_ms = float(packet["rtt_ms"])
        self.packets += 1
        if time.time() - self.last_rate_log > 10.0:
            rate = self.packets / (time.time() - self.last_rate_log)
            self.get_logger().info(f"quest input {rate:.0f} Hz, round trip {self.rtt_ms} ms")
            self.packets, self.last_rate_log = 0, time.time()

    def release(self):
        """Client gone: publish an all-released Joy so both clutches open immediately."""
        joy = Joy(axes=[0.0] * len(JOY_AXES), buttons=[0] * len(JOY_BUTTONS))
        joy.header.stamp = self.get_clock().now().to_msg()
        self.joy_pub.publish(joy)
        self.previous_buttons.clear()
        self.client, self.face_names = "", []

    def mosaic_jpeg(self) -> bytes | None:
        """Camera tiles (MOSAIC_TILES) with a status line, shown in the headset."""
        w, h = self.quest["view_tile"]
        canvas = np.full((2 * h, 2 * w, 3), 24, dtype=np.uint8)
        for cam, (row, col) in MOSAIC_TILES.items():
            msg = self.images.get(cam)
            if msg is not None:
                y, x = int(row * h), int(col * w)
                canvas[y:y + h, x:x + w] = cv2.resize(image_to_rgb(msg), (w, h), interpolation=cv2.INTER_AREA)
        rec = self.status.get("recorder", {})
        teleop = self.status.get("teleop", {})
        engaged = teleop.get("engaged", {})
        saved = rec.get("saved", {})
        state = rec.get("state", "idle")
        label = {"recording": f"REC {rec.get('frames', 0)}", "labeling": "A success / X failure"}.get(state, "idle")
        text = (f"{label}  saved {saved.get('success', '?')} ok {saved.get('failure', '?')} failed"
                f"  L{'*' if engaged.get('left') else '-'} R{'*' if engaged.get('right') else '-'}")
        color = {"recording": (230, 40, 40), "labeling": (240, 200, 40)}.get(state, (220, 220, 220))
        cv2.putText(canvas, text, (12, 2 * h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)
        ok, jpeg = cv2.imencode(".jpg", cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 75])
        return jpeg.tobytes() if ok else None


async def serve(node: QuestBridge):
    web_dir = Path(get_package_share_directory("tiago_dc")) / "web"
    token = load_token(Path(node.cfg["recorder"]["data_root"]) / ".quest_token")
    host, port = node.quest.get("http_host", "127.0.0.1"), int(node.quest["http_port"])
    active: dict[str, web.WebSocketResponse | None] = {"ws": None}
    loop = asyncio.get_running_loop()

    async def index(_request):
        return web.FileResponse(web_dir / "index.html")

    async def client_js(_request):
        return web.FileResponse(web_dir / "client.js")

    async def stream(ws: web.WebSocketResponse):
        period = 1.0 / float(node.quest["view_fps"])
        while not ws.closed:
            started = time.monotonic()
            jpeg = await loop.run_in_executor(None, node.mosaic_jpeg)
            status = {"type": "status", **node.status, "rtt_ms": node.rtt_ms}
            try:
                await ws.send_str(json.dumps(status))
                if jpeg:
                    await ws.send_bytes(jpeg)
            except (ConnectionResetError, RuntimeError):
                return
            await asyncio.sleep(max(0.0, period - (time.monotonic() - started)))

    async def websocket(request):
        if request.query.get("token") != token:
            raise web.HTTPForbidden(text="bad token")
        ws = web.WebSocketResponse(heartbeat=5.0, max_msg_size=1 << 20)
        await ws.prepare(request)
        previous, active["ws"] = active["ws"], ws
        if previous is not None and not previous.closed:
            await previous.send_str(json.dumps({"type": "error", "message": "another client took control"}))
            await previous.close()
        node.get_logger().info(f"client connected from {request.remote}")
        sender = asyncio.create_task(stream(ws))
        try:
            async for message in ws:
                if message.type != WSMsgType.TEXT:
                    continue
                packet = json.loads(message.data)
                kind = packet.get("type")
                if kind == "pose":
                    node.handle_pose_packet(packet)
                elif kind == "hello":
                    node.handle_hello(packet)
                elif kind == "event" and packet.get("name") in WEB_EVENTS:
                    node.publish_event(packet["name"])
                elif kind == "ping":
                    await ws.send_str(json.dumps({"type": "pong", "t": packet.get("t")}))
        finally:
            sender.cancel()
            if active["ws"] is ws:
                active["ws"] = None
                node.release()
            node.get_logger().info("client disconnected")
        return ws

    app = web.Application()
    app.add_routes([web.get("/", index), web.get("/client.js", client_js), web.get("/ws", websocket)])
    runner = web.AppRunner(app, access_log=None)
    await runner.setup()
    await web.TCPSite(runner, host, port).start()
    url = f"http://localhost:{port}/?token={token}"
    Path(node.cfg["recorder"]["data_root"], "quest_url.txt").write_text(url + "\n")
    node.get_logger().info(f"Quest page: {url}  (listening on {host}:{port})")
    await asyncio.Event().wait()


def main():
    rclpy.init()
    node = QuestBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        asyncio.run(serve(node))
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
