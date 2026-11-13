"""Meta Quest bridge: serves a WebXR page and turns its controller stream into ROS topics.

The Quest Browser opens http://localhost:<port>/?token=... . "localhost" is reached through
port forwarding (Quest --adb reverse--> Mac --ssh -L--> this machine), and WebXR treats
localhost as a secure origin, so no TLS, APK or PC VR runtime is needed.

Publishes /quest/{left,right}/pose (PoseStamped, WebXR local-floor frame), /quest/joy (Joy, see
common.JOY_AXES/JOY_BUTTONS) and button presses as /teleop/event:
    right A = start recording, right B = save, left X = discard, left Y = ready pose,
    right thumbstick click = reset_scene (Gazebo only).
Streams a JPEG mosaic of the cameras plus teleop/recorder status back to the page.
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
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import String

from .common import (EVENT_TOPIC, JOY_AXES, JOY_BUTTONS, QUEST_JOY_TOPIC, RECORDER_STATUS_TOPIC,
                     SIDES, TELEOP_STATUS_TOPIC, config_path, image_to_rgb, load_config,
                     quest_pose_topic)

# xr-standard gamepad: buttons[4]/[5] are A/B on the right controller and X/Y on the left.
# buttons[3] is the thumbstick click; the right one re-randomizes the objects (Gazebo only).
BUTTON_EVENTS = {("right", 4): "start", ("right", 5): "save", ("left", 4): "discard", ("left", 5): "ready",
                 ("right", 3): "reset_scene"}
WEB_EVENTS = {"start", "save", "discard", "ready", "reset_scene"}


def load_token(path: Path) -> str:
    """A short per-workspace token, stable across restarts so the Quest URL can be bookmarked."""
    if path.exists():
        return path.read_text().strip()
    path.parent.mkdir(parents=True, exist_ok=True)
    token = "".join(secrets.choice(string.ascii_lowercase + string.digits) for _ in range(8))
    path.write_text(token + "\n")
    path.chmod(0o600)
    return token


class QuestBridge(Node):
    def __init__(self):
        super().__init__("tiago_dc_quest_bridge")
        self.cfg, _ = load_config(config_path(self))
        self.quest = self.cfg["quest"]
        self.pose_pubs = {s: self.create_publisher(PoseStamped, quest_pose_topic(s), 10) for s in SIDES}
        self.joy_pub = self.create_publisher(Joy, QUEST_JOY_TOPIC, 10)
        self.event_pub = self.create_publisher(String, EVENT_TOPIC, 10)
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
            position, orientation = hand["pose"]["position"], hand["pose"]["orientation"]
            if len(position) != 3 or len(orientation) != 4 or not np.all(np.isfinite(position + orientation)):
                continue
            msg = PoseStamped()
            msg.header.stamp, msg.header.frame_id = stamp, "quest_local_floor"
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in position)
            (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w) = (float(v) for v in orientation)
            self.pose_pubs[side].publish(msg)
            axes[f"{side}_trigger"] = float(hand.get("trigger", 0.0))
            axes[f"{side}_squeeze"] = float(hand.get("squeeze", 0.0))
            pressed[f"{side}_squeeze"] = int(axes[f"{side}_squeeze"] > 0.5)
            buttons = hand.get("buttons", [])
            for key, event in BUTTON_EVENTS.items():
                if key[0] != side:
                    continue
                down = bool(buttons[key[1]]) if key[1] < len(buttons) else False
                if down and not self.previous_buttons.get(key, False):
                    self.publish_event(event)
                self.previous_buttons[key] = down
            name = {("right", 4): "a", ("right", 5): "b", ("left", 4): "x", ("left", 5): "y"}
            for index in (4, 5):
                pressed[name[(side, index)]] = int(self.previous_buttons[(side, index)])
        joy = Joy(axes=[axes[n] for n in JOY_AXES], buttons=[pressed[n] for n in JOY_BUTTONS])
        joy.header.stamp = stamp
        self.joy_pub.publish(joy)
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

    def mosaic_jpeg(self) -> bytes | None:
        """Head camera on top, wrist cameras below, with a status line (shown in the headset)."""
        w, h = self.quest["view_tile"]
        canvas = np.full((2 * h, 2 * w, 3), 24, dtype=np.uint8)
        slots = {"head_camera": (0, w // 2), "left_camera": (h, 0), "right_camera": (h, w)}
        for cam, (y, x) in slots.items():
            msg = self.images.get(cam)
            if msg is not None and cam in slots:
                canvas[y:y + h, x:x + w] = cv2.resize(image_to_rgb(msg), (w, h), interpolation=cv2.INTER_AREA)
        rec = self.status.get("recorder", {})
        teleop = self.status.get("teleop", {})
        engaged = teleop.get("engaged", {})
        text = (f"{'REC ' + str(rec.get('frames', 0)) if rec.get('recording') else 'idle'}"
                f"  next ep {rec.get('next_episode', '?')}"
                f"  L{'*' if engaged.get('left') else '-'} R{'*' if engaged.get('right') else '-'}")
        color = (230, 40, 40) if rec.get("recording") else (220, 220, 220)
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
