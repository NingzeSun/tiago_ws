#!/usr/bin/env python3
"""Pretend to be a Quest client: the WebSocket protocol of the Unity app (or, with --webxr, web/client.js).

Holds both grips, moves both controllers in small circles, opens and closes the triggers, sends
a head pose and (Unity) synthetic eye gaze and face weights, and (optionally) presses Y (ready),
A (start), then B (stop) and A (success) or X (failure). Tests the whole chain headlessly:
bridge -> teleop -> controllers -> recorder.

    scripts/fake_quest_client.py --record --seconds 12      (inside the container)
"""

import argparse
import asyncio
import json
import math
import os
import time
from pathlib import Path

import aiohttp


FACE_NAMES = ["BrowLowererL", "BrowLowererR", "EyesClosedL", "EyesClosedR", "JawDrop"]


def pose(position, orientation=(0.0, 0.0, 0.0, 1.0)) -> dict:
    return {"position": list(position), "orientation": list(orientation)}


def operator(t: float, webxr: bool) -> dict:
    """Head (and, like the Unity app, eyes, panel gaze point and face) at time t."""
    out = {"t": t, "head": pose([0.0, 1.6, 0.0])}
    if webxr:
        return out
    out["eyes"] = {s: {"pose": pose([x, 1.6, -0.08]), "confidence": 0.9} for s, x in (("left", -0.032), ("right", 0.032))}
    out["gaze_uv"] = [0.5 + 0.2 * math.cos(t), 0.3 + 0.2 * math.sin(t)]  # circles around the head-camera tile
    out["face"] = [0.5 + 0.5 * math.sin(t + i) for i in range(len(FACE_NAMES))]
    return out


def hand(t: float, side: str, squeeze: float, trigger: float, buttons: list[bool]) -> dict:
    sign = 1.0 if side == "right" else -1.0
    # WebXR local-floor: x right, y up, z toward the operator. 5 cm circles in the x-y plane.
    position = [0.25 * sign + 0.05 * math.cos(t), 1.1 + 0.05 * math.sin(t), -0.35]
    return {"pose": {"position": position, "orientation": [0.0, 0.0, 0.0, 1.0]},
            "trigger": trigger, "squeeze": squeeze, "buttons": buttons}


async def run(args):
    url = f"ws://{args.host}:{args.port}/ws?token={args.token}"
    statuses = []
    async with aiohttp.ClientSession() as session, session.ws_connect(url) as ws:
        async def reader():
            async for message in ws:
                if message.type == aiohttp.WSMsgType.TEXT:
                    data = json.loads(message.data)
                    if data.get("type") == "status":
                        statuses.append(data)

        reader_task = asyncio.create_task(reader())
        await ws.send_str(json.dumps({"type": "hello", "client": "webxr" if args.webxr else "fake",
                                      "face_names": [] if args.webxr else FACE_NAMES}))

        async def packet(t, squeeze=0.0, trigger=0.0, press=None):
            buttons = {"left": [False] * 6, "right": [False] * 6}
            if press:
                side, index = press
                buttons[side][index] = True
            await ws.send_str(json.dumps({"type": "pose", "rtt_ms": None, **operator(time.time() - t0, args.webxr),
                                          "hands": {s: hand(t, s, squeeze, trigger, buttons[s]) for s in ("left", "right")}}))

        async def press(side, index):
            for pressed in (True, False):
                await packet(0.0, press=(side, index) if pressed else None)
                await asyncio.sleep(0.05)

        t0 = time.time()  # headset clock
        if args.ready:
            await press("left", 5)
            print("Y pressed (ready); waiting 6 s")
            await asyncio.sleep(6.0)
        if args.record:
            await press("right", 4)
            print("A pressed (start recording)")
        start = time.time()
        while (elapsed := time.time() - start) < args.seconds:
            trigger = 0.5 - 0.5 * math.cos(elapsed * 2 * math.pi / 6.0)
            await packet(elapsed * 2 * math.pi / 4.0, squeeze=1.0, trigger=trigger)
            await asyncio.sleep(1.0 / 72.0)
        for _ in range(10):  # release the grips before saving
            await packet(0.0)
            await asyncio.sleep(0.02)
        if args.record:
            await press("right", 5)
            print("B pressed (stop)")
            await asyncio.sleep(1.0)  # the bridge maps A to success once the recorder reports `labeling`
            await press(*(("left", 4) if args.failure else ("right", 4)))
            print("X pressed (failure)" if args.failure else "A pressed (success)")
        await asyncio.sleep(3.0)
        reader_task.cancel()
    if statuses:
        last = statuses[-1]
        print("teleop:", json.dumps(last.get("teleop")))
        print("recorder:", json.dumps(last.get("recorder")))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=8080)
    default_token = Path(os.environ.get("TIAGO_WS", "."), "data", ".quest_token")
    parser.add_argument("--token", default=default_token.read_text().strip() if default_token.exists() else "")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--ready", action="store_true", help="press Y (ready pose) first")
    parser.add_argument("--record", action="store_true", help="press A before, B and A (success) after the motion")
    parser.add_argument("--failure", action="store_true", help="with --record: label the episode a failure (X)")
    parser.add_argument("--webxr", action="store_true", help="send only what the WebXR page sends (no eyes, face)")
    asyncio.run(run(parser.parse_args()))


if __name__ == "__main__":
    main()
