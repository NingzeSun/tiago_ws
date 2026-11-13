#!/usr/bin/env python3
"""Pretend to be the Quest page: speak the same WebSocket protocol as web/client.js.

Holds both grips, moves both controllers in small circles, opens and closes the triggers, and
(optionally) presses Y (ready), A (start) and B (save). Tests the whole chain headlessly:
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

        async def packet(t, squeeze=0.0, trigger=0.0, press=None):
            buttons = {"left": [False] * 6, "right": [False] * 6}
            if press:
                side, index = press
                buttons[side][index] = True
            await ws.send_str(json.dumps({"type": "pose", "rtt_ms": None, "hands": {
                s: hand(t, s, squeeze, trigger, buttons[s]) for s in ("left", "right")}}))

        async def press(side, index):
            for pressed in (True, False):
                await packet(0.0, press=(side, index) if pressed else None)
                await asyncio.sleep(0.05)

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
            print("B pressed (save)")
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
    parser.add_argument("--record", action="store_true", help="press A before and B after the motion")
    asyncio.run(run(parser.parse_args()))


if __name__ == "__main__":
    main()
