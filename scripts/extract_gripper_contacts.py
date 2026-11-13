#!/usr/bin/env python3
"""Derive grasp contacts from gripper closure and write Bi-SKAD `contacts/episodeN.npz` files.

Bi-SKAD's extract_contacts.py replays RoboTwin episodes in SAPIEN to read physical contacts;
Gazebo and real-robot episodes cannot be replayed that way. Here a contact is read from the
gripper itself: the jaw was commanded closed further than the fingers actually are, and the
fingers have stopped moving, so something is between them. An empty jaw reaches its command,
so it never counts.

Per arm and frame (normalized gripper, 1 = open):
    gap      = measured - commanded   (state/gripper_measured, endpose/*_gripper)
    stalled  = |measured[t] - measured[t-1]| <= --stall
    contact starts when gap >= --gap and stalled, and lasts while gap >= --gap / 2.
Contact position = measured TCP (between the fingertips). The two finger clusters are the TCP
+/- half the measured jaw width along the closing axis (TCP y), so build_affordance_labels.py
sees two links in contact (its --contact-min-links default) and gets per-finger SAM2 prompts.
The trust fields mirror extract_contacts.py's for an exact "replay" (the data is the recording).

    /home/n7sun/miniconda3/envs/RoboTwin5090/bin/python scripts/extract_gripper_contacts.py \
        data/place_cube_can_tray/tiago_gazebo_teleop
"""

import argparse
import json
import re
from pathlib import Path

import h5py
import numpy as np
import yaml

WORKSPACE = Path(__file__).resolve().parents[1]
MAX_CLUSTERS = 4  # same padding as Bi-SKAD's extract_contacts.py
ACTOR_NAMES = ["grasped_object"]  # the object identity is not observable from the gripper


def quat_wxyz_to_matrix(q: np.ndarray) -> np.ndarray:
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    w, x, y, z = np.moveaxis(q, -1, 0)
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


def closure_contact(commanded: np.ndarray, measured: np.ndarray, gap: float, stall: float) -> np.ndarray:
    """Per-frame contact flags for one arm (hysteresis: start on gap + stall, hold on gap / 2)."""
    speed = np.abs(np.diff(measured, prepend=measured[:1]))
    blocked = measured - commanded
    contact = np.zeros(len(measured), dtype=bool)
    active = False
    for t in range(len(measured)):
        if active:
            active = blocked[t] >= gap / 2
        else:
            active = blocked[t] >= gap and speed[t] <= stall
        contact[t] = active
    return contact


def episode_contacts(path: Path, grippers: dict, gap: float, stall: float) -> dict:
    with h5py.File(path, "r") as f:
        endpose = np.stack([f["endpose/left_endpose"], f["endpose/right_endpose"]], axis=1)  # (T, 2, 7)
        commanded = np.stack([f["endpose/left_gripper"], f["endpose/right_gripper"]], axis=1)
        measured = np.asarray(f["state/gripper_measured"], dtype=np.float64)
        meta = json.loads(f.attrs["meta"])
    frames = len(endpose)
    contact = np.zeros((frames, 2), dtype=bool)
    position = np.full((frames, 2, 3), np.nan)
    clusters = np.full((frames, 2, MAX_CLUSTERS, 3), np.nan)
    cluster_valid = np.zeros((frames, 2, MAX_CLUSTERS), dtype=bool)
    actor = np.full((frames, 2), -1, dtype=np.int64)
    for arm, side in enumerate(("left", "right")):
        flags = closure_contact(commanded[:, arm], measured[:, arm], gap, stall)
        closed_gap, open_gap = grippers[side]["jaw_opening"]
        width = closed_gap + np.clip(measured[:, arm], 0.0, 1.0) * (open_gap - closed_gap)
        tcp = endpose[:, arm, :3]
        closing_axis = quat_wxyz_to_matrix(endpose[:, arm, 3:7])[:, :, 1]
        contact[:, arm] = flags
        position[flags, arm] = tcp[flags]
        for slot, sign in ((0, 1.0), (1, -1.0)):
            clusters[flags, arm, slot] = tcp[flags] + sign * 0.5 * width[flags, None] * closing_axis[flags]
            cluster_valid[flags, arm, slot] = True
        actor[flags, arm] = 0
    return {
        "contact": contact,
        "contact_pos": position,
        "contact_clusters": clusters,
        "contact_cluster_valid": cluster_valid,
        "contact_actor": actor,
        "actor_names": np.asarray(ACTOR_NAMES, dtype=object),
        "replay_ok": np.asarray(True),
        "check_success": np.asarray(bool(meta.get("operator_marked_success", True))),
        "num_frames": np.asarray(frames, dtype=np.int64),
        "recorded_frames": np.asarray(frames, dtype=np.int64),
        "endpose_max_err": np.asarray(0.0),
        "gripper_max_err": np.asarray(0.0),
        "contact_source": np.asarray("gripper_closure"),
        "closure_gap": np.asarray(gap),
        "closure_stall": np.asarray(stall),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("collection", type=Path, help="folder holding data/episodeN.hdf5 (Bi-SKAD --raw-root)")
    parser.add_argument("--config", type=Path, default=WORKSPACE / "config" / "sim.yaml",
                        help="config the episodes were recorded with (jaw_opening per gripper)")
    parser.add_argument("--gap", type=float, default=0.08,
                        help="normalized jaw distance short of the command that counts as blocked")
    parser.add_argument("--stall", type=float, default=0.01,
                        help="max normalized finger motion per frame for the jaw to count as stopped")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    grippers = {side: arm["gripper"] for side, arm in yaml.safe_load(args.config.read_text())["arms"].items()}
    out_dir = args.collection / "contacts"
    out_dir.mkdir(exist_ok=True)
    episodes = sorted((p for p in (args.collection / "data").glob("episode*.hdf5")),
                      key=lambda p: int(re.search(r"(\d+)", p.stem).group(1)))
    if not episodes:
        raise SystemExit(f"no episodes under {args.collection / 'data'}")
    for path in episodes:
        out = out_dir / f"{path.stem}.npz"
        if out.exists() and not args.overwrite:
            print(f"{path.stem}: exists, skipped (--overwrite to redo)")
            continue
        arrays = episode_contacts(path, grippers, args.gap, args.stall)
        np.savez_compressed(out, **arrays)
        runs = []
        for arm, side in enumerate(("left", "right")):
            flags = arrays["contact"][:, arm]
            starts = np.flatnonzero(flags & ~np.r_[False, flags[:-1]])
            runs.append(f"{side} {int(flags.sum())} frames, onsets {starts.tolist()}")
        print(f"{path.stem}: " + " | ".join(runs))


if __name__ == "__main__":
    main()
