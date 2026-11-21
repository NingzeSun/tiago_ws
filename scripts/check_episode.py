#!/usr/bin/env python3
"""Check a recorded episode with CoDex's own readers and write visual sanity plots.

Runs on the host (no ROS), in the CoDex environment:

    /home/n7sun/miniconda3/envs/RoboTwin5090/bin/python scripts/check_episode.py \
        data/place_cube_can_tray/tiago_gazebo_teleop/data/episode0.hdf5

It imports prepare_dataset.read_joint_vector / read_eef_abs, constants.eef_abs_to_r6 and
raw_io.encoded_rgb_to_array from a CoDex task package (read-only), so "passes here" means
"prepare_dataset.py can consume it". It also projects both TCPs into every camera with the
stored intrinsic/extrinsic (CoDex's project_points_robotwin convention) and draws them, which
catches wrong frames, wrong extrinsic direction, or swapped colour channels at a glance.
"""

import argparse
import importlib.util
import json
import sys
from pathlib import Path

import cv2
import h5py
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

DEFAULT_TASK = Path("/home/n7sun/Research 2026/projects/CoDex/tasks/place_burger_fries")
CAMERAS = ("head_camera", "left_camera", "right_camera")


def load_codex(task_dir: Path):
    sys.dont_write_bytecode = True  # read-only use of the CoDex tree
    sys.path.insert(0, str(task_dir))
    from codex_bread import constants, raw_io  # noqa: PLC0415

    spec = importlib.util.spec_from_file_location("prepare_dataset", task_dir / "scripts/pipeline/prepare_dataset.py")
    prepare = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(prepare)
    return constants, raw_io, prepare


def project(points_world, extrinsic, intrinsic):
    camera = extrinsic[:3, :3] @ points_world + extrinsic[:3, 3]
    if camera[2] <= 1e-6:
        return None
    return intrinsic[0, 0] * camera[0] / camera[2] + intrinsic[0, 2], intrinsic[1, 1] * camera[1] / camera[2] + intrinsic[1, 2]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--codex-task", type=Path, default=DEFAULT_TASK)
    parser.add_argument("--out", type=Path, default=None, help="plot directory (default: next to the episode)")
    args = parser.parse_args()
    constants, raw_io, prepare = load_codex(args.codex_task)
    out = args.out or args.episode.parent.parent / "checks"
    out.mkdir(parents=True, exist_ok=True)
    problems = []

    with h5py.File(args.episode, "r") as f:
        states = prepare.read_joint_vector(f)
        eef_abs = prepare.read_eef_abs(f)
        eef_r6 = constants.eef_abs_to_r6(eef_abs)
        steps = len(eef_abs)
        sim_t = np.asarray(f["timestamp/sim_s"])
        meta = json.loads(f.attrs["meta"])
        left_tcp = np.asarray(f["endpose/left_endpose"])[:, :3]
        right_tcp = np.asarray(f["endpose/right_endpose"])[:, :3]
        grip_cmd = np.stack([f["endpose/left_gripper"], f["endpose/right_gripper"]], axis=1)
        grip_meas = np.asarray(f["state/gripper_measured"])
        cmd_tcp = {s: np.asarray(f[f"action/{s}_tcp_command"])[:, :3] for s in ("left", "right")}
        success = bool(f.attrs["success"]) if "success" in f.attrs else None
        op = f.get("operator")
        valid = None if op is None else {k: np.asarray(op[k]).mean(axis=0)
                                         for k in ("head_valid", "controller_valid", "eye_valid", "face_valid")}
        if op is not None:
            valid.update(client=op.attrs["client"] or "?", faces=op["face_weights"].shape[1],
                         gaze=(np.asarray(op["gaze_camera"]) >= 0).mean())
        frames = {}
        for cam in CAMERAS:
            rgb = f[f"observation/{cam}/rgb"]
            K = np.asarray(f[f"observation/{cam}/intrinsic_cv"])
            E = np.asarray(f[f"observation/{cam}/extrinsic_cv"])
            if len(rgb) != steps or K.shape != (steps, 3, 3) or E.shape != (steps, 3, 4):
                problems.append(f"{cam}: length/shape mismatch")
            frames[cam] = [(raw_io.encoded_rgb_to_array(rgb[i]), K[i], E[i]) for i in (0, steps // 2, steps - 1)]

    print(f"episode     {args.episode}")
    print(f"steps       {steps}  ({meta['dropped_steps']} dropped while recording)")
    dt = np.diff(sim_t)
    print(f"rate        {1.0 / dt.mean():.2f} Hz sim time (step dt min {dt.min():.3f} max {dt.max():.3f} s)")
    print(f"states      {states.shape} (joint_action/vector)   eef_abs {eef_abs.shape}   eef_abs_r6 {eef_r6.shape}")
    for name, tcp in (("left", left_tcp), ("right", right_tcp)):
        path = np.linalg.norm(np.diff(tcp, axis=0), axis=1).sum()
        print(f"{name:5s} TCP   range x {np.ptp(tcp[:, 0]):.3f} y {np.ptp(tcp[:, 1]):.3f} z {np.ptp(tcp[:, 2]):.3f} m, "
              f"path {path:.3f} m, tracking |cmd - measured| mean {np.linalg.norm(cmd_tcp[name] - tcp, axis=1).mean() * 1000:.1f} mm")
    print(f"gripper     commanded [{grip_cmd.min():.2f}, {grip_cmd.max():.2f}]  measured [{grip_meas.min():.2f}, {grip_meas.max():.2f}]")
    print(f"success     {'not labeled (older episode)' if success is None else success}")
    if valid is not None:
        c = valid["controller_valid"]
        print(f"operator    {valid['client']}: valid head {valid['head_valid']:.0%}, controllers {c[0]:.0%} / {c[1]:.0%}, "
              f"eyes {valid['eye_valid']:.0%}, face {valid['face_valid']:.0%} ({valid['faces']} weights), "
              f"gaze on a camera {valid['gaze']:.0%}")
    for array, name in ((states, "states"), (eef_abs, "eef_abs"), (eef_r6, "eef_abs_r6")):
        if not np.all(np.isfinite(array)):
            problems.append(f"{name} has non-finite values")
    if eef_r6.shape[1] != constants.EEF_ABS_R6_ACTION_DIM:
        problems.append(f"eef_abs_r6 dim {eef_r6.shape[1]} != {constants.EEF_ABS_R6_ACTION_DIM}")

    # Contact sheet: first/middle/last frame per camera with both TCPs projected.
    rows = []
    for cam in CAMERAS:
        tiles = []
        for (rgb, K, E), index in zip(frames[cam], (0, steps // 2, steps - 1)):
            image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            for tcp, color in ((left_tcp[index], (0, 200, 255)), (right_tcp[index], (255, 120, 0))):
                uv = project(tcp, E, K)
                if uv is not None and 0 <= uv[0] < image.shape[1] and 0 <= uv[1] < image.shape[0]:
                    cv2.circle(image, (int(uv[0]), int(uv[1])), 9, color, 3)
            cv2.putText(image, f"{cam} t={index}", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            tiles.append(cv2.resize(image, (480, 360)))
        rows.append(np.hstack(tiles))
    sheet = out / f"{args.episode.stem}_frames.jpg"
    cv2.imwrite(str(sheet), np.vstack(rows))

    figure, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    t = sim_t - sim_t[0]
    for i, axis_name in enumerate("xyz"):
        axes[0].plot(t, left_tcp[:, i], label=f"left {axis_name}")
        axes[0].plot(t, right_tcp[:, i], "--", label=f"right {axis_name}")
    axes[0].set_ylabel("TCP position (m)")
    axes[0].legend(ncol=6, fontsize=8)
    axes[1].plot(t, grip_cmd[:, 0], label="left cmd")
    axes[1].plot(t, grip_meas[:, 0], ":", label="left measured")
    axes[1].plot(t, grip_cmd[:, 1], label="right cmd")
    axes[1].plot(t, grip_meas[:, 1], ":", label="right measured")
    axes[1].set_ylabel("gripper (1 = open)")
    axes[1].legend(ncol=4, fontsize=8)
    axes[2].plot(t[1:], dt * 1000)
    axes[2].set_ylabel("step dt (ms)")
    axes[2].set_xlabel("sim time (s)")
    figure.tight_layout()
    plot = out / f"{args.episode.stem}_traces.png"
    figure.savefig(plot, dpi=110)

    print(f"plots       {sheet}\n            {plot}")
    if problems:
        print("PROBLEMS:\n  " + "\n  ".join(problems))
        sys.exit(1)
    print("OK: readable by CoDex prepare_dataset readers")


if __name__ == "__main__":
    main()
