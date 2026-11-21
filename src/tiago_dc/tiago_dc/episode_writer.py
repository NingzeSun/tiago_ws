"""Write one teleop episode in the RoboTwin 2.0 HDF5 layout that CoDex reads.

Layout under <data_root>/<task_name>/<collection>/:
    data/episode{N}.hdf5          RoboTwin keys + TIAGo extras (see below)
    instructions/episode{N}.json  {"seen": [instruction], "unseen": [instruction]}
    meta/episode{N}.json          robot, rates, config hash, timing, drop counts

RoboTwin keys (exactly what CoDex's prepare_dataset / build_affordance_labels read):
    observation/<cam>/rgb           JPEG bytes, fixed-length S dtype, null-padded
    observation/<cam>/intrinsic_cv  (T, 3, 3)
    observation/<cam>/extrinsic_cv  (T, 3, 4) world -> OpenCV optical frame
    observation/<cam>/cam2world_gl  (T, 4, 4) camera (OpenGL axes) -> world
    joint_action/{left_arm, right_arm} (T, 7) measured arm joints
    joint_action/{left_gripper, right_gripper} (T,) commanded normalized gripper (1 open)
    joint_action/vector             (T, 16) [left_arm, left_gripper, right_arm, right_gripper]
    endpose/{left_endpose, right_endpose} (T, 7) measured TCP [xyz, qw qx qy qz] in world
    endpose/{left_gripper, right_gripper} (T,) commanded normalized gripper
TIAGo extras (ignored by CoDex):
    action/joint_command            (T, 16) commanded arm joints + normalized grippers
    action/{left,right}_tcp_command (T, 7) commanded TCP [xyz, qw qx qy qz]
    state/joint_position, state/joint_velocity (T, J) full /joint_states, names in attrs
    state/gripper_measured          (T, 2) finger joint mapped to normalized gripper
    timestamp/sim_s, timestamp/wall_s (T,)
Operator (ignored by CoDex; quest_local_floor frame, WebXR axes, poses [xyz, qw qx qy qz]; NaN
where the matching *_valid flag is false):
    operator/head_pose, operator/{left,right}_controller_pose (T, 7)
    operator/{left,right}_eye_pose  (T, 7) gaze origin; the gaze points along the pose's -z
    operator/eye_confidence         (T, 2)
    operator/gaze_camera            (T,) index into attrs gaze_cameras, -1 = off the camera tiles
    operator/gaze_pixel             (T, 2) (u, v) px in that camera's image
    operator/face_weights           (T, F) Meta face expressions, names in attrs face_names
    operator/{head,eye,face}_valid (T,), operator/controller_valid (T, 2)
    operator/headset_s              (T,) headset clock
Root attrs: instruction, success (operator label), meta (JSON).

JPEG convention: RoboTwin's collector `cv2.imencode`s the true-RGB array without RGB2BGR, so
the stored JPEG is channel-reversed and `cv2.imdecode` returns true RGB (CoDex's
`raw_io.encoded_rgb_to_array`). This writer follows the same convention on purpose.
"""

from __future__ import annotations

import json
import os
import re
from pathlib import Path

import cv2
import h5py
import numpy as np

ROBOTWIN_CAMERAS = ("head_camera", "left_camera", "right_camera")
OPENCV_TO_OPENGL = np.diag([1.0, -1.0, -1.0, 1.0])


def encode_rgb(rgb: np.ndarray, quality: int) -> bytes:
    ok, buffer = cv2.imencode(".jpg", rgb, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise ValueError("JPEG encoding failed")
    return buffer.tobytes()


def camera_matrices(K: np.ndarray, world_from_optical: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(intrinsic_cv, extrinsic_cv, cam2world_gl) in RoboTwin's conventions."""
    optical_from_world = np.linalg.inv(world_from_optical)
    return (np.asarray(K, dtype=np.float32).reshape(3, 3),
            optical_from_world[:3, :].astype(np.float32),
            (world_from_optical @ OPENCV_TO_OPENGL).astype(np.float32))


class EpisodeBuffer:
    """Per-step samples kept in memory (JPEG-compressed) until the episode is saved."""

    def __init__(self, cameras: list[str], joint_names: list[str]):
        self.cameras = cameras
        self.joint_names = joint_names
        self.operator = {"client": "", "face_names": []}
        self.steps: list[dict] = []

    def __len__(self):
        return len(self.steps)

    def append(self, step: dict):
        self.steps.append(step)

    def stacked(self, key: str) -> np.ndarray:
        return np.stack([np.asarray(step[key]) for step in self.steps])

    def operator_stacked(self, key: str) -> np.ndarray:
        return np.stack([np.asarray(step["operator"][key]) for step in self.steps])


def next_episode_index(root: Path) -> int:
    indices = [int(m.group(1)) for p in (root / "data").glob("episode*.hdf5")
               if (m := re.fullmatch(r"episode(\d+)\.hdf5", p.name))]
    return max(indices, default=-1) + 1


def write_episode(root: Path, buffer: EpisodeBuffer, instruction: str, meta: dict) -> Path:
    """Write data/instructions/meta for the next free episode index; returns the HDF5 path.

    The HDF5 is written to a temporary name and renamed, so a crash never leaves a truncated
    episode{N}.hdf5 that CoDex would pick up.
    """
    if len(buffer) == 0:
        raise ValueError("empty episode")
    for sub in ("data", "instructions", "meta"):
        (root / sub).mkdir(parents=True, exist_ok=True)
    index = next_episode_index(root)
    path = root / "data" / f"episode{index}.hdf5"
    tmp = path.with_suffix(".hdf5.partial")

    with h5py.File(tmp, "w") as f:
        for cam in buffer.cameras:
            group = f.create_group(f"observation/{cam}")
            jpegs = [step["rgb"][cam] for step in buffer.steps]
            group.create_dataset("rgb", data=jpegs, dtype=f"S{max(len(j) for j in jpegs)}")
            group.create_dataset("intrinsic_cv", data=np.stack([s["intrinsic_cv"][cam] for s in buffer.steps]))
            group.create_dataset("extrinsic_cv", data=np.stack([s["extrinsic_cv"][cam] for s in buffer.steps]))
            group.create_dataset("cam2world_gl", data=np.stack([s["cam2world_gl"][cam] for s in buffer.steps]))

        left_arm, right_arm = buffer.stacked("left_arm"), buffer.stacked("right_arm")
        left_grip, right_grip = buffer.stacked("left_gripper"), buffer.stacked("right_gripper")
        joint = f.create_group("joint_action")
        joint.create_dataset("left_arm", data=left_arm)
        joint.create_dataset("left_gripper", data=left_grip)
        joint.create_dataset("right_arm", data=right_arm)
        joint.create_dataset("right_gripper", data=right_grip)
        joint.create_dataset("vector", data=np.concatenate(
            [left_arm, left_grip[:, None], right_arm, right_grip[:, None]], axis=1))

        endpose = f.create_group("endpose")
        endpose.create_dataset("left_endpose", data=buffer.stacked("left_endpose"))
        endpose.create_dataset("left_gripper", data=left_grip)
        endpose.create_dataset("right_endpose", data=buffer.stacked("right_endpose"))
        endpose.create_dataset("right_gripper", data=right_grip)

        action = f.create_group("action")
        action.create_dataset("joint_command", data=buffer.stacked("joint_command"))
        action.create_dataset("left_tcp_command", data=buffer.stacked("left_tcp_command"))
        action.create_dataset("right_tcp_command", data=buffer.stacked("right_tcp_command"))

        state = f.create_group("state")
        state.attrs["joint_names"] = json.dumps(buffer.joint_names)
        state.create_dataset("joint_position", data=buffer.stacked("joint_position"))
        state.create_dataset("joint_velocity", data=buffer.stacked("joint_velocity"))
        state.create_dataset("gripper_measured", data=buffer.stacked("gripper_measured"))

        stamps = f.create_group("timestamp")
        stamps.create_dataset("sim_s", data=buffer.stacked("sim_s"))
        stamps.create_dataset("wall_s", data=buffer.stacked("wall_s"))

        operator = f.create_group("operator")
        operator.attrs["frame"] = "quest_local_floor: headset floor frame, x right, y up, z toward the operator"
        operator.attrs["client"] = buffer.operator["client"]
        operator.attrs["face_names"] = json.dumps(buffer.operator["face_names"])
        operator.attrs["gaze_cameras"] = json.dumps(buffer.cameras)
        for key in ("head_pose", "left_controller_pose", "right_controller_pose", "left_eye_pose", "right_eye_pose",
                    "eye_confidence", "gaze_pixel", "headset_s", "head_valid", "controller_valid", "eye_valid",
                    "face_valid"):
            operator.create_dataset(key, data=buffer.operator_stacked(key))
        operator.create_dataset("gaze_camera", data=buffer.operator_stacked("gaze_camera").astype(np.int8))
        faces = [step["operator"]["face_weights"] for step in buffer.steps]
        face_weights = np.full((len(faces), len(buffer.operator["face_names"])), np.nan, dtype=np.float32)
        for i, weights in enumerate(faces):
            if weights is not None and len(weights) == face_weights.shape[1]:
                face_weights[i] = weights
        operator.create_dataset("face_weights", data=face_weights)
        f.attrs["instruction"] = instruction
        f.attrs["success"] = bool(meta["success"])
        f.attrs["meta"] = json.dumps(meta)
    os.replace(tmp, path)

    (root / "instructions" / f"episode{index}.json").write_text(
        json.dumps({"seen": [instruction], "unseen": [instruction]}, indent=2))
    (root / "meta" / f"episode{index}.json").write_text(json.dumps({**meta, "episode_index": index}, indent=2))
    return path
