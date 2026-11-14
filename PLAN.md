# PLAN — Meta Quest VR teleop data collection for TIAGo Pro (ROS 2 Humble), validated in Gazebo first

> Lives at `/home/n7sun/Research 2026/projects/tiago_ws/PLAN.md` on RUMI (L1). Day-to-day commands are in
> `README.md`; what is verified is in `docs/status.md`. **Priority: everything stays compatible with
> DeSKAD** (RoboTwin HDF5 layout, camera roles, `eef_abs_r6`); this plan is the reference, not a contract.

---

## 1. Context

**Goal:** collect teleoperation demonstrations on a PAL TIAGo Pro using a Meta Quest, then train and deploy
DeSKAD / diffusion policy / VLA models on the real robot. Before touching the real robot, the full pipeline
(Quest → teleop → recorder → dataset → policy rollout) must work in Gazebo with the same ROS 2 interfaces as the
real robot.

**About me:** new to ROS 2 and real robots. Explain each ROS 2 concept briefly the first time it is used. My
earlier data came from RoboTwin (no ROS); the recorder writes that same RoboTwin HDF5 layout so the DeSKAD
training code stays the same.

**Robot:** PAL TIAGo Pro (two 7-DoF SEA arms, PAL Pro grippers, torso lift, head, omni base), PAL OS 25.01
("alum", Ubuntu 22.04, ROS 2 Humble, Cyclone DDS). The lab robot is **tiago-pro-30** (confirmed 2026-10-19), the build script default used here.

**Docs:** https://docs.pal-robotics.com/25.01/tiagopro.html ·
[simulator](https://docs.pal-robotics.com/25.01/general/getting-started-simulator.html) ·
[ROS 2 connection](https://docs.pal-robotics.com/25.01/development/robot-communication-ros2.html) ·
[topics](https://docs.pal-robotics.com/25.01/topics.html) · lab notes (from the lab's Tiago_Startup.pdf): `docs/robot_day.md`.

### Machines
| Name | Where | Role |
|---|---|---|
| **L1 = RUMI** | server room, RTX 5090, Ubuntu 24.04 host, headless | development, Gazebo sim, training. Not connected to the robot. |
| **L2** | TIAGo lab, RTX 5090 | real-robot day only; Ethernet to robot; runs the SAME image under rootless podman (lab's engine); nothing installed on its host |
| **MacBook** | office, campus network | ssh to RUMI (port forwards 8080 Quest page, 8765 Foxglove), Quest USB relay via `adb reverse`, Foxglove. No ROS 2. |

### Facts from the lab's PAL instructions
- PAL files on RUMI: `tiago_ws/uwaterloo-air/` (`uwaterloo-air-build-docker.sh` + `keys/`), owner-only and
  gitignored. **The build script embeds credentials: never print
  it, copy it or `keys/` into a repo, or commit them.**
- The script builds `development-tiago-pro-30:alum-edge` from `palrobotics/deploy-alum-edge-tiago-pro:latest`
  with `docker buildx` (installed user-locally in `~/.docker/cli-plugins/docker-buildx`). Built 2026-10-16 (8.7 GB).
- **Do not use `--create`**: it sets `ROS_LOCALHOST_ONLY=1`. `docker/run_ros.sh` replaces it.
- Inside the container: `source /opt/pal/alum/setup.bash` (done by `docker/env_sim.sh`).
- Real robot (L2): `10.68.0.1` over TIAGo Ethernet/Wi-Fi, `ROS_DOMAIN_ID=2`,
  `ros2 run cyclone_dev_cfg pal_connect.sh <iface> 10.68.0.1` (or the lab's `pal_connect_mod.sh`).
- MoveIt: `ros2 launch tiago_pro_moveit_config moveit_rviz.launch.py` (not used by teleop, see Phase 2).
- Robot quirks: arms shake when base/torso move → base and torso fixed during collection; head may vibrate;
  wireless e-stop flaky (status light must be green).

---

## 2. Hard rules

1. **Never send motion commands to the REAL robot without my explicit confirmation in chat.** (`config/real.yaml`
   ships with `teleop.enable_motion: false` and no automatic ready motion.)
2. All teleop/policy motion goes through a **deadman** (Quest grip). Release = stop sending commands / hold.
3. **Never assume** topic, controller, joint or frame names or message types — verify (`scripts/probe_interface.py`).
4. Sim vs real differences live **only in config files** (`config/sim.yaml`, `config/real.yaml`).
5. In sim, all nodes use `use_sim_time:=true` (from the config's top-level `use_sim_time`).
6. Prefer `safe_command` topics where they exist. (The Gazebo controllers expose only `~/joint_trajectory`; check
   the real robot.)
7. RUMI is shared: no host `sudo` changes without asking; long-running processes in tmux.
8. Ask before deleting files, images or containers.

---

## 3. Status (2026-10-16)

| Phase | State | Notes |
|---|---|---|
| 0 Sim environment | **done** | Gazebo Classic 11 headless on the GPU via Xvfb + VirtualGL EGL, RTF ≈ 0.96; container `tiago_dc` from `tiago-dc:dev`; Foxglove bridge on 127.0.0.1:8765 |
| 1 Robot interface in sim | **done** | `config/sim.yaml`, `docs/interface_sim.md` (regenerate with `scripts/probe_interface.py`) |
| 2 Teleop core | **done (Quest instead of keyboard)** | pinocchio DLS IK, clutch/deadman, workspace box, speed and joint limits |
| 3 Quest bridge | **done in sim; needs the headset** | WebXR page in the Quest Browser (no APK, no ZMQ relay); tested end-to-end with `scripts/fake_quest_client.py` |
| 4 Recorder | **done** | RoboTwin HDF5 layout, checked with DeSKAD's own readers and `prepare_dataset.py`; replay error 0.26° mean; contacts from gripper closure |
| 5 Policy runtime in sim | not started | |
| 6 Real robot | not started | `config/real.yaml` is an unverified draft |

---

## 4. Phase 0 — Simulation environment on RUMI (done)

What exists (details in README):
- Host: Ubuntu 24.04, driver 595.91, Docker 29.1 with the NVIDIA runtime, user in `docker` group, tmux, ~790 GB free.
- `docker/Dockerfile.dev` = PAL image + foxglove_bridge, aiohttp, h5py, Xvfb, VirtualGL → `tiago-dc:dev`
  (`docker/build_dev.sh`).
- `docker/run_ros.sh`: idempotent, `--init --net host --ipc host --gpus all --runtime nvidia --device /dev/dri`,
  no X11, mounts the workspace at `/home/user/tiago_ws`, PAL entrypoint maps `user` to my uid.
- `docker/env_sim.sh`: `unset ROS_LOCALHOST_ONLY`, `ROS_DOMAIN_ID=42`, Cyclone DDS, workspace overlay.
- `scripts/start_sim.sh` / `stop_sim.sh`: tmux session `sim` inside the container (xvfb, gazebo, foxglove, teleop, shell).
- Launch: `tiago_pro_gazebo.launch.py gzclient:=False rviz:=False has_wrist_camera:=True world_name:=worlds/tiago_dc.world`.
  Gazebo Classic camera sensors need GLX; `vglrun -d egl` renders them on the 5090 without an X server.

Findings:
- `has_wrist_camera:=True` adds only the camera housing (no sensor). `src/pal_sea_arm_description_sim` is a
  sim-only overlay of `pal_sea_arm_description` that adds a 640×480 RGB Gazebo sensor (30 Hz) above each gripper,
  publishing `/gripper_{left,right}_camera/color/{image_raw,camera_info}` with `..._color_optical_frame`.
- PAL's startup tuck/home motions sweep the arms through anything in front of the robot (it shoved the base 0.18 m
  and threw objects). The world file therefore has no furniture; `scene_node` spawns table/objects after the arms
  reach the ready pose.
- The Gazebo head color image is stamped `head_front_camera_depth_optical_frame` (one sensor renders both); the
  recorder always uses each image's own `frame_id`.
- **Grasping does not hold in this Gazebo model.** PAL's gripper drives the finger joint and its linkage
  (ros2_control mimic joints) kinematically, so the fingers pass through objects, objects get ejected, and the
  measured finger position always equals the command. Teleop, recording and the data format are unaffected; task
  demos with real grasps need either a sim grasp assist or the real robot (open decision).
- PAL's `green_cube` (60 mm) and `coke_can_slim` (58 mm) exceed the gripper's ~55 mm pad opening; the scene uses
  our 40 mm `tiago_dc_cube` and 40 mm-diameter `tiago_dc_can`.
- Arm joints are set kinematically, so a pushed arm passes into the static table and the contact used to shove
  the omni base (planar_move only zeroes its velocity at 100 Hz). `src/omni_base_description_sim` is a sim-only
  overlay of `omni_base_description` that welds `base_footprint` to the world (Gazebo SDF joint; URDF/TF
  unchanged), matching the fixed base during real collection. The TCP floor stays at 0.83 m.
- `tiago_pro_mujoco` / `end_effectors_teleop` not evaluated (MuJoCo could be an alternative if Gazebo grasping matters).

## 5. Phase 1 — Robot interface in sim (done)

Verified names (all in `config/sim.yaml`): `/joint_states` (50 Hz); arms `/arm_{left,right}_controller/joint_trajectory`
(JointTrajectoryController, joints `arm_{side}_{1..7}_joint`); grippers `/gripper_{side}_controller/joint_trajectory`
on `gripper_{side}_finger_joint` (0.07 = open, 0.0 = closed); `/torso_controller`, `/head_controller`;
TCP frames `gripper_{side}_grasping_link`; head camera `/head_front_camera/color/image_raw` (bgr8, 640×480, 30 Hz).
No `safe_command` topics and no torso camera in this sim.

## 6. Phase 2 — Teleop core (done; Quest replaces the keyboard)

- `tiago_dc/ik.py`: damped least squares on the full URDF from `/robot_description` (only the arm's 7 joints move;
  torso/head held at measured values), joint limits with margin, per-tick joint-step limit.
- **IK vs MoveIt Servo:** chose pinocchio IK — no extra servers, deterministic per-tick latency, identical in sim
  and on the robot, and its output (joint targets) is exactly what the recorder stores. MoveIt Servo would add
  collision checking; revisit if self-collisions become a problem on the robot.
- `teleop_node`: grip = clutch + deadman; target = TCP_at_press ⊕ R·(hand_now ⊖ hand_at_press); workspace box,
  0.35 m/s and 1.5 rad/s target limits, 1.5 rad/s joint limit (all in config); 50 Hz single-point trajectories with a
  0.1 s horizon; auto re-anchor when the target leaves the reachable set; trigger → gripper; `ready` event drives
  torso 0.10 m, head tilt −0.55 rad and an IK-found ready pose (TCPs at x 0.5, y ±0.22, z 1.0, camera on top).
- Keyboard input node skipped: the WebXR page's 2D buttons plus `fake_quest_client.py` cover testing.

## 7. Phase 3 — Meta Quest bridge (done in sim; first headset test pending)

Changed from the original plan: the Quest Browser itself runs a WebXR page (`tiago_dc/web/`) served by
`quest_webxr_node` on RUMI's `127.0.0.1:8080`. **No APK, no oculus_reader, no ZMQ relay and no Python on the Mac.**
- Path A: Quest USB → Mac `adb reverse tcp:8080 tcp:8080` → Mac `ssh -L 8080:localhost:8080` → RUMI. WebXR treats
  `http://localhost` as secure, so no TLS. Needs Quest developer mode.
- Path B: `cloudflared tunnel --url http://127.0.0.1:8080` (no USB/developer mode; data passes through Cloudflare).
- Token-protected WebSocket (token in `data/.quest_token`, URL in `data/quest_url.txt`).
- Publishes `/quest/{left,right}/pose`, `/quest/joy`, events on `/teleop/event`. Mapping: grip = deadman,
  trigger = gripper, A = start, B = save, X = discard, Y = ready, right stick click = reset objects (sim).
- The page shows a head + wrist camera mosaic with a REC/clutch status line, in passthrough or VR; round-trip
  latency is measured every second and logged by the bridge.
- Same bridge on L2 on robot day: Quest by USB or Wi-Fi adb to L2 (`scripts/quest_connect.sh`, adb is in the image,
  container created with `TIAGO_USB=1`), or plugged into the MacBook with an ssh tunnel to a headless L2.
- Optional later test: can the Quest reach RUMI directly over the campus network? A non-localhost origin needs
  HTTPS for WebXR, so this would need a certificate; the Mac relay or cloudflared avoid that.

## 8. Phase 4 — Data recorder (done)

- `recorder_node`: 15 Hz on sim time; newest message per stream; drops a step if any stream is older than 100 ms.
- Format: RoboTwin HDF5 (`observation/{head,left,right}_camera/{rgb,intrinsic_cv,extrinsic_cv,cam2world_gl}`,
  `joint_action/*`, `endpose/*`) + `instructions/episode{N}.json` + `meta/episode{N}.json` (robot id, rates, config
  sha256, drops). Extras: `action/joint_command`, `action/*_tcp_command`, `state/*`, `timestamp/*`.
- `scripts/check_episode.py` (DeSKAD readers + projected-TCP frame sheet + traces), `scripts/replay_episode.py`
  (sim only). LeRobot/zarr converters not written (DeSKAD first).
- Contacts for SKAE labels: `scripts/extract_gripper_contacts.py` writes DeSKAD `contacts/episodeN.npz` from
  gripper closure (commanded closed beyond the measured opening, fingers stalled = contact; TCP as contact point,
  fingertip pads as the two clusters). Verified with DeSKAD's `load_episode_contacts` on a synthetic stall and
  with zero false contacts on empty-jaw closes. In this Gazebo model the fingers never stall (see Phase 0), so
  real positives come from the robot (PAL's grasper uses the same stall rule: `/gripper_*_grasper_srv/is_grasped`).
- `scripts/scripted_grasp_demo.py`: headless grasp episode through the Quest protocol (test harness).

## 9. Phase 5 — Policy runtime in sim (next)

- Separate policy environment (the DeSKAD conda env `RoboTwin5090` already has CUDA 12.8+ PyTorch for the 5090).
- Policy server (no ROS): DeSKAD's serve path. Obs dict built by the **same code as the recorder**; JPEG images.
- `policy_client_node` (ROS): builds obs, queries the server, executes chunks, requires the deadman.
  DeSKAD runtime constraint: no runtime action overrides/gates beyond the teleop safety limits.
- Server location is a config option (same machine by default; ssh tunnel otherwise). DDS never crosses the campus network.

## 10. Phase 6 — Real robot on L2 (only after Phases 0–5 pass; every motion step needs my confirmation)

- Before going: `docker save tiago-dc:dev | gzip > tiago-dc-dev.tar.gz` (or a private registry), code in git, a printed checklist.
- Ask the lab: podman access on L2 (NVIDIA CDI spec, USB permission for the Quest), who handles the e-stop.
- Power-up, e-stop and connection details: `docs/robot_day.md`.
- On L2: `podman load` the image, plug Ethernet, `ping 10.68.0.1`, `TIAGO_NET_ADMIN=1 docker/run_ros.sh`, `export ROS_DOMAIN_ID=2`,
  `ros2 run cyclone_dev_cfg pal_connect.sh <eth_iface> 10.68.0.1`, wait up to a minute, `ros2 topic list`.
  `env_sim.sh` sets domain 42; override with `TIAGO_SIM_DOMAIN_ID=2` (or a `env_real.sh`).
- If no topics: PAL troubleshooting (`ddsperf sanity`; for direct cable, robot Wi-Fi off or br0 highest priority).
- Time sync L2 ↔ robot (chrony); check header stamp offsets.
- Read-only first: `TIAGO_DC_CONFIG=config/real.yaml scripts/probe_interface.py docs/interface_real.md`; diff with
  sim; fix `config/real.yaml` (wrist camera topics are guesses; real wrist cameras/mount must be calibrated).
- Read-only recording (cameras + joints, no motion); check bandwidth; compressed transport if needed.
- First motion: `enable_motion: true` only with my confirmation; one arm, very slow, small workspace, someone on the e-stop.

## 11. Repo layout

```
tiago_ws/                     (/home/n7sun/Research 2026/projects/tiago_ws, mounted at /home/user/tiago_ws)
  PLAN.md  README.md  .gitignore
  uwaterloo-air/   PAL build script + keys (owner-only, gitignored)
  docker/          Dockerfile.dev, build_dev.sh, run_ros.sh, exec_ros.sh, env_sim.sh, engine.sh (docker or podman)
  config/          sim.yaml (verified), real.yaml (draft)
  src/tiago_dc/    ament_python: ik, teleop_node, quest_webxr_node (+ web/), recorder_node, episode_writer, scene_node
  src/pal_sea_arm_description_sim/   sim-only wrist camera sensors
  src/omni_base_description_sim/     sim-only weld of the base to the world
  worlds/          tiago_dc.world, models/tiago_dc_{tray,cube,can}
  scripts/         start_sim.sh, stop_sim.sh, probe_interface.py, snapshot_cameras.py, fake_quest_client.py,
                   check_episode.py, replay_episode.py, extract_gripper_contacts.py, scripted_grasp_demo.py
  docs/            interface_sim.md, status.md, robot_day.md, cyclonedds_tiago.xml, foxglove_layout.json
  data/            episodes, snapshots, Quest token (gitignored)
```

## 12. Reporting

After each phase: summarize what works and what doesn't, list exact commands to verify it, update `docs/status.md`.
