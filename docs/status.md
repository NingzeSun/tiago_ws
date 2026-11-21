# Status — 2026-10-16 (RUMI, Gazebo)

## Verified on RUMI

| Check | Result |
|---|---|
| PAL image `development-tiago-pro-30:alum-edge` | built with user-local buildx v0.38.0 (8.7 GB) |
| `tiago-dc:dev` + container `tiago_dc` | GPU visible (`nvidia-smi`), uid mapped, `ROS_DOMAIN_ID=42`, Cyclone DDS |
| Headless rendering | Xvfb + `vglrun -d egl`: `glxinfo` reports *NVIDIA GeForce RTX 5090*; `gzserver` listed in `nvidia-smi` |
| Gazebo | real-time factor 0.96 with three 640×480 cameras at 30 Hz |
| Cameras | head `bgr8` 30 Hz; left/right wrist `rgb8` 30 Hz (overlay sensors); upright views of fingers and objects |
| Controllers | arms, grippers, torso and head JointTrajectoryControllers active; gripper 0.07 open / 0.0 closed (fingertips 83 mm / 28 mm apart) |
| Startup | arms reach the ready pose, robot base stays at the origin (< 1 cm), table/cube/can/tray spawned |
| Teleop end-to-end | `fake_quest_client.py --record --seconds 12`: input 70 Hz, both TCPs traced 10 cm circles, grippers fully closed and reopened |
| Recorder | 183 steps in 12.2 s sim (15.00 Hz), 0 dropped; command → measured TCP lag 8.9 mm mean |
| CoDex compatibility | `check_episode.py` via CoDex readers: states (T, 16), eef_abs (T, 14), eef_abs_r6 (T, 20); `prepare_dataset.py` produced 90 samples; images decode to true RGB via `raw_io.encoded_rgb_to_array` |
| Geometry | both TCPs project between the fingertips (wrist cameras) and onto the grippers (head camera) using the stored `intrinsic_cv` / `extrinsic_cv` |
| Replay | `replay_episode.py`: joint error 0.26° mean, 0.68° p95, 1.00° max |
| Events | start / stop / success / failure / discard / ready / reset_scene all behave as designed (labels, 2026-11-15) |
| Operator streams (2026-11-15) | `fake_quest_client.py` (Unity-style and `--webxr`): `operator/*` filled with valid flags, gaze mapped to head-camera pixels, failures in `<collection>_failed/`, both pass `check_episode.py` |
| Page | served on 127.0.0.1:8080; WebSocket without token → 403 |
| Contacts (gripper closure) | empty-jaw closes → 0 contact frames; synthetic 40 mm stall → contact from frame 100 to release, accepted by CoDex's `load_episode_contacts` (grasp event 100, finger points 29 px apart at 0.48 m) |
| Base weld (2026-10-19) | grippers driven to the 0.83 m TCP floor and swept, then torso lowered 0.10 m (TCPs 8 cm into the table): base drift ≤ 0.006 mm, yaw ≤ 0.001° |
| Gazebo grasping | **does not hold**: PAL's gripper drives fingers and linkage kinematically; closing on the can flung it 1.8 m and the fingers reached fully closed |

## Not verified yet

- A real Quest in the loop: WebXR input mapping on the headset, comfort of `quest_to_world`, passthrough panel
  placement, round-trip latency over Mac → ssh. Do this first tomorrow (README §2).
- The Unity app (`quest_unity/QuestTeleopClient.cs`): never compiled or run on a Quest Pro; check its controller
  axes against the WebXR page, the eye and face streams, and the panel gaze point.
- Cloudflare latency: the headset works through the tunnel (operator report, 2026-10-19), but lag is noticeable and not measured.
- Foxglove layout import (`docs/foxglove_layout.json`) in the Foxglove app.
- Long sessions (memory over many episodes).
- A real positive contact from closure (needs the robot, or a sim grasp assist).
- Anything on the real robot (`config/real.yaml` is a draft; motion disabled).
