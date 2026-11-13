# TIAGo Pro teleop data collection (ROS 2 Humble · Gazebo · Meta Quest)

The Quest Browser streams both controllers to a ROS 2 bridge. Teleop drives the TIAGo Pro arms
and grippers, and a recorder writes episodes in the RoboTwin HDF5 layout that Bi-SKAD reads.
Sim and robot differ only in `config/sim.yaml` vs `config/real.yaml`. Design, status and the
other scripts are in `PLAN.md` and `docs/`.

Workspace on RUMI: `/home/n7sun/Research 2026/projects/tiago_ws`, mounted in container `tiago_dc` at
`/home/user/tiago_ws`. `docker/exec_ros.sh …` runs a command in the container. The scripts use podman
when it is installed (the lab desktop L2) and Docker otherwise (RUMI); `TIAGO_ENGINE=docker|podman` overrides. Never commit or
print `uwaterloo-air/` (PAL keys). Commands run from the workspace root in VS Code terminals on RUMI.

## 1. Simulator

```bash
scripts/start_sim.sh
```

Gazebo and teleop run in the background in the container, and keep running if VS Code disconnects.
After about 60 s the arms reach the ready pose and the table and objects appear.

To read the teleop output (other processes: `gazebo`, `foxglove`, `xvfb`):

```bash
docker/exec_ros.sh tmux capture-pane -p -t sim:teleop -S -80
```

```bash
scripts/stop_sim.sh
```

## 2. Quest

In a second terminal, and keep it running:

```bash
~/.local/bin/cloudflared tunnel --url http://127.0.0.1:8080 --no-autoupdate
```

It prints a hostname such as `https://bright-river-cats-example.trycloudflare.com`. In the
**Quest Browser**, open that hostname plus `/?token=` and the token from `data/quest_url.txt`,
including `https://`. Then press **Enter passthrough** or **Enter VR**. The hostname changes
whenever cloudflared restarts. Do not share it.

On the Mac, `http://localhost:8080/?token=…` shows the same cameras and buttons; VS Code
forwards the port. The newest page takes control, so reload the Quest page last.

## 3. Recording

| Quest | Action |
|---|---|
| Grip (hold) | engage that arm; the gripper follows your hand's motion. Release to re-position. |
| Trigger | close that gripper |
| Right A / B | start / stop and save |
| Left X | discard |
| Left Y | ready pose |
| Right stick click | new object positions (sim only) |

Face the robot's forward direction. Per episode: **Y** → **stick click** → **A** → task → **B**
(**X** if it failed). Episodes go to `data/place_cube_can_tray/tiago_gazebo_teleop/`.

Check an episode (writes a frame sheet and plots into `.../checks/`):

```bash
/home/n7sun/miniconda3/envs/RoboTwin5090/bin/python scripts/check_episode.py data/place_cube_can_tray/tiago_gazebo_teleop/data/episode0.hdf5
```

## 4. Robot day (real TIAGo Pro, robot 30)

Same image and code on the lab machine (L2). ROS 2 and PAL's stack are inside the image.
`config/real.yaml` starts with `enable_motion: false`, so nothing moves until you enable it.
Power-up and e-stop details are in `docs/robot_day.md`.

### Pack on RUMI

These commands create `/home/n7sun/tiago_robot_day/` (image several GB, code about 100 KB):

```bash
mkdir -p ~/tiago_robot_day && cp ~/.local/bin/cloudflared ~/tiago_robot_day/
```

```bash
docker save tiago-dc:dev | gzip > ~/tiago_robot_day/tiago-dc-dev.tar.gz
```

```bash
tar czf ~/tiago_robot_day/tiago_ws_code.tar.gz -C "/home/n7sun/Research 2026/projects" --exclude=tiago_ws/data --exclude=tiago_ws/uwaterloo-air --exclude=tiago_ws/build --exclude=tiago_ws/install --exclude=tiago_ws/log tiago_ws
```

Copy the folder to your home directory on L2 (scp or a USB drive). From the MacBook, open a VS
Code Remote-SSH window to L2, as you do for RUMI. Commands below run in two of its terminals:
- **Terminal 1** is a container shell connected to the robot, where teleop runs.
- **Terminal 2** connects the Quest.

### Set up once on L2

```bash
cd ~/tiago_robot_day && podman load < tiago-dc-dev.tar.gz && tar xzf tiago_ws_code.tar.gz && cd tiago_ws
```

```bash
TIAGO_USB=1 TIAGO_NET_ADMIN=1 docker/run_ros.sh && docker/exec_ros.sh colcon build --symlink-install
```

### Terminal 1: robot

Power up the robot and wait about 5 minutes; the e-stop button must be green. Connect L2 to the
robot by Ethernet, then find the interface name with `ip a`.

```bash
cd ~/tiago_robot_day/tiago_ws && docker/exec_ros.sh
```

```bash
ros2 run cyclone_dev_cfg pal_connect.sh <interface> 10.68.0.1
```

`ros2 topic list` should show the robot's topics. Check the interface without moving:

```bash
python3 scripts/probe_interface.py docs/interface_real.md
```

Fix `config/real.yaml` where it differs from `docs/interface_sim.md`; the wrist camera topics are
marked `VERIFY`. Measure the lab table: the gripper floor `teleop.workspace.z` (0.83 m) assumes a
table top at 0.815 m. Then start teleop:

```bash
ros2 launch tiago_dc teleop.launch.py
```

### Terminal 2: Quest over USB

The Quest needs Developer Mode, set once in the Meta Horizon phone app (headset → Headset settings
→ Developer mode). Plug it into L2, then run:

```bash
cd ~/tiago_robot_day/tiago_ws && scripts/quest_connect.sh usb
```

Accept "Allow USB debugging" in the headset. The page opens at `http://localhost:8080/?token=…`;
the token is in `data/quest_url.txt`.

Without a cable, then run the tunnel and open the page as in section 2:

```bash
~/tiago_robot_day/cloudflared tunnel --url http://127.0.0.1:8080 --no-autoupdate
```

### Collect

1. Read-only: the page shows the real cameras. Record a short episode; the arms do not move.
2. First motion, with someone on the e-stop: set `teleop.enable_motion: true` in `config/real.yaml`
   and restart the launch (Ctrl-C, launch again). Check clearance, press **Y**, then make small,
   slow moves with one arm. Speed is capped at 0.10 m/s.
3. Collect as in section 3. Episodes go to `data/place_cube_can_tray/tiago_real_teleop/`. Copy them
   back to RUMI and run `scripts/check_episode.py`.
