# Robot day: the lab's TIAGo Pro notes

Everything from the lab's `Tiago_Startup.pdf`, plus how it maps onto this workspace. The motion
rules are in PLAN.md: no motion without explicit confirmation; `config/real.yaml` keeps
`teleop.enable_motion: false` until a supervised first test.

## Power up

1. Turn the key switch at the front of the robot (base power).
2. Wait for the head lights to turn **yellow**.
3. Press the **Computer** switch on the shoulder.
4. Wait for a full boot. This can take **5 minutes or more**; do not run anything before it finishes.
5. Tap the robot's face. It greets you and is enabled.

## Container and connection

The lab builds the image with `uwaterloo-air-build-docker.sh` (robot 30, the default). The lab desktop
uses **podman**, not Docker; its run command is `podman run -it --cap-add=NET_ADMIN --net=host ...`.
Our scripts pick podman automatically when it is installed (`docker/engine.sh`), and `podman load`
reads the `docker save` archive from RUMI. Here:

```bash
TIAGO_USB=1 TIAGO_NET_ADMIN=1 docker/run_ros.sh
```

(These options only apply when the container is first created. `podman rm -f tiago_dc` first if it
already exists. `TIAGO_USB=1` is for a Quest plugged into this machine, see below.)

1. Network: robohub Wi-Fi, TIAGo Wi-Fi or TIAGo Ethernet all work. On TIAGo Wi-Fi or Ethernet the
   robot address is `10.68.0.1`. On robohub, find it with `ping tiago`.
2. `ifconfig` (or `ip a`): note the interface that reaches the robot.
3. In the container, ROS is sourced by `docker/env_sim.sh`. The lab's alternative is `sh /pal_entrypoint.sh`
   or `source /opt/pal/alum/setup.bash`.
4. Robot domain: the shell `pal_connect.sh` opens (step 5) gets `ROS_DOMAIN_ID=2` and
   `TIAGO_DC_CONFIG=config/real.yaml` from `docker/env_sim.sh`; other shells stay on the sim's 42.
5. Connect: `ros2 run cyclone_dev_cfg pal_connect.sh <interface> <robot_address>`. The lab also
   mentions a `pal_connect_mod.sh` (`sh pal_connect_mod.sh <interface> <robot_address>`) that may
   be needed inside Docker. It is **not** in `uwaterloo-air/`; ask the lab for it.
6. `ros2 topic list` should show many topics. If not: `echo $ROS_DOMAIN_ID`, set it to 2, retry.
   The Cyclone DDS settings from the notes are in `docs/cyclonedds_tiago.xml`.
7. MoveIt with RViz: `ros2 launch tiago_pro_moveit_config moveit_rviz.launch.py`. It needs a display.
   If Qt says it cannot open the display, run `xhost +local:root` on the host before starting the container.

Rootless podman (the usual setup on a shared desktop) differs from Docker in three places:
- `--userns=keep-id` maps your uid into the container, so the files you record stay yours. On the
  first `run_ros.sh`, podman may spend a few minutes copying the image for this mapping.
- The GPU comes from the NVIDIA CDI spec (`/etc/cdi/nvidia.yaml`). Without it, `run_ros.sh` warns and
  creates the container without a GPU, which is fine for the real robot.
- With `TIAGO_USB=1`, adb runs as your user and can open the Quest only if your account can write to
  its USB node (`ls -l /dev/bus/usb/*/*`; the lab may have udev rules). If it cannot, use the
  MacBook relay (option 3 below).

## Quest and a headless lab machine (L2)

**adb** (Android Debug Bridge) is Google's tool for talking to Android devices; the Quest runs
Android. We only use it for `adb reverse`, which makes the headset's `localhost:8080` reach a
computer, so the Quest Browser can open our teleop page as a secure `http://localhost` origin. adb is
in the `tiago-dc:dev` image, so nothing gets installed on L2. The Quest needs Developer Mode, and you
accept "Allow USB debugging" in the headset the first time.

If L2 is headless, ssh in from the MacBook and work in tmux. Forward the page and Foxglove so the
Mac can monitor:

```bash
ssh -L 8080:localhost:8080 -L 8765:localhost:8765 <user>@<L2>
```

Then pick whichever of these fits:

1. **Cable reaches L2.** Create the container with USB access, then connect:

   ```bash
   TIAGO_USB=1 TIAGO_NET_ADMIN=1 docker/run_ros.sh
   ```

   ```bash
   scripts/quest_connect.sh usb
   ```

2. **Cable too short: Wi-Fi adb to L2 (no cable while operating).** The Quest and L2 must be on
   the same network, e.g. the lab Wi-Fi. Plug the Quest into L2 (or any adb machine) once, switch
   it to Wi-Fi adb, unplug, and connect over the network:

   ```bash
   scripts/quest_connect.sh enable-wifi
   ```

   ```bash
   scripts/quest_connect.sh wifi <quest-ip>
   ```

   Wi-Fi adb turns off when the headset reboots; repeat `enable-wifi` then. Video and controller
   traffic go over Wi-Fi, so check the round-trip time on the page.

3. **Quest plugged into the MacBook** (same as the simulator test). With the ssh command above
   running, on the Mac:

   ```bash
   adb reverse tcp:8080 tcp:8080
   ```

   The Mac can also use Wi-Fi adb instead of the cable (`adb tcpip 5555` while plugged in, then
   `adb connect <quest-ip>:5555`).

A 3–5 m active USB-C extension cable also turns options 2 and 3 into option 1.

## Known problems with the lab robot

1. Slow start: wait about 5 minutes after power-up.
2. Rickety: the arms move a lot when the base or torso moves. Keep both fixed while collecting.
3. The head sometimes vibrates in place. Do not rely on the head camera alone.
4. The wireless e-stop is temperamental:
   - power button **red** = e-stop engaged;
   - **yellow** = try holding the button for 1 s to reset;
   - it must be **green** before the robot works properly;
   - on the robot, `sudo systemctl restart pal_startup.service` restarts the services;
   - more: https://docs.pal-robotics.com/edge/general/tiagopro/power-management
5. Text-to-speech does not work.
6. The face can delaminate while running. Watch it and press it back into place.

## Simulator note from the same document

Slow Gazebo (< 10 fps) means the container lacks GPU passthrough (`--runtime nvidia --gpus all
--device /dev/dri`). `docker/run_ros.sh` always adds them.
