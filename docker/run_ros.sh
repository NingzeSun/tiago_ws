#!/usr/bin/env bash
# Start the long-lived tiago_dc container (create it on first use) with Docker or podman (see
# engine.sh). Headless: no X11 mount; GPU and /dev/dri are passed through for Gazebo camera
# rendering (TIAGO_GPU=0 skips them). Host networking keeps DDS discovery simple; ROS_DOMAIN_ID in
# env_sim.sh isolates this sim from other users.
set -euo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"
NAME=${TIAGO_CONTAINER:-tiago_dc}
IMAGE=${TIAGO_IMAGE:-tiago-dc:dev}
source "$WS/docker/engine.sh"

# Robot day: TIAGO_NET_ADMIN=1 adds the NET_ADMIN capability the lab's run command uses for
# pal_connect.sh. Like every option here it applies only when the container is first created.
EXTRA=()
[ "${TIAGO_NET_ADMIN:-0}" = 1 ] && EXTRA+=(--cap-add=NET_ADMIN)
# TIAGO_GPU=0 for a machine without an NVIDIA GPU (the real robot needs none; Gazebo does).
if [ "${TIAGO_GPU:-1}" = 1 ]; then
    if [ "$TIAGO_ENGINE" = docker ]; then
        EXTRA+=(--gpus all --runtime nvidia --device /dev/dri)
    elif grep -qs 'nvidia.com/gpu' /etc/cdi/*.y*ml /var/run/cdi/*.y*ml; then
        # podman reaches the GPU through the NVIDIA Container Toolkit's CDI spec.
        EXTRA+=(--device nvidia.com/gpu=all --device /dev/dri)
    else
        echo "no NVIDIA CDI spec for podman (sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml);" \
             "creating without GPU: fine for the real robot, Gazebo cameras will not render" >&2
    fi
    EXTRA+=(-e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all)
fi
# TIAGO_USB=1 lets adb inside the container reach a Quest plugged into this machine. The device
# cgroup rule (USB major 189) keeps working when the headset is unplugged and replugged. Rootless
# podman has no device cgroup; there the host user's own permission on the node decides.
if [ "${TIAGO_USB:-0}" = 1 ]; then
    EXTRA+=(-v /dev/bus/usb:/dev/bus/usb)
    [ "$TIAGO_ROOTLESS" = 1 ] || EXTRA+=(--device-cgroup-rule='c 189:* rmw')
fi
# Rootless podman: keep-id maps the host uid to the same container uid, which the PAL entrypoint
# assigns to `user`, so workspace files stay owned by the host user. The entrypoint needs root.
[ "$TIAGO_ROOTLESS" = 1 ] && EXTRA+=(--userns=keep-id --user root)

if ctr container inspect "$NAME" >/dev/null 2>&1; then
    if [ "$(ctr inspect -f '{{.State.Running}}' "$NAME")" != true ]; then
        ctr start "$NAME" >/dev/null
    fi
else
    # --init reaps the zombies Gazebo leaves behind when a launch is killed.
    ctr run -d --init --name "$NAME" \
        --net host --ipc host \
        -e LOCAL_USER_ID="$(id -u)" -e LOCAL_GROUP_ID="$(id -g)" \
        -v "$WS:/home/user/tiago_ws" \
        -w /home/user/tiago_ws \
        "${EXTRA[@]}" \
        "$IMAGE" sleep infinity >/dev/null
fi
echo "container '$NAME' is running ($TIAGO_ENGINE, $IMAGE); open a shell with docker/exec_ros.sh"
