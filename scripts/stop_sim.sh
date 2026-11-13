#!/usr/bin/env bash
# Stop the tmux session `sim` and any Gazebo/ROS processes it left behind. The container keeps
# running; `docker stop tiago_dc` (podman on L2) stops it too.
set -uo pipefail
NAME=${TIAGO_CONTAINER:-tiago_dc}
source "$(dirname "$0")/../docker/engine.sh"
# The [x]yz patterns keep pkill -f from matching this shell's own command line.
ctr exec -u user "$NAME" bash -c '
tmux kill-session -t sim 2>/dev/null
sleep 2
pkill -INT -f "[r]os2 launch" 2>/dev/null
sleep 4
pkill -9 -f "[g]zserver|[r]os2 launch|[f]oxglove_bridge|[m]ove_group|[t]iago_dc/lib" 2>/dev/null
pkill -x Xvfb 2>/dev/null
rm -f /tmp/.X99-lock /tmp/.X11-unix/X99
ros2 daemon stop >/dev/null 2>&1
true'
echo "sim stopped"
