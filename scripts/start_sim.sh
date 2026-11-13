#!/usr/bin/env bash
# One command: container (if needed) + headless Gazebo + foxglove_bridge + teleop/recorder,
# each in a window of the tmux session `sim` INSIDE the container.
#   scripts/start_sim.sh [--no-teleop] [world_name_or_path]
# Attach with: docker/exec_ros.sh tmux attach -t sim     (detach: Ctrl-b d)
set -euo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"
TELEOP=true
if [ "${1:-}" = "--no-teleop" ]; then TELEOP=false; shift; fi
WORLD=${1:-/home/user/tiago_ws/worlds/tiago_dc.world}
NAME=${TIAGO_CONTAINER:-tiago_dc}
source "$WS/docker/engine.sh"

"$WS/docker/run_ros.sh"
in_ctr() { ctr exec -u user -w /home/user/tiago_ws "$NAME" bash -c "$1"; }

if in_ctr 'tmux has-session -t sim 2>/dev/null'; then
    echo "tmux session 'sim' already running in $NAME; stop it first with scripts/stop_sim.sh" >&2
    exit 1
fi

# Every window sources the env; `exec bash` keeps the window open after a crash so the log stays visible.
win() { printf 'source /home/user/tiago_ws/docker/env_sim.sh; %s; exec bash' "$1"; }

in_ctr "pkill -x Xvfb; rm -f /tmp/.X99-lock /tmp/.X11-unix/X99; true"
in_ctr "tmux new-session -d -s sim -n xvfb \"$(win 'Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp')\""
for _ in $(seq 20); do
    in_ctr 'DISPLAY=:99 vglrun -d egl glxinfo -B 2>/dev/null | grep -q "NVIDIA"' && break
    sleep 0.5
done
if ! in_ctr 'DISPLAY=:99 vglrun -d egl glxinfo -B 2>/dev/null | grep -q "NVIDIA"'; then
    echo "virtual display :99 is not rendering on the NVIDIA GPU; see the xvfb window" >&2
    exit 1
fi
# vglrun -d egl: Gazebo Classic camera sensors render on the GPU without a real X server.
in_ctr "tmux new-window -t sim -n gazebo \"$(win "DISPLAY=:99 vglrun -d egl ros2 launch tiago_pro_gazebo tiago_pro_gazebo.launch.py gzclient:=False rviz:=False has_wrist_camera:=True world_name:=$WORLD")\""
in_ctr "tmux new-window -t sim -n foxglove \"$(win 'ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 address:=127.0.0.1 use_sim_time:=true')\""
if $TELEOP; then
    in_ctr "tmux new-window -t sim -n teleop \"$(win 'sleep 50; ros2 launch tiago_dc teleop.launch.py')\""
fi
in_ctr "tmux new-window -t sim -n shell \"$(win 'true')\""
echo "sim starting in tmux session 'sim' inside $NAME (Gazebo needs ~30-60 s)."
echo "attach: docker/exec_ros.sh tmux attach -t sim"
