#!/usr/bin/env bash
# Open a ROS shell in the running container, or run one command there:
#   docker/exec_ros.sh                      interactive shell
#   docker/exec_ros.sh ros2 topic list      one-shot command
set -euo pipefail
source "$(dirname "$0")/engine.sh"
NAME=${TIAGO_CONTAINER:-tiago_dc}
if [ $# -eq 0 ]; then
    exec "$TIAGO_ENGINE" exec -it -u user -w /home/user/tiago_ws "$NAME" bash -i
fi
TTY=()
[ -t 0 ] && [ -t 1 ] && TTY=(-it)
exec "$TIAGO_ENGINE" exec "${TTY[@]}" -u user -w /home/user/tiago_ws "$NAME" \
    bash -c 'source /home/user/tiago_ws/docker/env_sim.sh && exec "$@"' _ "$@"
