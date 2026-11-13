# Sourced by every container shell (see Dockerfile.dev and exec_ros.sh).
source /opt/pal/alum/setup.bash
# PAL: ROS_LOCALHOST_ONLY breaks robot connection and can break the simulator.
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TIAGO_WS=/home/user/tiago_ws
if [ "${PAL_ROBOT_CONNECTED:-0}" = 1 ]; then
    # Shell opened by PAL's pal_connect.sh (robot day): keep the robot's domain and use the robot config.
    export ROS_DOMAIN_ID=2
    export TIAGO_DC_CONFIG=$TIAGO_WS/config/real.yaml
else
    # Non-default domain so other ROS users on this shared server never see our topics.
    export ROS_DOMAIN_ID=${TIAGO_SIM_DOMAIN_ID:-42}
    export TIAGO_DC_CONFIG=${TIAGO_DC_CONFIG:-$TIAGO_WS/config/sim.yaml}
fi
export GAZEBO_MODEL_PATH=$TIAGO_WS/worlds/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}
if [ -f "$TIAGO_WS/install/setup.bash" ]; then
    source "$TIAGO_WS/install/setup.bash"
fi
