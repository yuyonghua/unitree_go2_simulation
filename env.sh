#!/bin/bash
# Environment setup for go2_base_ws (Base Robot Assets & Sim-to-Real HAL)

# 1. Source ROS 2 base environment
source /opt/ros/humble/setup.bash

# 2. Source Unitree ROS 2 CycloneDDS packages if present
if [ -f "/home/user/unitree_ws/sdk/unitree_ros2/cyclonedds_ws/install/setup.bash" ]; then
    source /home/user/unitree_ws/sdk/unitree_ros2/cyclonedds_ws/install/setup.bash
fi

# 3. Source local install overlay
if [ -f "$HOME/git/yyh/go2_base_ws/install/setup.bash" ]; then
    source "$HOME/git/yyh/go2_base_ws/install/setup.bash"
fi

# 4. Middleware & DDS configuration
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/git/yyh/go2_base_ws/cyclonedds.xml
export ROS_DOMAIN_ID=0

export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# 5. Gazebo Fortress simulation asset search paths
BASE_DIR="$HOME/git/yyh/go2_base_ws/src/go2_gazebo_bringup"
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$BASE_DIR/worlds:$BASE_DIR/models:$HOME/git/pal_gazebo_worlds/models:$HOME/git/pal_gazebo_worlds/worlds
export IGN_GAZEBO_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH
export SDF_PATH=$SDF_PATH:$BASE_DIR/models:$HOME/git/pal_gazebo_worlds/models

export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib

# 6. UDP receive buffer check
if [ "$(uname -s)" = "Linux" ]; then
    RMEM_CUR=$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)
    if [ "$RMEM_CUR" -lt 2147483647 ] 2>/dev/null; then
        echo -e "\033[33m[INFO] CycloneDDS recommendation: sudo sysctl -w net.core.rmem_max=2147483647\033[0m"
    fi
fi

# 7. Diagnostic shortcuts
alias echo_tf="ros2 topic echo /robot1/tf"
alias echo_tf_static="ros2 topic echo /robot1/tf_static"
alias tf_tree="ros2 run tf2_tools view_frames --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static"
