#!/bin/bash
# go2_base_ws: 机器人机体资产与虚实硬件抽象基座环境

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. ROS 2 基础环境
source /opt/ros/humble/setup.bash

# 2. 本地工作空间 Install (优先当前目录，兼容跨目录 source)
[ -f "$DIR/install/setup.bash" ] && source "$DIR/install/setup.bash"

# 3. CycloneDDS 中间件配置 (本地回环单播)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$DIR/cyclonedds.xml"
export ROS_DOMAIN_ID=0

# 4. Gazebo 仿真模型检索路径
export GZ_SIM_RESOURCE_PATH="$DIR/src/go2_gazebo_bringup/worlds:$DIR/src/go2_gazebo_bringup/models:$GZ_SIM_RESOURCE_PATH"
export SDF_PATH="$DIR/src/go2_gazebo_bringup/models:$SDF_PATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/humble/lib"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="/opt/ros/humble/lib"
