#!/bin/bash

# Setup environment
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Change your px4 path
export GAZEBO_MODEL_PATH=$SCRIPT_DIR/../sim/models:/PX4-Autopilot/Tools/sitl_gazebo/models

echo $ROS_MASTER_URI
MODEL="uav_lidar"
RVIZ_ENABLE=true
WORLD="mapping_test" # mapping_village - , mapping_test - simple objects for test
roslaunch $SCRIPT_DIR/../launch/uav_simulator.launch \
          sdf:=$SCRIPT_DIR/../sim/models/$MODEL/$MODEL.sdf \
          world:=$SCRIPT_DIR/../sim/worlds/$WORLD.world \
          script_directory:=$SCRIPT_DIR \
          rviz_enable:=$RVIZ_ENABLE
