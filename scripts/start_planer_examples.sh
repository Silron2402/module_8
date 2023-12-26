#!/bin/bash

# Setup environment
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Change your px4 path
export GAZEBO_MODEL_PATH=$SCRIPT_DIR/../sim/models:/PX4-Autopilot/Tools/sitl_gazebo/models

WORLD="mapping_test" # mapping_village - , mapping_test - simple objects for test
roslaunch $SCRIPT_DIR/../launch/planer_example.launch script_directory:=$SCRIPT_DIR
