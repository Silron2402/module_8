#!/bin/bash

# Function to display the script usage
usage() {
    echo "Usage: $0 [-h] [-c image_name] [-m command]"
    echo "  -h              Display this help message."
    echo "  -c image_name   Specify the name of docker image."
    echo "  -m command      Specify the command to run inside the Docker container."
    exit 1
}

# Default values
image_name=""
command="/home/docker/app/scripts/start_sim.sh"
command="bash"

# Parse the command-line arguments
while getopts ":hc:m:" opt; do
    case $opt in
        h)
            usage
            ;;
        c)
            image_name="$OPTARG"
            ;;
        m)
            command="$OPTARG"
            ;;
        \?)
            echo "Invalid option: -$OPTARG"
            usage
            ;;
        :)
            echo "Option -$OPTARG requires an argument."
            usage
            ;;
    esac
done

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Container run
CONTAINER_ID=$(docker run -it --rm -d --privileged \
    --user=$(id -u $USER):$(id -g $USER) \
    --network=host \
    --name px4_simulation \
    --env="DISPLAY" \
    --runtime=nvidia --gpus all \
    --env="LIBGL_ALWAYS_SOFTWARE=0" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$SCRIPT_DIR/..:/home/docker/app" \
    --volume="$SCRIPT_DIR/../sim/.cache:/home/docker/.gazebo" \
   $image_name)

# Run gazebo with the specified command
docker exec -it $CONTAINER_ID /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                                            source /PX4-Autopilot/Tools/setup_gazebo.bash \
                                            /PX4-Autopilot /PX4-Autopilot/build/px4_sitl_default && \
                                            export ROS_PACKAGE_PATH=/opt/ros/noetic/share:/PX4-Autopilot:/PX4-Autopilot/Tools/sitl_gazebo && \
                                            $command"

echo "STOP DOCKER CONTAINER!"
docker stop $CONTAINER_ID
