#!/bin/bash

from="$(pwd)"
launch_file="at.launch"
build_command="catkin build"
ws_dir="../../../at_ws"

if [[ ! -d "$ws_dir" ]]; then
    echo "Expecting directory $ws_dir"
    exit 1
fi

if [[ -d /opt/ros/melodic ]]; then
    echo "Detected ROS melodic"
elif [[ -d /opt/ros/lunar ]]; then
    build_command="catkin_make"
    echo "Detected ROS lunar"
fi

if (( $# == 0 )); then
    echo "Using default launch file: ${launch_file}"
elif (( $# == 1 )); then
    launch_file="$1"
    if [[ "${launch_file}" != *".launch" ]]; then
        launch_file="${launch_file}.launch"
    fi
    echo "Using launch file: ${launch_file}"
else
    echo "Error: Expecting 0 or 1 argument."
    exit 1
fi

cd "$ws_dir"

$build_command

if [[ $? -ne 0 ]]; then
    echo -e "\n! Build failed !"
    exit 1
fi

if [[ ! -f devel/setup.bash ]]; then
    echo -e "\n! Could not find devel/setup.bash to source !"
    exit 1
fi

source devel/setup.bash

cd "$from"
roslaunch "${launch_file}"
