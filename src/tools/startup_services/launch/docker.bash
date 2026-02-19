#!/bin/bash

cd ${FRC_ROS_DIR}
source robot.env
if [ "${RUNTIME}" = "nvidia" ]; then
    echo "RUNTIME is set to nvidia"
else
    echo "RUNTIME is set to docker"
fi
#make session IMG_NAME=ubuntu:latest
make session RUNTIME=nvidia IMG_NAME=ghcr.io/c-k-robotics/frc_ros_humble_gpu_jetpack6:stable
