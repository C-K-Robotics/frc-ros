#!/bin/bash

cd ${FRC_ROS_DIR}
source stack.env
if [ "${RUNTIME}" = "nvidia" ]; then
    echo "RUNTIME is set to nvidia"
else
    echo "RUNTIME is set to docker"
fi
make session IMG_NAME=ubuntu:latest