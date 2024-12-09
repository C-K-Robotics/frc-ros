#!/bin/bash

xhost +
docker run \
    --name frc_ros \
    --runtime nvidia \
    -it \
    --rm \
    --privileged \
    --net=host \
    --gpus all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY=$DISPLAY \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    --device /dev/video0 \
    --volume='/dev/input:/dev/input' \
    --volume='/home/jetson/.Xauthority:/root/.Xauthority:rw' \
    --volume='/tmp/.X11-unix/:/tmp/.X11-unix' \
    ghcr.io/c-k-robotics/frc-ros:humble-arm
    
    #dustynv/opencv:4.8.1-r36.2.0 
    #arm64v8/ros:humble-perception
