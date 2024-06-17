#!/bin/bash

xhost +
docker run \
    --name frc_ros \
    --runtime nvidia \
    -it \
    --rm \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    --device /dev/video0 \
    --volume='/dev/input:/dev/input' \
    -v /usr/local/cuda:/usr/local/cuda \
    -v /usr/local/cuda-10.2:/usr/local/cuda-10.2 \
    -v /usr/src/cudnn_samples_v8:/usr/src/cudnn_samples_v8 \
    -v /usr/include:/usr/include \
    -v /usr/lib/aarch64-linux-gnu/libcudnn.so:/usr/lib/aarch64-linux-gnu/libcudnn.so \
    -v /usr/share/doc/libcublas-dev \
    -v /usr/share/doc/libcublas10 \
    -v /usr/lib/aarch64-linux-gnu/libcublas*:/usr/lib/aarch64-linux-gnu/libcublas* \
    -v /usr/lib/aarch64-linux-gnu/stubs:/usr/lib/aarch64-linux-gnu/stubs \
    --volume='/home/jetson/.Xauthority:/root/.Xauthority:rw' \
    --volume='/tmp/.X11-unix/:/tmp/.X11-unix' \
    --volume='/home/jetson/projects/frc-ros:/home/projects/frc-ros' \
    arm64v8/ros:humble-perception
