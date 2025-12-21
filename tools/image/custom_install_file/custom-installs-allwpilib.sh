#!/bin/sh
apt update
apt install -y openjdk-17-jdk protobuf-compiler libprotobuf-dev ninja-build

git clone https://github.com/wpilibsuite/allwpilib.git

# Build allwpilib using cmake
cd allwpilib && mkdir build-cmake
cmake --preset default && cd build-cmake
cmake --build . --parallel 3 --target wpimath  # build wpimath first to avoid timeout
cmake --build . --parallel 3

sudo cmake --build . --target install
ldconfig
cd / && rm -rf /allwpilib
