#!/bin/bash
apt update

# Add ROS to sources list
#sudo add-apt-repository universe
#sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
#echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

#apt update

git clone --recurse-submodules https://github.com/osqp/osqp -b v0.6.2
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
cmake --build . -j6
cmake --build . --target install -j6
cd ../..
rm -rf osqp

# install casadi. TODO(HaoruXue): Add back -DWITH_OPENCL=ON -DWITH_CLANG=ON
apt install -y gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends
#source /opt/ros/humble/setup.bash
apt install -y --no-install-recommends coinor-libipopt-dev libslicot-dev
cd /opt && git clone https://github.com/casadi/casadi.git -b 3.6.4
cd casadi && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DWITH_IPOPT=ON -DWITH_SLICOT=ON -DWITH_LAPACK=ON -DWITH_QPOASES=ON -DWITH_OSQP=ON .. && make -j8
make install -j8
cd /opt && rm -rf casadi

#git clone --recurse-submodules https://github.com/grpc/grpc.git -b v1.54.1
#cd grpc
#mkdir build
#cd build
#cmake -DgRPC_INSTALL=ON -DgRPC_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local .. 
#cmake --build .
#cmake --build . --target install
#cd ../..
#rm -rf grpc

apt install -y cargo llvm-dev libclang-dev
cd /opt && git clone -b 0.11.0-dev-87-g6aa7bcb https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds.git
cd zenoh-plugin-ros2dds
cargo build --release
mv target/release/zenoh-bridge-ros2dds /usr/local/bin/
mv target/release/libzenoh_plugin_ros2dds.so /usr/local/lib/
cd ../
rm -rf zenoh-plugin-ros2dds

apt install -y ros-$ROS_DISTRO-librealsense2*

wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | sudo bash

cd /tmp
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build && cd build
cmake .. && make -j4
sudo make install
cd /tmp
rm -rf Livox-SDK2

apt clean
rm -rf /var/lib/apt/lists/*
