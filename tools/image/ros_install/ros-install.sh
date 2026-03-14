#!/bin/bash

##### Set Locale to UTF-8
locale  # check for UTF-8

sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# ensure env is available for all sessions in the container
sudo tee /etc/profile.d/locale.sh >/dev/null <<'EOF'
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
EOF
sudo chmod 644 /etc/profile.d/locale.sh
# apply to current shell
source /etc/profile.d/locale.sh

locale  # verify settings

##### Enable required repositories
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

##### Install ROS2 Apt Source
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

##### Install development tools
sudo apt update && sudo apt install ros-dev-tools -y

##### Install ROS2
sudo apt install ros-$ROS_DISTRO-desktop-full -y
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp -y
sudo apt install ros-$ROS_DISTRO-rmw-zenoh-cpp -y

##### Initialize rosdep
sudo rosdep init
