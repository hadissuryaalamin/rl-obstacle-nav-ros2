#!/usr/bin/env bash
# Installs ROS 2 Lyrical + Gazebo (Jetty, via ros_gz) + RL Python deps on Ubuntu 26.04.
set -e

# 1. ROS 2 apt repository
sudo apt update
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# 2. ROS 2 + Gazebo + package dependencies
sudo apt update
sudo apt install -y \
  ros-lyrical-desktop \
  ros-lyrical-ros-gz \
  ros-lyrical-xacro \
  ros-lyrical-robot-state-publisher \
  ros-dev-tools \
  python3-venv python3-pip

# 3. Python RL deps in a venv that can still see rclpy (system site-packages)
python3 -m venv --system-site-packages ~/venvs/tutor
~/venvs/tutor/bin/pip install --upgrade pip
~/venvs/tutor/bin/pip install stable-baselines3 gymnasium numpy

echo
echo "Done. Add to ~/.bashrc:"
echo "  source /opt/ros/lyrical/setup.bash"
