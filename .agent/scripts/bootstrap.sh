#!/bin/bash
set -e

# [TASK-003] Bootstrap Automation
# This script aligns with the official ROS 2 Jazzy installation guide:
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

echo "Bootstrapping ROS 2 Agent Workspace..."

# 1. Locale Check
echo "Checking locale..."
if ! locale | grep -q 'LC_ALL=.*UTF-8'; then
    echo "Locale does not appear to support UTF-8. Attempting to set it..."
    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    echo "Locale set to en_US.UTF-8"
else
    echo "Locale looks good."
fi

# 2. Setup Sources
echo "Setting up sources..."
# Enable Universe
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

# Install curl
sudo apt update && sudo apt install -y curl

# Install ros2-apt-source (Handles GPG/Repo automatically)
echo "Installing ros2-apt-source..."
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
export ROS_APT_SOURCE_VERSION
if [ -z "${ROS_APT_SOURCE_VERSION}" ]; then
    echo "Error: Failed to determine ROS apt source version from GitHub API." >&2
    exit 1
fi
curl -fL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")_all.deb"

if [ ! -s /tmp/ros2-apt-source.deb ]; then
    echo "Error: Failed to download ros2-apt-source .deb or file is empty." >&2
    exit 1
fi
sudo dpkg -i /tmp/ros2-apt-source.deb
rm /tmp/ros2-apt-source.deb

# 3. Install Dependencies
echo "Installing ROS 2 and development tools..."
sudo apt update
sudo apt install -y ros-dev-tools

# Check for ROS 2 Base
if ! dpkg -l | grep -q ros-jazzy-ros-base; then
    echo "Installing ros-jazzy-ros-base..."
    sudo apt install -y ros-jazzy-ros-base
else
    echo "ros-jazzy-ros-base is already installed."
fi

# 3b. Install Python Dependencies (Required for message generation)
echo "Installing Python dependencies..."
sudo apt install -y python3-pip
pip3 install --break-system-packages empy==3.3.4 catkin_pkg lark

# 4. Initialize rosdep
echo "Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

rosdep update

# 5. Codespace Configuration
if [ "${CODESPACES}" = "true" ]; then
    echo "Detected GitHub Codespaces environment."
    echo "Converting SSH URLs to HTTPS in .repos files..."
    find configs -name "*.repos" -print0 | xargs -0 sed -i 's|git@github.com:|https://github.com/|g'
    echo "URL conversion complete."
fi

echo "Bootstrap complete! You can now run './.agent/scripts/setup.sh <workspace_name>'."
