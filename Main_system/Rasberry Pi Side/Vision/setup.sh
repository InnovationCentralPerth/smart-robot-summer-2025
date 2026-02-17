#!/bin/bash

echo "=== Vision Project Setup ==="

# Set up udev rules for OAK-D Lite
echo "Configuring camera permissions (udev rules)..."
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "Installing Python dependencies..."
# depthai < 3.0.0 is required for compatibility with MobileNetSpatialDetectionNetwork
pip install "depthai<3.0.0" opencv-python blobconverter numpy rerun-sdk ai-edge-litert --break-system-packages

echo "Checking ROS 2 installation..."
if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "ROS 2 Jazzy not found. Installing ROS 2 repository..."
    sudo apt update && sudo apt install -y curl gnupg2 lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    echo "Installing ROS 2 Jazzy and SLAM packages (Large Download)..."
    sudo apt install -y ros-jazzy-desktop ros-jazzy-depthai-ros ros-jazzy-rtabmap-ros
else
    echo "ROS 2 Jazzy found."
fi

echo "Making scripts executable..."
chmod +x scripts/*.sh

echo "Setup complete!"
