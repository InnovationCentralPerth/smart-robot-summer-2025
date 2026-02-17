#!/bin/bash

# Source ROS 2 if available to check for ros2 command
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

echo "Installing Python dependencies..."
# Ensure pip is installed
if ! command -v pip &> /dev/null; then
    if command -v pip3 &> /dev/null; then
        PIP_CMD=pip3
    else
        echo "pip not found. Installing python3-pip..."
        sudo apt update && sudo apt install -y python3-pip
        PIP_CMD=pip3
    fi
else
    PIP_CMD=pip
fi

$PIP_CMD install opencv-python numpy --break-system-packages

echo "Installing ROS 2 dependencies..."
if command -v ros2 &> /dev/null; then
    echo "ROS 2 detected. Ensuring rtabmap-ros is installed..."
    # We use sudo here, so it might ask for a password
    if [ -f "/etc/apt/sources.list.d/ros2.list" ]; then
        sudo apt update
        sudo apt install -y ros-jazzy-rtabmap-ros ros-jazzy-rmw-cyclonedds-cpp python3-opencv
    else
        echo "ROS 2 sources list not found, but ros2 command exists. Assuming custom install."
    fi
else
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
         echo "ROS 2 setup file found but ros2 command failed. Sourcing and retrying..."
         source /opt/ros/jazzy/setup.bash
         if command -v ros2 &> /dev/null; then
             sudo apt update
             sudo apt install -y ros-jazzy-rtabmap-ros ros-jazzy-rmw-cyclonedds-cpp python3-opencv
         else
             echo "ROS 2 Jazzy not found. Please install it first following the official guide."
         fi
    else
        echo "ROS 2 Jazzy not found at /opt/ros/jazzy/setup.bash. Please install it first."
    fi
fi

echo "Done!"
