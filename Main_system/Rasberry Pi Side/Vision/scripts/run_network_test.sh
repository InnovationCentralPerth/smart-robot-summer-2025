#!/bin/bash
# Force direct connection to the PC
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS="192.168.6.11"

# Source ROS
source /opt/ros/jazzy/setup.bash

echo "Starting Network Test on Pi (Directing data to 192.168.6.11)..."
python3 "/home/icp/icp/Rasberry Pi Side/Vision/src/network_test.py"
