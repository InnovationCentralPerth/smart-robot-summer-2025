#!/bin/bash

# Kill any existing camera processes to free the device
killall -9 python3 2>/dev/null

# Source ROS 2 setup
source /opt/ros/jazzy/setup.bash

echo "Starting Object Detection & Distance Measurement (MobileNet SSD)..."
echo "Press 'q' in the window to quit."

# Run the standard detection script
python3 "/home/icp/icp/Rasberry Pi Side/Vision/src/distance_detection.py"
