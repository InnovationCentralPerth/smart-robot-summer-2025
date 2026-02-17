#!/bin/bash

# Kill any existing camera processes to free the device
killall -9 python3 2>/dev/null

# Source ROS 2 setup
source /opt/ros/jazzy/setup.bash

echo "Starting COMBINED Object Detection..."
echo "- VPU: MobileNet SSD (Standard Objects)"
echo "- CPU: TFLite Classifier (Custom Trained Items)"
echo "- Depth: Laser Rangefinder active."
echo "Press 'q' in the window to quit."

# Run the combined script
python3 "/home/icp/icp/Rasberry Pi Side/Vision/src/distance_detection_combined.py"
