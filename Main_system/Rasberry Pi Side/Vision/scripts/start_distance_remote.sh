#!/bin/bash

# Default to known Laptop IP
DEFAULT_IP="192.168.6.11"
LAPTOP_IP=${1:-$DEFAULT_IP}

# Kill any existing camera processes to free the device
killall -9 python3 2>/dev/null

# Source ROS 2 setup just in case
source /opt/ros/jazzy/setup.bash

echo "Starting REMOTE VIDEO STREAMING (Headless + UDP Video + Depth)..."
echo "Streaming 80-Class Video Feed to PC at $LAPTOP_IP:5005"
echo "Streaming Depth Heatmap to PC at $LAPTOP_IP:5006"
echo "Run 'pc_video_receiver_dual.py' on your Windows PC to see both."

# Run the remote detection script with the target IP
python3 "/home/icp/icp/Rasberry Pi Side/Vision/src/distance_detection_streaming.py" $LAPTOP_IP
