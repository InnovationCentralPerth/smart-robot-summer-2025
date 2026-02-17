#!/bin/bash

# Force ROS 2 to use standard discovery
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Source ROS 2 Jazzy environment
source /opt/ros/jazzy/setup.bash

# Clean up old ROS 2 processes
killall -9 component_container rtabmap_viz robot_state_publisher 2>/dev/null

# Archive the old RTAB-Map database instead of deleting it
if [ -f ~/.ros/rtabmap.db ]; then
    mkdir -p "/home/icp/icp/Rasberry Pi Side/Vision/src/scans"
    TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)
    mv ~/.ros/rtabmap.db "/home/icp/icp/Rasberry Pi Side/Vision/src/scans/scan_$TIMESTAMP.db"
    echo "Previous scan archived to: "/home/icp/icp/Rasberry Pi Side/Vision/src/scans/scan_$TIMESTAMP.db""
fi

# Run the Ultralight SLAM configuration
echo "Starting OAK-D Lite SLAM (Local UI Mode)..."
echo "Movement Tip: Move VERY SLOWLY until the red screen turns clear."
ros2 launch "/home/icp/icp/Rasberry Pi Side/Vision/src/optimized_slam.launch.py" camera_model:=OAK-D-LITE viz:=true
