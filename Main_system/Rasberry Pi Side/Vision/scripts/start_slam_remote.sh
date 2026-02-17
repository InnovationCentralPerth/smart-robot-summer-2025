#!/bin/bash

# Default to your known Laptop IP if not provided
DEFAULT_IP="192.168.6.11"
LAPTOP_IP=${1:-$DEFAULT_IP}

# Force ROS 2 to use standard discovery
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS="$LAPTOP_IP"

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

# Get IP Address
PI_IP=$(hostname -I | awk '{print $1}')

# Run the Ultralight SLAM configuration
echo "--------------------------------------------------------"
echo "Starting REMOTE SLAM (Pi -> Laptop)"
echo "Pi IP:     $PI_IP"
echo "Laptop IP: $LAPTOP_IP (Target)"
echo "--------------------------------------------------------"
echo "ON YOUR LAPTOP (Run these commands):"
echo ""
echo "export ROS_DOMAIN_ID=0"
echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET"
echo "export ROS_STATIC_PEERS=$PI_IP"
echo "source /opt/ros/jazzy/setup.bash"
echo "ros2 run rtabmap_viz rtabmap_viz --ros-args -p subscribe_map_data:=true -p frame_id:=oak -r mapData:=/mapData -r odom:=/odom"
echo "--------------------------------------------------------"

ros2 launch "/home/icp/icp/Rasberry Pi Side/Vision/src/optimized_slam.launch.py" camera_model:=OAK-D-LITE viz:=false
