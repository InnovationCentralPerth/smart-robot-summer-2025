#!/bin/bash

# Configuration
# Change this IP to match your Raspberry Pi's IP address
PI_IP="192.168.6.28"
# Change this if you use a different ROS_DOMAIN_ID
DOMAIN_ID=0

echo "-------------------------------------------------------"
echo "Initializing Remote Eyes (RTAB-Map Viewer)"
echo "Connecting to Pi at: $PI_IP"
echo "-------------------------------------------------------"

# 1. Source ROS 2 Jazzy
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "ERROR: ROS 2 Jazzy not found at /opt/ros/jazzy/setup.bash"
    echo "Please ensure you have installed ros-jazzy-desktop"
    exit 1
fi

# 2. Set Networking Environment Variables
export ROS_DOMAIN_ID=$DOMAIN_ID
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS="$PI_IP"

# 3. FastDDS P2P Configuration (Ensures stable connection over WiFi)
cat << XML > /tmp/fastdds_p2p.xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastrtps_profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>CustomUDPTransport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="P2PParticipant">
        <rtps>
            <builtin>
                <initialPeersList>
                    <locator>
                        <udpv4>
                            <address>$PI_IP</address>
                        </udpv4>
                    </locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
</profiles>
XML
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_p2p.xml

# 4. Launch rtabmap_viz
echo "Launching Viewer... (Close window to exit)"
ros2 run rtabmap_viz rtabmap_viz --ros-args \
  -p subscribe_map_data:=true \
  -p frame_id:=oak \
  -r mapData:=/mapData \
  -r odom:=/odom
