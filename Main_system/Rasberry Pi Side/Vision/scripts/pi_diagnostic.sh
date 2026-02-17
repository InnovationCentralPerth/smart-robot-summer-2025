#!/bin/bash
source /opt/ros/jazzy/setup.bash

echo "=== ROS 2 Local Diagnostic ==="
echo "1. Starting Talker in background..."
ros2 run demo_nodes_cpp talker > /tmp/talker.log 2>&1 &
TALKER_PID=$!

sleep 5

echo "2. Checking if Pi can see the topic locally..."
TOPICS=$(ros2 topic list)
echo "Local topics found:"
echo "$TOPICS"

if echo "$TOPICS" | grep -q "/chatter"; then
    echo "SUCCESS: Pi can see its own topics."
else
    echo "FAILURE: Pi cannot see its own topics locally."
fi

echo "3. Checking if type information is available locally..."
ros2 topic info /chatter

kill $TALKER_PID
echo "=== End Diagnostic ==="
