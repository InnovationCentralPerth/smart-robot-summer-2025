#!/bin/bash

# Ensure local bin is in PATH for rerun
export PATH=$PATH:$HOME/.local/bin

# Clean up
killall -9 python3 component_container rtabmap_viz robot_state_publisher depthai-viewer rerun 2>/dev/null

echo "Starting FASTEST AI SLAM (Rerun 3D Visualization)..."
echo "This uses the 'Rerun' engine to visualize 3D Point Clouds at high speed."
echo "Note: If the window doesn't open, ensure you have a display connected."

python3 "/home/icp/icp/Rasberry Pi Side/Vision/src/fast_rerun.py"
