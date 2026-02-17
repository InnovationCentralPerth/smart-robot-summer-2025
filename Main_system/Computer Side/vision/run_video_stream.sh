#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
PYTHON_SCRIPT="$SCRIPT_DIR/video_receiver.py"

echo "-------------------------------------------------------"
echo "Initializing Remote Video Stream (OAK-D Lite)"
echo "Listening on port 5005..."
echo "-------------------------------------------------------"

# Check if python script exists
if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "ERROR: $PYTHON_SCRIPT not found."
    exit 1
fi

# Kill any existing instances to avoid "Address already in use"
pkill -f "video_receiver.py" 2>/dev/null

# Run the python script
# Note: Ensure opencv-python and numpy are installed in WSL (pip install opencv-python numpy)
python3 "$PYTHON_SCRIPT"
