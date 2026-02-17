#!/bin/bash
cd "$(dirname "$0")"

# Check if required packages are installed (basic check)
if ! python3 -c "import paho.mqtt.client, serial" &> /dev/null; then
    echo "Missing dependencies. Attempting to run setup.sh..."
    if [ -f "./setup.sh" ]; then
        chmod +x ./setup.sh
        sudo ./setup.sh
    else
        echo "Error: setup.sh not found!"
        exit 1
    fi
fi

python3 run.py
