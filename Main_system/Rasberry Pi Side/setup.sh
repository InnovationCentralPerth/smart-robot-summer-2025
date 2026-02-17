#!/bin/bash

# Update package list
echo "Updating package list..."
sudo apt-get update

# Install system dependencies
# python3-tk is for the GUI
# xvfb and scrot are for virtual displays and screenshots
# xterm is used for multi-window component management
echo "Installing system dependencies..."
sudo apt-get install -y python3 python3-pip python3-tk xterm xvfb scrot

# Install Python dependencies from requirements.txt
echo "Installing Python libraries..."
# paho-mqtt version 2.x is required by the updated code
pip3 install "paho-mqtt>=2.0.0" --break-system-packages
pip3 install -r requirements.txt --break-system-packages

# Add user to dialout group for Serial access (Arduino)
echo "Adding user to dialout group..."
sudo usermod -a -G dialout $USER

echo "Installation complete!"
echo "Please reboot or log out and back in for group changes (serial permissions) to take effect."
echo "You can then run ./start_game.sh"
