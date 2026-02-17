# Remote Vision System (PC Side)

This folder contains the scripts to visualize the SLAM data and raw video stream from your Raspberry Pi robot on your Windows PC (via WSL2).

## Prerequisites

1.  **Windows Subsystem for Linux (WSL2)** running Ubuntu 24.04.
2.  **ROS 2 Jazzy** installed on WSL2.
3.  **Python 3** with `opencv-python` and `numpy`.
4.  **Windows Firewall Rule** allowing UDP traffic on ports 7400-7500 (for ROS 2) and 5005 (for Video).

## Setup

1.  Open your Ubuntu terminal.
2.  Navigate to this directory:
    ```bash
    cd "/mnt/c/Users/$(whoami)/OneDrive/Desktop/Computer Side/vision"
    ```
3.  Run the dependency installer:
    ```bash
    ./install_deps.sh
    ```

## Usage

### 1. View 3D SLAM Map
To see the 3D map being built by RTAB-Map:

1.  Edit `run_slam_viewer.sh` if your Raspberry Pi IP address has changed (default is `192.168.6.28`).
2.  Run:
    ```bash
    ./run_slam_viewer.sh
    ```

### 2. View Raw Video Stream
To see the raw video feed from the OAK-D Lite camera:

1.  Run:
    ```bash
    ./run_video_stream.sh
    ```

## Files

-   `run_slam_viewer.sh`: Configures ROS 2 networking and launches `rtabmap_viz`.
-   `run_video_stream.sh`: Launches the Python video receiver.
-   `video_receiver.py`: Python script that listens for UDP video packets and displays them using OpenCV.
-   `install_deps.sh`: Helper script to install necessary Python and ROS 2 packages.
