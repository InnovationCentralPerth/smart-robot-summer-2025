# Vision Project: OAK-D Lite SLAM & Detection

This repository contains a complete suite for 3D SLAM mapping, Object Detection, and Distance Measurement using the OAK-D Lite camera on a Raspberry Pi (or any Linux system).

## 📂 Project Structure

- **scripts/**: Bash scripts to start the various modes.
- **src/**: Python source code for detection and processing.
- **launch/**: ROS 2 launch files.
- **pc_client/**: Python scripts to run on a Windows/Mac PC for remote viewing.
- **models/**: Place your custom trained `.tflite` or `.blob` files here.

## 🚀 Quick Start

### 1. Installation
Run the setup script to install dependencies:
```bash
./setup.sh
```

### 2. Distance & Object Detection (No ROS needed)
*   **Standard Mode (MobileNet - 20 Classes):**
    ```bash
    ./scripts/start_distance.sh
    ```
*   **Custom Mode (Your trained TFLite model):**
    Place your `custom_model.tflite` in the `models/` folder, then run:
    ```bash
    ./scripts/start_distance_custom.sh
    ```
*   **Remote Mode (Stream to PC):**
    Run on Pi: `./scripts/start_distance_remote.sh <PC_IP>`
    Run on PC: `python pc_client/pc_video_receiver_dual.py`

### 3. 3D SLAM Mapping (Requires ROS 2 Jazzy)
*   **Local UI (View on Pi):**
    ```bash
    ./scripts/start_slam_local.sh
    ```
*   **Remote UI (View on Laptop):**
    Run on Pi: `./scripts/start_slam_remote.sh <LAPTOP_IP>`
    Run on Laptop: Follow the instructions printed by the script.
*   **Fastest AI (30FPS Face Tracking - No Map):**
    ```bash
    ./scripts/start_slam_fast.sh
    ```

### 4. Viewing & Exporting Scans
*   **View Scan:** `./scripts/view_scan.sh`
*   **Export to PLY:** `./scripts/export_scan.sh` (Saves to `exports/`)

## 🔧 Requirements
*   **Hardware:** OAK-D Lite, Raspberry Pi 4/5.
*   **OS:** Ubuntu 24.04 (Noble) or Raspberry Pi OS (Bookworm).
*   **ROS 2:** Jazzy Jalisco (Automatically installed by `setup.sh` if missing).
*   **Python:** 3.10+ (Requires `depthai < 3.0.0` for spatial detection features).

## 🚀 Recent Fixes
*   **Permissions:** Automatically sets up `udev` rules for OAK-D Lite access.
*   **Path Correction:** Source code updated to use absolute paths for model loading, avoiding errors when scripts are run from different directories.
*   **SLAM Compatibility:** Full integration with ROS 2 Jazzy and RTAB-Map for live 3D mapping.

## 🧠 Custom Training
To use your own AI model:
1.  Train an Object Detection model (YOLO/SSD) or Classifier in Edge Impulse.
2.  Export as **TensorFlow Lite (float32)** or **OpenVINO**.
3.  Rename the file to `custom_model.tflite` (or `.blob`).
4.  Copy it to the `models/` folder.
