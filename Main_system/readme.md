# Smart Robot Summer 2025

**Author:** Dave Mitchell Qiu  
**Organization:** Innovation Central Perth

This repository contains the complete source code for the "Smart Robot Summer 2025" project, integrating a robotic arm, computer vision (OAK-D Lite), AI/LLM control, and an interactive game.

## 📂 Repository Structure

The project is divided into four main documentation areas:

### 1. [Computer Side Control](./Computer%20Side)
This folder contains the main control software for Windows/Linux PC:
-   **AI & Logic:** Integration with OpenAI (LLM) for high-level decision making.
-   **Frontend:** A user interface (Smart Stacker) to interact with the robot.
-   **Communication:** Sends commands to the robot via MQTT.

👉 **[View Computer Side Instructions](./Computer%20Side/readme.md)**

### 2. [Computer Side Vision](./Computer%20Side/vision)
This folder contains the **PC-side scripts** for visualizing the robot's data via **WSL2 (Ubuntu 24.04)**:
-   **3D SLAM Map Viewer:** Visualizes the RTAB-Map data being built by the robot using ROS 2 Jazzy.
-   **Video Receiver:** Displays the raw video stream from the OAK-D Lite camera.
-   **Prerequisites:** Requires WSL2, ROS 2 Jazzy, and specific firewall rules for UDP traffic.

👉 **[View PC Vision Instructions](./Computer%20Side/vision/README.md)**

### 3. [Rasberry Pi Side](./Rasberry%20Pi%20Side)
This folder contains the software to be run on the robot's onboard Raspberry Pi (Ubuntu/Debian):
-   **Hardware Control:** Serial communication with the Arduino/Braccio robot arm.
-   **Game Logic:** The "Animal Game" logic and state management.
-   **MQTT Broker:** Hosts the local communication network for the system.

👉 **[View Raspberry Pi Instructions](./Rasberry%20Pi%20Side/README.md)**

### 4. [Vision System (On Raspberry Pi)](./Rasberry%20Pi%20Side/Vision)
Located inside the Raspberry Pi folder, this specialized module handles all camera-based tasks:
-   **Object Detection:** Real-time identification using MobileNet or custom TFLite models.
-   **3D SLAM:** Simultaneous Localization and Mapping using RTAB-Map and ROS 2.
-   **Remote Streaming:** Transmission of video feeds to the PC client.

👉 **[View Vision System Instructions](./Rasberry%20Pi%20Side/Vision/README.md)**

## 🚀 Quick Start Overview

To get the full system running, you generally need to:

1.  **Set up the Raspberry Pi:**
    *   Navigate to `Rasberry Pi Side/` and run `./setup.sh`.
    *   Start the core game system with `./start_game.sh`.

2.  **Start the Vision System (Optional):**
    *   Open a new terminal on the Pi.
    *   Navigate to `Rasberry Pi Side/Vision/` and run `./setup.sh` (first time only).
    *   Run `./scripts/start_distance.sh` for detection or `./scripts/start_slam_local.sh` for mapping.

3.  **Set up the PC (Control):**
    *   Navigate to `Computer Side/` and run `setup.bat`.
    *   Configure your `.env` file with API keys and the Pi's IP address.
    *   Run `run.bat` to start the control interface.

4.  **Set up the PC (Vision Viewer - Optional):**
    *   Open WSL2 (Ubuntu 24.04).
    *   Navigate to `Computer Side/vision/` and run `./install_deps.sh`.
    *   Run `./run_slam_viewer.sh` to see the map or `./run_video_stream.sh` to see the camera.

## ✨ Key Features

*   **Smart Control:** Natural language processing to control the robot arm.
*   **Computer Vision:** Real-time object detection and 3D mapping (SLAM).
*   **IoT Connectivity:** Uses MQTT for robust communication between the PC and Robot.
*   **Interactive Game:** "Animal Game" mode demonstrating complex logic and interaction.

---
*For detailed documentation, please refer to the README files linked above.*
