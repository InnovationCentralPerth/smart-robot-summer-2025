# Animal Game - Raspberry Pi Side

This folder contains the software to run the Animal Game on a Raspberry Pi or Ubuntu machine.

## Prerequisites

- Ubuntu 24.04 (or similar Debian-based Linux)
- Python 3
- Internet connection (for installing dependencies)

## Installation

1. Open a terminal in this folder.
2. Run the setup script to install all dependencies:
   ```bash
   chmod +x setup.sh
   ./setup.sh
   ```
   **Note:** This script will:
   - Install required system packages (like `python3-tk`).
   - Install Python libraries (`paho-mqtt`, `amqtt`, `pyserial`).
   - Add your user to the `dialout` group (required for Arduino communication).
   
3. **Important:** If this is the first time running the setup, you must **restart your computer** or log out and log back in for the group changes to take effect.

## Usage

To start the game, run:

```bash
./start_game.sh
```

This will automatically:
1. Start a local MQTT broker (using Python `amqtt`).
2. Launch the game components.

**Display Modes:**
- **With GUI (Desktop):** If a graphical environment and `xterm` are available, it will launch separate terminal windows for each component.
- **Headless (No Monitor/SSH):** If no display is found, the system now automatically switches to **Headless Mode**. GUI components (like `gui.py` and `mqtt_gui.py`) will remain functional in the background without crashing, and logs will be printed to the console.

## Code Updates
- **MQTT v2:** The codebase has been updated to support `paho-mqtt` version 2.0+. All MQTT connection callbacks now include the required `properties` argument.
- **GUI Fallback:** Tkinter windows now have a try/except fallback to prevent application crashes when running via SSH or without an X server.

## Configuration

- **Serial Port:** The default serial port for the Arduino is set to `/dev/ttyACM0`. If your Arduino is on a different port (e.g., `/dev/ttyUSB0`), edit `serial_comm.py` and change the `SERIAL_PORT` variable.
- **MQTT Broker:** The system uses a built-in Python broker on `localhost:1883`. No external broker installation is required.

## Troubleshooting

- **Serial Error:** If you see "Error opening serial port", make sure your Arduino is connected and you have rebooted after running `setup.sh`.
- **Permission Denied:** Ensure scripts are executable (`chmod +x *.sh`).
