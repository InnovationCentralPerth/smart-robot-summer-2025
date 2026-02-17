# Smart Robot Control - Computer Side

This folder contains the software required to control the Smart Robot from your computer.

## Prerequisites

1.  **Python 3.10 or newer**: You must have Python installed on your computer.
    *   Download it here: [https://www.python.org/downloads/](https://www.python.org/downloads/)
    *   **IMPORTANT:** During installation, make sure to check the box that says **"Add Python to PATH"**.

## How to Install

1.  Open this folder (`Computer Side`).
2.  Double-click the file named **`setup.bat`**.
3.  A black window will appear and install the necessary libraries (like `paho-mqtt`, `requests`, etc.).
4.  Wait for it to say "Dependencies installed successfully" or ask you to press any key to continue.

## Configuration

Before running the application, you can change settings in the **`.env`** file (open it with Notepad):

*   **`MODEL`**: The AI model to use (e.g., `gpt-4o-mini`).
*   **`MQTT_HOST`**: The IP address of your robot or MQTT broker (use `localhost` if it's on this computer).
*   **`OPENAI_API_KEY`**: Your API key for AI features.

## How to Run the Application

1.  Double-click the file named **`run.bat`**.
2.  The application will start, launching the Smart Stacker Frontend.

## Troubleshooting

*   **"Python is not recognized..."**: This means Python is not installed or was not added to your PATH. Reinstall Python and ensure the "Add to PATH" checkbox is selected.
*   **Missing modules**: If the application closes immediately or says "Module not found", try running `setup.bat` again to ensure all requirements are installed.
