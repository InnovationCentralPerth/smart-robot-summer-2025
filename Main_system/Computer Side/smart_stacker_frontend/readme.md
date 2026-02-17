# Smart Stacker Frontend

LLM-powered voice and text to robot command interpreter that communicates with the stacker over MQTT.

## Prerequisites
- Python 3.12 (recommended) with `pip`
- MQTT broker (e.g., Mosquitto) reachable on TCP port 1883
- (Optional) API-compatible LLM server such as Ollama if you want model-backed parsing

## Set up Python
### macOS
```bash
# Install Python 3.12 (Homebrew)
brew install python@3.12
python3.12 -m pip install --upgrade pip virtualenv
python3.12 -m venv .venv
source .venv/bin/activate
```

### Windows
```powershell
# Install Python 3.12 from python.org installer (ensure "Add to PATH")
py -3.12 -m pip install --upgrade pip virtualenv
py -3.12 -m venv .venv
.\.venv\Scripts\activate
```

### Linux (Debian/Ubuntu)
```bash
sudo apt-get update
sudo apt-get install -y python3.12 python3.12-venv python3-pip
python3.12 -m venv .venv
source .venv/bin/activate
```

## Install Python requirements
From the repository root (with the virtual environment activated):
```bash
pip install -r requirements.txt
```

## Install and run an MQTT broker
You can point the frontend at any broker via `MQTT_HOST`/`MQTT_PORT`. To run one locally:

### macOS (Homebrew)
```bash
brew install mosquitto
brew services start mosquitto  # or: mosquitto -v
```

### Windows
- Install Mosquitto from https://mosquitto.org/download/ (choose the Windows installer).
- Start the broker from an elevated PowerShell:
```powershell
"C:\Program Files\mosquitto\mosquitto.exe" -v
```

### Linux (Debian/Ubuntu)
```bash
sudo apt-get install -y mosquitto
sudo systemctl enable --now mosquitto  # or: mosquitto -v
```

## Quick hardware-free test flow
Use the provided MQTT mock plus the CLI to verify end-to-end behavior without a robot.

1. **Start the broker** (see above). On another terminal, set environment variables to point at it (example shown for localhost):
```bash
export MQTT_HOST=localhost
export MQTT_PORT=1883
```

2. **Run the mock controller** to echo commands and publish status/positions:
```bash
python -m smart_stacker_frontend.mock_mqtt
```

3. **Run the frontend CLI** in a second terminal (safe mode by default):
```bash
python -m smart_stacker_frontend.main
```
   - Use `record`/`listen` to capture audio, enter a WAV file number, or type free text.
   - Add `--unsafe` to bypass safety validation if desired.

4. **Observe the exchange**: the mock prints received commands, publishes `BUSY → positions → DONE`, and the CLI displays `✅ Command executed` when it sees completion.

## Pointing to a remote or different LLM server
If you have an API-compatible LLM endpoint (e.g., remote Ollama), set:
```bash
export HOST_URL=http://<host>:11434
export MODEL=<model-name>
```
Then run the CLI as usual. Without an available LLM server, the frontend falls back to simple heuristics.

## Common environment variables
- `MQTT_HOST` / `MQTT_PORT`: broker location (default `192.168.1.101:1883`).
- `STACKER_TOPIC_COMMAND`, `STACKER_TOPIC_STATUS`, `STACKER_TOPIC_POSITIONS`: override topic names if your broker uses different ones.
- `HOST_URL`, `MODEL`: LLM server endpoint and model selection.

## Troubleshooting
- **Cannot connect to MQTT**: confirm broker is running and `MQTT_HOST`/`MQTT_PORT` match. Use `mosquitto_sub`/`mosquitto_pub` to sanity check.
- **Audio issues**: the CLI reports whether a microphone is available; you can still process existing WAV files without a mic.
- **LLM unavailable**: errors connecting to `HOST_URL` trigger a fallback heuristic so the CLI keeps working.
