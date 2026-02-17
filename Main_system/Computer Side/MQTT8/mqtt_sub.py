import paho.mqtt.client as mqtt
import os
from dotenv import load_dotenv
from pathlib import Path

# Load .env from parent directory
env_path = Path(__file__).resolve().parent.parent / '.env'
load_dotenv(dotenv_path=env_path)

# MQTT broker settings
BROKER = os.getenv("MQTT_HOST", "localhost")
PORT = int(os.getenv("MQTT_PORT", 1883))

# Topics to subscribe to
TOPICS = [
    ("stacker/positions", 0),

    ("stacker/command", 0),
    ("stacker/status", 0)
]

# Callback when connected to broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[INFO] Connected to MQTT Broker")
        # Subscribe to all topics
        client.subscribe(TOPICS)
        print(f"[INFO] Subscribed to topics: {[t[0] for t in TOPICS]}")
        print("========This shows the MQTT messages========")
    else:
        print(f"[ERROR] Failed to connect, return code {rc}")

# Callback when message is received
def on_message(client, userdata, msg):
    print(f"[MQTT] {msg.topic} | {msg.payload.decode()}")

# Create client instance
client = mqtt.Client()

# Attach callbacks
client.on_connect = on_connect
client.on_message = on_message

# Connect to broker
client.connect(BROKER, PORT, keepalive=60)

# Blocking loop to process network traffic and callbacks
client.loop_forever()
