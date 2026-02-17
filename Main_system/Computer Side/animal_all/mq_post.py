import paho.mqtt.client as mqtt
import time
import os
from dotenv import load_dotenv
from pathlib import Path

# Load .env from parent directory
env_path = Path(__file__).resolve().parent.parent / '.env'
load_dotenv(dotenv_path=env_path)

MQTT_HOST = os.getenv("MQTT_HOST", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))

client = mqtt.Client()
client.connect(MQTT_HOST, MQTT_PORT, 60)

# Publish messages every 2 seconds
while True:

    client.publish("test/topic", "Hello MQTT!")
    print("Message sent")
    time.sleep(2)
