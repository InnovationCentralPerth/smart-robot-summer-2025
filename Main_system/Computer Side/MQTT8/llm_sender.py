import paho.mqtt.client as mqtt
import ast
import os
from dotenv import load_dotenv
from pathlib import Path

# Load .env from parent directory
env_path = Path(__file__).resolve().parent.parent / '.env'
load_dotenv(dotenv_path=env_path)

MQTT_BROKER = os.getenv("MQTT_HOST", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
TOPIC = os.getenv("COMMAND_TOPIC", "stacker/command")
TOPIC_DIRECT = "stacker/direct"
STATUS_TOPIC = os.getenv("STATUS_TOPIC", "stacker/status")

# --- Connect to MQTT broker ---
client = mqtt.Client()

def on_status(client, userdata, msg):
    print(f"[STATUS from MQTT] {msg.payload.decode()}")

client.connect(MQTT_BROKER, MQTT_PORT, 60)

client.loop_start()

# Subscribe to status topic
client.subscribe(STATUS_TOPIC)
client.message_callback_add(STATUS_TOPIC, on_status)

print("LLM Sender (terminal version)")
print("========This shows the middleground messaging can input========\n")

while True:
    user_input = input("Input: ").strip()
    if user_input.lower() == "exit":
        break

    # Try parsing as dict
    try:
        cmd_dict = ast.literal_eval(user_input)
        if isinstance(cmd_dict, dict):
            client.publish(TOPIC, str(cmd_dict))
            print(f"[Auto Rearrange] Sent: {cmd_dict}\n")
            continue
    except:
        pass

    # Otherwise direct command
    if user_input:
        client.publish(TOPIC_DIRECT, user_input)
        print(f"[Direct Arduino] Sent: {user_input}\n")

# Cleanup
client.loop_stop()
client.disconnect()
