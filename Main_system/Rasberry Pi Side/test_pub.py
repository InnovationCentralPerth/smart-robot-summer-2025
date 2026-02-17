import paho.mqtt.client as mqtt
import time

BROKER = "localhost"
TOPIC = "animal_game/direct"
MESSAGE = "A,B"

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.connect(BROKER, 1883, 60)
client.publish(TOPIC, MESSAGE)
print(f"Published '{MESSAGE}' to '{TOPIC}'")
time.sleep(1) # Ensure message sends
client.disconnect()
