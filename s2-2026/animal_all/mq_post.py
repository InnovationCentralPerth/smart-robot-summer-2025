import paho.mqtt.client as mqtt
import time

client = mqtt.Client()
client.connect("localhost", 1883, 60)  # Replace with broker IP if needed

# Publish messages every 2 seconds
while True:
    client.publish("test/topic", "Hello MQTT!")
    print("Message sent")
    time.sleep(2)
