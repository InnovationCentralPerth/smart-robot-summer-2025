import json, time, paho.mqtt.client as mqtt

HOST, PORT = "localhost", 1883
CMD, STATUS, POS = "stacker/command", "stacker/status", "stacker/positions"

def on_message(client, userdata, msg):
    if msg.topic == CMD:
        command = json.loads(msg.payload.decode())
        print("Received command:", command)
        client.publish(STATUS, '"BUSY"')
        # Simulate work
        time.sleep(2)
        # Echo new positions (here just mirror the command)
        client.publish(POS, json.dumps(command))
        client.publish(STATUS, '"DONE"')

client = mqtt.Client()
client.on_message = on_message
client.connect(HOST, PORT, 60)
client.subscribe(CMD)
client.loop_forever()
