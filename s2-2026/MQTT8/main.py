import paho.mqtt.client as mqtt
import ast
from gui import create_gui
from command_processor import process_command, execute_moves
from serial_comm import send_command

MQTT_BROKER = "localhost"
TOPIC_COMMANDS = "animal_game/commands"
TOPIC_DIRECT = "animal_game/direct"
TOPIC_STATUS = "animal_game/status"

# --- MQTT client setup ---
client = mqtt.Client()
client.connect(MQTT_BROKER, 1883, 60)
client.loop_start()  # start background MQTT loop

# --- Status callback for Arduino and GUI ---
def status_callback(msg_status):
    print(f"[STATUS] {msg_status}")          # prints in LLM Sender terminal
    client.publish(TOPIC_STATUS, msg_status) # publishes to LLM Sender

# --- Set send_command callback ---
send_command.status_callback = status_callback

# --- MQTT callbacks ---
def on_connect(client, userdata, flags, rc):
    print("[MQTT] Connected with result code", rc)
    client.subscribe(TOPIC_COMMANDS)
    client.subscribe(TOPIC_DIRECT)

def on_message(client, userdata, msg):
    payload = msg.payload.decode().strip()
    print(f"[MQTT] Received on {msg.topic}: {payload}")

    try:
        if msg.topic == TOPIC_COMMANDS:
            cmd_dict = ast.literal_eval(payload)
            if isinstance(cmd_dict, dict):
                moves = process_command(cmd_dict, positions)
                execute_moves(moves, positions, update_display)

        elif msg.topic == TOPIC_DIRECT:
            send_command(payload, wait_done=True)
            if "," in payload and len(payload.split(",")) == 2:
                src, dst = payload.split(",")
                src, dst = src.strip(), dst.strip()
                if src in positions and dst in positions and positions[src]:
                    positions[dst] = positions[src]
                    positions[src] = None
                    update_display()
            print(f"[DEBUG] Direct command sent: {payload}")

    except Exception as e:
        print(f"[MQTT] Error: {e}")

client.on_connect = on_connect
client.on_message = on_message

# --- Create GUI after MQTT is ready ---
root, update_display, positions = create_gui(status_callback=status_callback)

# --- Start GUI mainloop ---
root.mainloop()
