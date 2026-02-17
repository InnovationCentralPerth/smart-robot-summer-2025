import paho.mqtt.client as mqtt
import ast

MQTT_BROKER = "localhost"
TOPIC = "animal_game/commands"
TOPIC_DIRECT = "animal_game/direct"
STATUS_TOPIC = "animal_game/status"

# --- Connect to MQTT broker ---
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

def on_status(client, userdata, msg):
    print(f"[STATUS from MQTT] {msg.payload.decode()}")

client.connect(MQTT_BROKER, 1883, 60)
client.loop_start()

# Subscribe to status topic
client.subscribe(STATUS_TOPIC)
client.message_callback_add(STATUS_TOPIC, on_status)

# ... (previous code)
import time
import sys
import ast

# ... (setup code)

print("LLM Sender (terminal version)")
print("========This shows the middleground messaging can input========\n")

def process_input(text):
    text = text.strip()
    if not text:
        return True
    if text.lower() == "exit":
        return False

    # Try parsing as dict
    try:
        cmd_dict = ast.literal_eval(text)
        if isinstance(cmd_dict, dict):
            client.publish(TOPIC, str(cmd_dict))
            print(f"[Auto Rearrange] Sent: {cmd_dict}")
            # print("\n") # formatting
            return True
    except:
        pass

    # Otherwise direct command
    if text:
        client.publish(TOPIC_DIRECT, text)
        print(f"[Direct Arduino] Sent: {text}")
        # print("\n")
    
    return True

# Main loop
try:
    if sys.stdin.isatty():
        # Interactive mode
        while True:
            try:
                user_input = input("Input: ")
                if not process_input(user_input):
                    break
            except EOFError:
                break
    else:
        # Non-interactive (Piped or Detached)
        print("Non-interactive mode. Listening on stdin...")
        while True:
            line = sys.stdin.readline()
            if line == "":
                # EOF reached. 
                # If we are in detached mode (e.g. background run.py), stdin might be /dev/null (EOF).
                # If we are in piped mode (llm_sub), this means the pipe closed.
                # In both cases, we just want to stay alive for MQTT if needed, 
                # or exit if the intention was to stop. 
                # Given this script is also an MQTT listener (for status), we should stay alive.
                time.sleep(1) 
                continue
            
            if not process_input(line):
                break

except KeyboardInterrupt:
    pass

# Cleanup
client.loop_stop()
client.disconnect()
