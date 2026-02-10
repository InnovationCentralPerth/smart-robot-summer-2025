import subprocess
import json
import paho.mqtt.client as mqtt

STACKER_POS_TOPIC = "stacker/positions"
STACKER_STATUS_TOPIC = "stacker/status"
MQTT_BROKER = "localhost"
MQTT_PORT = 1883

# Initialize MQTT client
client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT, 60)

def publish(topic, message_obj):
    """
    Publishes a JSON message to the topic using MQTT.
    message_obj can be a dict (positions) or a string (status)
    """
    message = json.dumps(message_obj)
    client.publish(topic, message)
    return message

def process_line(line):
    line = line.strip()
    if not line:
        return

    animal_letter = {'F':'cub','E':'elephant','L':'lion'}
    name_to_pos = {'L1':'A', 'L2':'B', 'R2':'D', 'R1':'E', 'C':'C', 'F':'F'}

    try:
        # Handle status messages
        if "DONE" in line:
            published = publish(STACKER_STATUS_TOPIC, "DONE")
            print(f"[LLM Bridge] Published status: {published}")
        elif "BUSY" in line:
            published = publish(STACKER_STATUS_TOPIC, "BUSY")
            print(f"[LLM Bridge] Published status: {published}")

        # Handle position updates
        if "Final Positions:" in line:
            pos_part = line.split("Final Positions:", 1)[1].strip()
            positions_actual = eval(pos_part)  # {'A':'frog','B':None,...}

            positions_dict = {}
            for letter, animal in animal_letter.items():
                positions_dict[letter] = "N"  # default if not on board
                for pos_key, pos_animal in positions_actual.items():
                    if pos_animal == animal:
                        for short, full in name_to_pos.items():
                            if full == pos_key:
                                positions_dict[letter] = short
                                break

            published = publish(STACKER_POS_TOPIC, positions_dict)
            print(f"[LLM Bridge] Published positions: {published}")

    except Exception as e:
        print(f"[LLM Bridge] Failed to process line: {line}, error: {e}")


def main():
    try:
        proc = subprocess.Popen(
            ["python3", "-u", "llm_sender.py"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )

        buffer = ""
        while True:
            char = proc.stdout.read(1)
            if not char:
                break
            buffer += char
            if '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                process_line(line)

    except FileNotFoundError:
        print("[LLM Bridge] llm_sender.py not found. Nothing running.")
    except KeyboardInterrupt:
        print("[LLM Bridge] Terminated by user.")
        if proc:
            proc.terminate()
            proc.wait()

if __name__ == "__main__":
    main()
