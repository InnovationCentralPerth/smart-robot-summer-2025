import paho.mqtt.client as mqtt
import subprocess
import shlex
import os
from dotenv import load_dotenv
from pathlib import Path

# Load .env from parent directory
env_path = Path(__file__).resolve().parent.parent / '.env'
load_dotenv(dotenv_path=env_path)

MQTT_BROKER = os.getenv("MQTT_HOST", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
COMMAND_TOPIC = "stacker/command"

def main():
    # Launch llm_sender.py as subprocess with stdin PIPE
    proc = subprocess.Popen(
        ["python3", "-u", "llm_sender.py"],  # -u ensures unbuffered
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        stdin=subprocess.PIPE,
        text=True
    )

    def on_message(client, userdata, msg):
        payload = msg.payload.decode().strip()
        if payload:
            print(f"[LLM Sub] Sending to llm_sender: {payload}")
            proc.stdin.write(payload + "\n")
            proc.stdin.flush()  # important to ensure llm_sender receives it

    # MQTT client
    client = mqtt.Client()
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.subscribe(COMMAND_TOPIC)
    client.loop_start()


    try:
        # Optionally, print llm_sender output to this terminal
        while True:
            output_line = proc.stdout.readline()
            if output_line == "":
                break
            print(output_line, end="")  # already contains newline
    except KeyboardInterrupt:
        print("[LLM Sub] Terminating...")
    finally:
        client.loop_stop()
        proc.terminate()
        proc.wait()

if __name__ == "__main__":
    main()
