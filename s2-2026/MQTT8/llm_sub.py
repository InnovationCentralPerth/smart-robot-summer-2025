import paho.mqtt.client as mqtt
import subprocess
import shlex

MQTT_BROKER = "localhost"
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
    client.connect(MQTT_BROKER, 1883, 60)
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
