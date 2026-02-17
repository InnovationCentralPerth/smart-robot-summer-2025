#ANOTHER TERMINAL: mosquitto_pub -h localhost -p 1883 -t "stacker/command" -m '{"E": "L2", "L": "L1", "F": "C"}'
#HIDDEN LLMSENDER PUBLISH TO LLM python3 llm_pub.py
import subprocess
import time

scripts = [
    ("mqtt_gui.py", "#4 MQTT LLM INPUT AND OUTPUT"),
    ("llm_pub.py", "#4 HIDDEN LLMSENDER PUBLISH TO LLM"),
    ("llm_sub.py", "#3 HIDDEN LLM PUBLSIH TO LLMSENDER"),
    ("llm_sender.py", "#2 SENDER MIDDLEMAN LLM-RASPBERRY"),
    ("main.py", "#1 MAIN")

]

for script, title in scripts:
    subprocess.Popen([
        "x-terminal-emulator", "-T", title, "-e",
        f"bash -c 'python3 {script}; exec bash'"
    ])
    time.sleep(0.5)  # small delay so windows don’t overlap
