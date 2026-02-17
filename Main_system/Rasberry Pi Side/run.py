#ANOTHER TERMINAL: mosquitto_pub -h localhost -p 1883 -t "stacker/command" -m '{"E": "L2", "L": "L1", "F": "C"}'
#HIDDEN LLMSENDER PUBLISH TO LLM python3 llm_pub.py
import subprocess
import time
import sys

scripts = [
    ("mqtt_gui.py", "#4 MQTT LLM INPUT AND OUTPUT"),
    ("llm_pub.py", "#4 HIDDEN LLMSENDER PUBLISH TO LLM"),
    ("llm_sub.py", "#3 HIDDEN LLM PUBLSIH TO LLMSENDER"),
    ("llm_sender.py", "#2 SENDER MIDDLEMAN LLM-RASPBERRY"),
    ("main.py", "#1 MAIN")

]

import subprocess
import time
import sys
import shutil
import os

# Start the MQTT broker first
print("Starting local MQTT Broker...")
# -u for unbuffered output
broker_process = subprocess.Popen(["python3", "-u", "mqtt_broker.py"])
time.sleep(2) # Give it time to start

scripts = [
    ("mqtt_gui.py", "#4 MQTT LLM INPUT AND OUTPUT"),
    ("llm_pub.py", "#4 HIDDEN LLMSENDER PUBLISH TO LLM"),
    ("llm_sub.py", "#3 HIDDEN LLM PUBLSIH TO LLMSENDER"),
    ("llm_sender.py", "#2 SENDER MIDDLEMAN LLM-RASPBERRY"),
    ("main.py", "#1 MAIN")
]

terminal_cmd = "xterm"
has_xterm = shutil.which(terminal_cmd)

processes = []

for script, title in scripts:
    print(f"Starting {script}...")
    if has_xterm:
        p = subprocess.Popen([
            terminal_cmd, "-T", title, "-e",
            f"bash -c 'python3 {script}; exec bash'"
        ])
    else:
        print(f"Warning: {terminal_cmd} not found. Running {script} directly in background.")
        # -u for unbuffered output so we see logs immediately
        p = subprocess.Popen(["python3", "-u", script])
    processes.append(p)
    time.sleep(0.5)  # small delay so windows don’t overlap

try:
    print("All processes started. Press Ctrl+C to stop.")
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStopping all processes...")
    for p in processes:
        p.terminate()
    broker_process.terminate()
    print("Done.")
