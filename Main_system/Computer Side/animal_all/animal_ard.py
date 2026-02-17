import subprocess
import json
import re
import serial
import time
import os

# --- Speak function (espeak) ---
def speak(text):
    os.system(f'espeak "{text}"')

# --- Parse detection from Edge Impulse runner ---
def parse_detection(line):
    match = re.search(r'\[.*\]', line)
    if match:
        try:
            data = json.loads(match.group(0))
            if data and isinstance(data, list):
                return data
        except json.JSONDecodeError:
            return None
    return None

def main():
    # Setup serial connection to Arduino
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    time.sleep(2)  # Wait for Arduino reset
    speak("Starting detection system")

    print("Listening for detections...")

    process = subprocess.Popen(
        ["edge-impulse-linux-runner"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    last_detected = None      # Avoid repeated same detections
    detection_counter = 0     # Count detections until we hit 5

    for line in process.stdout:
        line = line.strip()
        detections = parse_detection(line)

        if detections:
            # Pick highest confidence detection
            best = max(detections, key=lambda d: d.get("value", 0))
            label = best.get("label")
            score = best.get("value", 0)

            # Always print detection results
            print(f"Detected: {label} ({score:.2f})")

            if score > 0.5:  # valid detection
                detection_counter += 1
                print(f"Valid detection count: {detection_counter}")

                # Only act every 5 detections
                if detection_counter >= 5 and label != last_detected:
                    last_detected = label
                    detection_counter = 0  # reset counter

                    # Decide command based on label
                    if label == "L":
                        cmd = "1"
                        speak("Lion detected, moving to position one")
                        print("🦁 Lion → sending 1")
                    elif label == "E":
                        cmd = "2"
                        speak("Elephant detected, moving to position two")
                        print("🐘 Elephant → sending 2")
                    elif label == "F":
                        cmd = "3"
                        speak("Frog detected, moving to position three")
                        print("🐸 Frog → sending 3")
                    else:
                        cmd = "4"
                        speak("Unknown detected, moving to home position")
                        print("⬅️ Unknown → sending 4")

                    # Send to Arduino
                    ser.write((cmd + "\n").encode())

                    # Read Arduino response
                    response = ser.readline().decode('utf-8').strip()
                    if response:
                        print("Arduino says:", response)

        else:
            last_detected = None  # reset if nothing detected

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting...")
