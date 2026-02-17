import subprocess
import json
import re

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
    print("Listening for detections...")

    process = subprocess.Popen(
        ["edge-impulse-linux-runner"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    last_detected = None  # store last detection to prevent repeats

    for line in process.stdout:
        line = line.strip()
        detections = parse_detection(line)
        if detections:
            # find the detection with highest confidence
            best = max(detections, key=lambda d: d.get("value", 0))
            label = best.get("label")
            score = best.get("value", 0)

            if score > 0.5 and label != last_detected:
                last_detected = label  # update last detection

                if label == "L":
                    print("🦁 Lion appears! Roar!")
                elif label == "E":
                    print("🐘 Elephant appears! Trumpet!")
                elif label == "F":
                    print("🦊 Fox appears! Sneaky!")
        else:
            last_detected = None  # reset if nothing detected

if __name__ == "__main__":
    main()
