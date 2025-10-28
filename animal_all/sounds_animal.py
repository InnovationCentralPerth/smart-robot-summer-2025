import subprocess
import json
import re
import random
import time

# Animals and questions
animals = ["L", "E", "F"]  # L = Lion, E = Elephant, F = Frog
animal_names = {"L": "lion", "E": "elephant", "F": "frog"}
animal_questions = {
    "L": "What animal makes the sound Roar?",
    "E": "What animal makes the sound Trumpet?",
    "F": "What is green?"
}

num_questions = 5
max_attempts = 30  # max wrong tries per question

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

# Start Edge Impulse runner
process = subprocess.Popen(
    ["edge-impulse-linux-runner"],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True
)

print("Welcome to the Animal Sound Game!")
time.sleep(1)

for i in range(num_questions):
    target = random.choice(animals)
    print(f"\nQuestion {i+1}: {animal_questions[target]}")

    detected_correct = False
    attempts = 0

    while not detected_correct and attempts < max_attempts:
        line = process.stdout.readline().strip()
        detections = parse_detection(line)
        if detections:
            best = max(detections, key=lambda d: d.get("value", 0))
            label = best.get("label")
            score = best.get("value", 0)

            if score > 0.5:
                attempts += 1
                if label == target:
                    print(f"✅ Correct! I see a {animal_names[label]}!")
                    detected_correct = True
                    time.sleep(0.5)
                else:
                    print(f"❌ Not quite, that’s a {animal_names.get(label,'unknown')}.")
                    time.sleep(0.3)

    if not detected_correct:
        print(f"⏩ Moving on! The correct answer was a {animal_names[target]}.\n")
        time.sleep(2)

print("\n🎉 Game Over! Great job!")
process.terminate()
