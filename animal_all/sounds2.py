import subprocess
import json
import re
import random
import time
import select

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
buffer_clear_time = 1.5  # Increased time to allow camera/scene to stabilize
question_timeout = 10.0  # Timeout for each question in seconds

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

def clear_buffer(process, timeout=0.5):
    """Clear the stdout buffer by reading all available lines with a timeout."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        readable, _, _ = select.select([process.stdout], [], [], 0.1)
        if readable:
            line = process.stdout.readline().strip()
            if line:
                print(f"[DEBUG] Cleared line: {line}")
        else:
            break

def check_process_running(process):
    """Check if the subprocess is still running."""
    return process.poll() is None

# Start Edge Impulse runner
try:
    process = subprocess.Popen(
        ["edge-impulse-linux-runner"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )
except Exception as e:
    print(f"Error starting edge-impulse-linux-runner: {e}")
    exit(1)

print("Welcome to the Animal Sound Game!")
time.sleep(1)

for i in range(num_questions):
    if not check_process_running(process):
        print("Error: edge-impulse-linux-runner stopped running. Exiting.")
        break

    target = random.choice(animals)
    print(f"\nQuestion {i+1}: {animal_questions[target]}")
    print("Please show the correct animal...")

    # Clear the buffer and wait for the scene to stabilize
    clear_buffer(process, timeout=0.5)
    time.sleep(buffer_clear_time)

    detected_correct = False
    attempts = 0
    question_start_time = time.time()

    while not detected_correct and attempts < max_attempts:
        # Check for question timeout
        if time.time() - question_start_time > question_timeout:
            print(f"⏰ Timeout! Moving on. The correct answer was a {animal_names[target]}.")
            break

        # Non-blocking read with timeout
        readable, _, _ = select.select([process.stdout], [], [], 0.5)
        if readable:
            line = process.stdout.readline().strip()
            if line:
                print(f"[DEBUG] Read line: {line}")
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
                            print(f"❌ Not quite, that’s a {animal_names.get(label, 'unknown')}.")
                            time.sleep(0.3)
        else:
            print("[DEBUG] No new data from runner. Waiting...")
            time.sleep(0.1)

    if not detected_correct and attempts >= max_attempts:
        print(f"⏩ Moving on! The correct answer was a {animal_names[target]}.")
        time.sleep(2)

print("\n🎉 Game Over! Great job!")
try:
    process.terminate()
    process.wait(timeout=2.0)  # Wait for clean termination
except:
    process.kill()  # Force kill if termination fails