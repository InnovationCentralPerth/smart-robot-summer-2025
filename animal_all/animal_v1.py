import subprocess

def main():
    print("Listening for detections...")

    # Run the Edge Impulse Linux runner
    process = subprocess.Popen(
        ["edge-impulse-linux-runner"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    for line in process.stdout:
        line = line.strip()
        # Only check if a known animal label appears in the line
        if "L" in line:
            print("🦁 Lion detected!")
        elif "E" in line:
            print("🐘 Elephant detected!")
        elif "F" in line:
            print("🦊 Fox detected!")

if __name__ == "__main__":
    main()

