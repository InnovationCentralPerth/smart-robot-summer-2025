import subprocess
import tkinter as tk
import serial
import time
import threading

# Initialize serial connection
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # Wait for Arduino reset

def main():
    latest_detection = "No detection yet"

    # Set up the main window
    root = tk.Tk()
    root.title("Animal Detection")
    root.geometry("350x150")

    # Label to show the latest detection
    label = tk.Label(root, text=latest_detection, font=("Arial", 16))
    label.pack(pady=20)

    # Function to update the label text safely from another thread
    def update_label(new_text):
        nonlocal latest_detection
        latest_detection = new_text
        label.config(text=latest_detection)

    # Function called when button is pressed
    def send_to_serial():
        if latest_detection == "🦁 Lion detected!":
            cmd = "l"
        elif latest_detection == "🐘 Elephant detected!":
            cmd = "e"
        elif latest_detection == "🦊 Frog detected!":
            cmd = "f"
        else:
            print("No valid detection to send.")
            return

        ser.write((cmd + "\n").encode())  # Send to Arduino
        print(f"Sent to Arduino: {cmd}")   # Optional: also print to terminal

        # Read Arduino response (non-blocking)
        while ser.in_waiting:
            response = ser.readline().decode('utf-8').strip()
            if response:
                print("Arduino says:", response)

    # Button to send the current detection to Arduino
    button = tk.Button(root, text="Send Current Detection", command=send_to_serial)
    button.pack(pady=10)

    # Run Edge Impulse runner in a background thread
    process = subprocess.Popen(
        ["edge-impulse-linux-runner"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1  # Line-buffered
    )

    def read_output():
        for line in process.stdout:
            line = line.strip()
            print(line)
            if "L" in line:
                root.after(0, lambda: update_label("🦁 Lion detected!"))
            elif "E" in line:
                root.after(0, lambda: update_label("🐘 Elephant detected!"))
            elif "F" in line:
                root.after(0, lambda: update_label("🦊 Frog detected!"))

    threading.Thread(target=read_output, daemon=True).start()

    root.mainloop()
    ser.close()

if __name__ == "__main__":
    main()
