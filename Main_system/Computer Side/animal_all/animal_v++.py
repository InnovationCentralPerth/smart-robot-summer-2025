import subprocess
import tkinter as tk
import serial
import time
import threading
import sounddevice as sd
import queue
import json
from vosk import Model, KaldiRecognizer

# ---------------- SERIAL SETUP ----------------
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # Wait for Arduino reset

# ---------------- VOSK SETUP ----------------
model_path = "vosk-model-small-en-us-0.15"
model = Model(model_path)
samplerate = 16000
device = 2  # USB mic index
rec = KaldiRecognizer(model, samplerate)

q = queue.Queue()

def audio_callback(indata, frames, time_info, status):
    if status:
        print(status, flush=True)
    q.put(bytes(indata))

# ---------------- MAIN APP ----------------
def main():
    latest_detection = "No detection yet"

    root = tk.Tk()
    root.title("Animal Detection")
    root.geometry("400x200")

    # Detection label
    label = tk.Label(root, text=latest_detection, font=("Arial", 16))
    label.pack(pady=20)

    # Function to update label
    def update_label(new_text):
        nonlocal latest_detection
        latest_detection = new_text
        label.config(text=latest_detection)

    # Function to send to Arduino
    def send_to_serial():
        if latest_detection == "🦁 Lion detected!":
            cmd = "1"
        elif latest_detection == "🐘 Elephant detected!":
            cmd = "2"
        elif latest_detection == "🦊 Fox detected!":
            cmd = "3"
        else:
            print("⚠️ No valid detection to send.")
            return

        ser.write((cmd + "\n").encode())
        print(f"✅ Sent to Arduino: {cmd}")

        while ser.in_waiting:
            response = ser.readline().decode('utf-8').strip()
            if response:
                print("Arduino says:", response)

    # Button for manual send
    button = tk.Button(root, text="Send Current Detection", command=send_to_serial)
    button.pack(pady=5)

    # Button to start listening to voice
    def start_voice():
        def voice_loop():
            with sd.InputStream(samplerate=samplerate, device=device, channels=1, dtype='int16', callback=audio_callback):
                print("🎤 Voice recognition started! Say 'animal'...")
                while True:
                    data = q.get()
                    if rec.AcceptWaveform(data):
                        result = json.loads(rec.Result())
                        text = result.get("text", "").lower()
                        if text:
                            print("You said:", text)
                            if "animal" in text:
                                root.after(0, send_to_serial)

        threading.Thread(target=voice_loop, daemon=True).start()

    voice_button = tk.Button(root, text="🎤 Start Voice Control", command=start_voice)
    voice_button.pack(pady=5)

    # Edge Impulse detection in background
    process = subprocess.Popen(
        ["edge-impulse-linux-runner"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
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
                root.after(0, lambda: update_label("🦊 Fox detected!"))

    threading.Thread(target=read_output, daemon=True).start()

    root.mainloop()
    ser.close()

if __name__ == "__main__":
    main()
