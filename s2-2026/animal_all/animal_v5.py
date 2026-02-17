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

    # Set up main window
    root = tk.Tk()
    root.title("🦁 Animal Detection Dashboard")
    root.geometry("500x300")
    root.configure(bg="#1e1e2f")  # dark background

    # ----- Styles -----
    label_font = ("Helvetica", 18, "bold")
    button_font = ("Helvetica", 12, "bold")
    button_bg = "#4CAF50"
    button_fg = "#ffffff"

    # ----- Frame for detection -----
    frame = tk.Frame(root, bg="#2e2e3e", bd=2, relief=tk.RIDGE)
    frame.pack(fill="both", expand=True, padx=20, pady=20)

    # Label to show the latest detection
    label = tk.Label(frame, text=latest_detection, font=label_font, bg="#2e2e3e", fg="#ffffff")
    label.pack(pady=10)

    # Update label safely from another thread
    def update_label(new_text):
        nonlocal latest_detection
        latest_detection = new_text
        label.config(text=latest_detection)

    # Generic function to send any signal to serial
    def send_serial(cmd):
        ser.write((cmd + "\n").encode())
        print(f"✅ Sent to Arduino: {cmd}")
        while ser.in_waiting:
            response = ser.readline().decode('utf-8').strip()
            if response:
                print("Arduino says:", response)

    # Function to send the current animal detection
    def send_current_detection():
        if latest_detection == "🦁 Lion detected!":
            send_serial("l")
        elif latest_detection == "🐘 Elephant detected!":
            send_serial("e")
        elif latest_detection == "🦊 Frog detected!":
            send_serial("f")
        else:
            print("⚠️ No valid detection to send.")

    # ----- Buttons -----
    btn_current = tk.Button(frame, text="Pick Up Animal", font=button_font,
                            bg=button_bg, fg=button_fg, activebackground="#45a049",
                            activeforeground="#ffffff", command=send_current_detection)
    btn_current.pack(pady=5)

    # Manual signal buttons
    manual_frame = tk.Frame(frame, bg="#2e2e3e")
    manual_frame.pack(pady=10)

    signals = [("1", "1"), ("2", "2"), ("3", "3"), ("4", "4"),
               ("Top", "t"), ("Lion", "l"), ("Elephant", "e"), ("Frog", "f")]

    for text, cmd in signals:
        btn = tk.Button(manual_frame, text=text, font=("Helvetica", 10, "bold"),
                        bg="#2196F3", fg="#ffffff", activebackground="#1976D2",
                        activeforeground="#ffffff", width=8,
                        command=lambda c=cmd: send_serial(c))
        btn.pack(side="left", padx=5, pady=5)

    # ----- Run Edge Impulse runner in background -----
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
                root.after(0, lambda: update_label("🦊 Frog detected!"))

    threading.Thread(target=read_output, daemon=True).start()

    root.mainloop()
    ser.close()

if __name__ == "__main__":
    main()
