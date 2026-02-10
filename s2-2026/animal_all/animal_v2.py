import subprocess
import tkinter as tk

def main():
    latest_detection = "No detection yet"

    # Set up the main window
    root = tk.Tk()
    root.title("Animal Detection")
    root.geometry("300x100")

    label = tk.Label(root, text=latest_detection, font=("Arial", 16))
    label.pack(expand=True)

    # Function to update the label text
    def update_label(new_text):
        nonlocal latest_detection
        latest_detection = new_text
        label.config(text=latest_detection)

    # Run Edge Impulse runner in the background
    process = subprocess.Popen(
        ["edge-impulse-linux-runner"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    def read_output():
        line = process.stdout.readline()
        if line:
            line = line.strip()
            print(line)
            if "L" in line:
                update_label("🦁 Lion detected!")
            elif "E" in line:
                update_label("🐘 Elephant detected!")
            elif "F" in line:
                update_label("🦊 Fox detected!")
        # Schedule next check
        root.after(100, read_output)

    # Start reading Edge Impulse output
    root.after(100, read_output)

    root.mainloop()

if __name__ == "__main__":
    main()
