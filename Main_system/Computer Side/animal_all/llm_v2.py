import tkinter as tk
from tkinter import messagebox
import serial
import time
import os

# Serial setup
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)

# Helper to speak text
def speak(text):
    os.system(f'espeak "{text}"')

# Positions and their animals
positions = {"A": None, "B": None, "C": None, "D": None, "E": None, "F": None}

# List of possible animals
animals = ["frog", "lion", "bear", "hippo", "elephant"]

# Send serial command
def send_command(cmd):
    cmd_str = str(cmd).strip()
    print(f"[DEBUG] Sending to Arduino: {cmd_str}")
    ser.write((cmd_str + "\n").encode())
    speak(f"Command: {cmd_str}")
    response = ser.readline().decode('utf-8').strip()
    if response:
        print(f"[Arduino reply] {response}")

# Update position display
def update_display():
    for pos, lbl in position_labels.items():
        lbl.config(text=positions[pos] if positions[pos] else "Empty")

# Assign an animal to a position (GUI only, no serial)
def assign_animal():
    animal = animal_var.get()
    pos = pos_var.get()
    # Remove animal from any other position
    for p, a in positions.items():
        if a == animal:
            positions[p] = None
    positions[pos] = animal  # Override any previous animal
    update_display()
    # NO serial command sent, no debug print

# Move animal from one position to another (sends serial)
def move_animal():
    src = move_from_var.get()
    dst = move_to_var.get()
    if positions[src] is None:
        messagebox.showerror("Error", f"No animal at {src} to move.")
        return
    positions[dst] = positions[src]  # Override if needed
    positions[src] = None
    update_display()
    send_command(f"{src},{dst}")  # Send both positions
    print(f"[DEBUG] Moved animal from {src} to {dst}")

# Clickable positions: assign the currently selected animal to that position and send serial
def click_position(pos):
    animal = animal_var.get()
    for p, a in positions.items():
        if a == animal:
            positions[p] = None
    positions[pos] = animal  # Override any existing animal
    update_display()
    send_command(pos)
    print(f"[DEBUG] Clicked position {pos}, moved {animal} there")

# Send typed serial input
def send_typed_command():
    cmd = typed_entry.get().strip()
    if cmd:
        send_command(cmd)
        typed_entry.delete(0, tk.END)

# GUI setup
root = tk.Tk()
root.title("Animal Position Controller")
root.geometry("900x450")

# Position display
position_frame = tk.Frame(root)
position_frame.pack(pady=20)

position_labels = {}
for pos in positions:
    lbl = tk.Label(position_frame, text="Empty", width=12, height=3,
                   relief="ridge", bd=2, padx=5, pady=5, font=("Arial", 14))
    lbl.pack(side="left", padx=5)
    lbl.bind("<Button-1>", lambda e, p=pos: click_position(p))
    position_labels[pos] = lbl

# Assign animal
assign_frame = tk.Frame(root)
assign_frame.pack(pady=10)

animal_var = tk.StringVar(value=animals[0])
pos_var = tk.StringVar(value="A")

tk.OptionMenu(assign_frame, animal_var, *animals).pack(side="left", padx=5)
tk.OptionMenu(assign_frame, pos_var, *positions.keys()).pack(side="left", padx=5)
tk.Button(assign_frame, text="Assign Animal", command=assign_animal).pack(side="left", padx=10)

# Move animal
move_frame = tk.Frame(root)
move_frame.pack(pady=10)

move_from_var = tk.StringVar(value="A")
move_to_var = tk.StringVar(value="B")

tk.Label(move_frame, text="From:").pack(side="left")
tk.OptionMenu(move_frame, move_from_var, *positions.keys()).pack(side="left", padx=5)
tk.Label(move_frame, text="To:").pack(side="left")
tk.OptionMenu(move_frame, move_to_var, *positions.keys()).pack(side="left", padx=5)
tk.Button(move_frame, text="Move Animal", command=move_animal).pack(side="left", padx=10)

# Direct command buttons
command_frame = tk.Frame(root)
command_frame.pack(pady=10)

for cmd in ["X","Y","Z","A","B","C","D","E","F"]:
    tk.Button(command_frame, text=cmd, width=5,
              command=lambda c=cmd: send_command(c)).pack(side="left", padx=2)

# Typing serial input
typed_frame = tk.Frame(root)
typed_frame.pack(pady=10)
typed_entry = tk.Entry(typed_frame, width=30, font=("Arial", 14))
typed_entry.pack(side="left", padx=5)
tk.Button(typed_frame, text="Send", font=("Arial", 12), command=send_typed_command).pack(side="left", padx=5)

# Exit handler
def on_close():
    ser.close()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
update_display()
root.mainloop()
