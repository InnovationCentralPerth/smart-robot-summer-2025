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
    ser.write((cmd + "\n").encode())
    speak(f"Command: {cmd}")
    response = ser.readline().decode('utf-8').strip()
    if response:
        print("Arduino says:", response)

# Update position display
def update_display():
    for pos, lbl in position_labels.items():
        lbl.config(text=positions[pos] if positions[pos] else "Empty")

# Assign an animal to a position
def assign_animal():
    animal = animal_var.get()
    pos = pos_var.get()
    if animal not in animals:
        messagebox.showerror("Error", "Select a valid animal.")
        return
    if positions[pos]:
        messagebox.showerror("Error", f"Position {pos} already has {positions[pos]}.")
        return
    # Ensure animal is not in any other position
    for p, a in positions.items():
        if a == animal:
            positions[p] = None
    positions[pos] = animal
    update_display()

# Move animal from one position to another
def move_animal():
    src = move_from_var.get()
    dst = move_to_var.get()
    if positions[src] is None:
        messagebox.showerror("Error", f"No animal at {src} to move.")
        return
    if positions[dst] is not None:
        messagebox.showerror("Error", f"Destination {dst} already has {positions[dst]}.")
        return
    positions[dst] = positions[src]
    positions[src] = None
    update_display()
    send_command(dst)  # Send the destination position to Arduino

# GUI setup
root = tk.Tk()
root.title("Animal Position Controller")

# Position display
position_frame = tk.Frame(root)
position_frame.pack(pady=10)

position_labels = {}
for pos in positions:
    lbl = tk.Label(position_frame, text="Empty", width=12, relief="ridge", padx=5, pady=5)
    lbl.pack(side="left", padx=5)
    position_labels[pos] = lbl

# Assign animal
assign_frame = tk.Frame(root)
assign_frame.pack(pady=10)

animal_var = tk.StringVar(value=animals[0])
pos_var = tk.StringVar(value="A")

tk.OptionMenu(assign_frame, animal_var, *animals).pack(side="left")
tk.OptionMenu(assign_frame, pos_var, *positions.keys()).pack(side="left")
tk.Button(assign_frame, text="Assign Animal", command=assign_animal).pack(side="left")

# Move animal
move_frame = tk.Frame(root)
move_frame.pack(pady=10)

move_from_var = tk.StringVar(value="A")
move_to_var = tk.StringVar(value="B")

tk.Label(move_frame, text="From:").pack(side="left")
tk.OptionMenu(move_frame, move_from_var, *positions.keys()).pack(side="left")
tk.Label(move_frame, text="To:").pack(side="left")
tk.OptionMenu(move_frame, move_to_var, *positions.keys()).pack(side="left")
tk.Button(move_frame, text="Move Animal", command=move_animal).pack(side="left")

# Direct command buttons
command_frame = tk.Frame(root)
command_frame.pack(pady=10)

for cmd in ["X","Y","Z","A","B","C","D","E","F"]:
    tk.Button(command_frame, text=cmd, width=4, command=lambda c=cmd: send_command(c)).pack(side="left", padx=2)

# Exit handler
def on_close():
    ser.close()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
update_display()
root.mainloop()
