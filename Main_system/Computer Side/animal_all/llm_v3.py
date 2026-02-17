import tkinter as tk
from tkinter import messagebox
import serial
import time
import os
import ast  # for safely parsing dict input

# Serial setup
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)

def speak(text):
    os.system(f'espeak "{text}"')

positions = {"A": None, "B": None, "C": None, "D": None, "E": None, "F": None}
animals = ["frog", "lion", "bear", "hippo", "elephant"]

def send_command(cmd):
    cmd_str = str(cmd).strip()
    print(f"[DEBUG] Sending to Arduino: {cmd_str}")
    ser.write((cmd_str + "\n").encode())
    speak(f"Command: {cmd_str}")
    response = ser.readline().decode('utf-8').strip()
    if response:
        print(f"[Arduino reply] {response}")

def update_display():
    for pos, lbl in position_labels.items():
        lbl.config(text=positions[pos] if positions[pos] else "Empty")

# Assign animal (GUI only)
def assign_animal():
    animal = animal_var.get()
    pos = pos_var.get()
    for p, a in positions.items():
        if a == animal:
            positions[p] = None
    positions[pos] = animal
    update_display()

# Move animal (sends serial)
def move_animal():
    src = move_from_var.get()
    dst = move_to_var.get()
    if positions[src] is None:
        messagebox.showerror("Error", f"No animal at {src} to move.")
        return
    positions[dst] = positions[src]
    positions[src] = None
    update_display()
    send_command(f"{src},{dst}")
    print(f"[DEBUG] Moved animal from {src} to {dst}")

# Clickable positions
def click_position(pos):
    animal = animal_var.get()
    for p, a in positions.items():
        if a == animal:
            positions[p] = None
    positions[pos] = animal
    update_display()
    send_command(pos)
    print(f"[DEBUG] Clicked position {pos}, moved {animal} there")

# Typing serial input
def send_typed_command():
    cmd = typed_entry.get().strip()
    if cmd:
        send_command(cmd)
        typed_entry.delete(0, tk.END)

# Auto rearrange function
def auto_rearrange():
    raw_input = auto_entry.get()
    try:
        target_mapping = ast.literal_eval(raw_input)  # safely parse dict
    except Exception as e:
        messagebox.showerror("Error", f"Invalid mapping format: {e}")
        return

    animal_letter = {'F':'frog','E':'elephant','L':'lion','H':'hippo','B':'bear'}
    temp_positions = positions.copy()
    moves = []
    processed = set()

    def find_temp_free(exclude):
        for p in temp_positions:
            if temp_positions[p] is None and p not in exclude:
                return p
        return None

    while len(processed) < len(target_mapping):
        for letter, target_pos in target_mapping.items():
            animal = animal_letter[letter]
            if letter in processed:
                continue
            curr_pos = None
            for p, a in temp_positions.items():
                if a == animal:
                    curr_pos = p
                    break
            if curr_pos == target_pos:
                processed.add(letter)
                continue
            if temp_positions[target_pos] is None:
                temp_positions[target_pos] = animal
                temp_positions[curr_pos] = None
                moves.append((curr_pos, target_pos))
                processed.add(letter)
            else:
                occupying_animal = temp_positions[target_pos]
                temp_free = find_temp_free(exclude=[curr_pos, target_pos])
                if temp_free is None:
                    temp_free = curr_pos
                temp_positions[temp_free] = occupying_animal
                temp_positions[target_pos] = None
                moves.append((target_pos, temp_free))

    # Execute moves
    for src, dst in moves:
        send_command(f"{src},{dst}")
        print(f"[DEBUG] Auto-move: {src} -> {dst}")
        positions[dst] = positions[src]
        positions[src] = None
        update_display()
        time.sleep(0.5)

# GUI setup
root = tk.Tk()
root.title("Animal Position Controller")
root.geometry("950x500")

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

# Auto rearrange
auto_frame = tk.Frame(root)
auto_frame.pack(pady=10)
auto_entry = tk.Entry(auto_frame, width=50, font=("Arial", 14))
auto_entry.pack(side="left", padx=5)
tk.Button(auto_frame, text="Auto Rearrange", font=("Arial", 12), command=auto_rearrange).pack(side="left", padx=5)

# Exit handler
def on_close():
    ser.close()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
update_display()
root.mainloop()
