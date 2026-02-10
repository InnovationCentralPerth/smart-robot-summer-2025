import tkinter as tk
from tkinter import messagebox
from serial_comm import send_command
from command_processor import process_command, execute_moves
import ast
import threading

positions = {"A": None, "B": "elephant", "C": "lion", "D": "cub", "E": None, "F": None}
animals = ["cub", "lion", "elephant"]

def create_gui(status_callback=None):
    root = tk.Tk()
    root.title("Animal Position Controller")
    root.geometry("950x500")

    # --- Position display ---
    position_frame = tk.Frame(root)
    position_frame.pack(pady=20)
    position_labels = {}
    for pos in positions:
        frame = tk.Frame(position_frame)
        frame.pack(side="left", padx=5)
        lbl_title = tk.Label(frame, text=pos, width=12, font=("Arial", 14, "bold"))
        lbl_title.pack()
        lbl_animal = tk.Label(frame, text="Empty", width=12, height=3,
                              relief="ridge", bd=2, padx=5, pady=5, font=("Arial", 14))
        lbl_animal.pack()
        position_labels[pos] = lbl_animal

    # --- Central update function ---
    def update_positions_and_display(src=None, dst=None, moves=None, assign=None):
        if assign:
            animal, pos = assign
            for p, a in positions.items():
                if a == animal:
                    positions[p] = None
            positions[pos] = animal

        if src and dst:
            positions[dst] = positions[src]
            positions[src] = None

        if moves:
            for s, d in moves:
                positions[d] = positions[s]
                positions[s] = None

        # Update labels
        for pos, lbl in position_labels.items():
            lbl.config(text=positions[pos] if positions[pos] else "Empty")

        # --- Status callback ---
        if status_callback:
            status_callback(f"CHANGED - Final Positions: {positions}")

    # --- Assign animal ---
    animal_var = tk.StringVar(value=animals[0])
    pos_var = tk.StringVar(value="A")
    assign_frame = tk.Frame(root)
    assign_frame.pack(pady=10)
    tk.OptionMenu(assign_frame, animal_var, *animals).pack(side="left", padx=5)
    tk.OptionMenu(assign_frame, pos_var, *positions.keys()).pack(side="left", padx=5)

    def assign_animal():
        animal = animal_var.get()
        pos = pos_var.get()
        update_positions_and_display(assign=(animal, pos))

    tk.Button(assign_frame, text="Assign Animal", command=assign_animal).pack(side="left", padx=10)

    # --- Move animal ---
    move_from_var = tk.StringVar(value="A")
    move_to_var = tk.StringVar(value="B")
    move_frame = tk.Frame(root)
    move_frame.pack(pady=10)
    tk.Label(move_frame, text="From:").pack(side="left")
    tk.OptionMenu(move_frame, move_from_var, *positions.keys()).pack(side="left", padx=5)
    tk.Label(move_frame, text="To:").pack(side="left")
    tk.OptionMenu(move_frame, move_to_var, *positions.keys()).pack(side="left", padx=5)

    def move_animal():
        src = move_from_var.get()
        dst = move_to_var.get()
        if positions[src] is None:
            messagebox.showerror("Error", f"No animal at {src} to move.")
            return
        threading.Thread(target=lambda: send_command(f"{src},{dst}", wait_done=True), daemon=True).start()
        update_positions_and_display(src=src, dst=dst)

    tk.Button(move_frame, text="Move Animal", command=move_animal).pack(side="left", padx=10)

    # --- Auto rearrange ---
    auto_frame = tk.Frame(root)
    auto_frame.pack(pady=10)
    auto_entry = tk.Entry(auto_frame, width=50, font=("Arial", 14))
    auto_entry.pack(side="left", padx=5)

    def auto_rearrange():
        raw_input = auto_entry.get()
        try:
            target_mapping = ast.literal_eval(raw_input)
            if isinstance(target_mapping, dict):
                moves = process_command(target_mapping, positions)
                execute_moves(moves, positions, update_positions_and_display)
        except Exception as e:
            messagebox.showerror("Error", f"Invalid mapping: {e}")

    tk.Button(auto_frame, text="Auto Rearrange", font=("Arial", 12), command=auto_rearrange).pack(side="left", padx=5)

    # --- Direct command ---
    direct_frame = tk.Frame(root)
    direct_frame.pack(pady=10)
    direct_entry = tk.Entry(direct_frame, width=20, font=("Arial", 14))
    direct_entry.pack(side="left", padx=5)

    def send_direct_command():
        cmd = direct_entry.get().strip()
        if cmd:
            threading.Thread(target=lambda: send_command(cmd, wait_done=True), daemon=True).start()
            if "," in cmd and len(cmd.split(",")) == 2:
                src, dst = cmd.split(",")
                src, dst = src.strip(), dst.strip()
                if src in positions and dst in positions and positions[src]:
                    update_positions_and_display(src=src, dst=dst)
            direct_entry.delete(0, tk.END)
            print(f"[DEBUG] Direct command sent: {cmd}")

    tk.Button(direct_frame, text="Send Direct", font=("Arial", 12),
              command=send_direct_command).pack(side="left", padx=5)

    # --- Exit handler ---
    def on_close():
        close_serial()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    update_positions_and_display()  # Initial display
    return root, update_positions_and_display, positions
