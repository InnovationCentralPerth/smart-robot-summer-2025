import time
from serial_comm import send_command

# Mapping letters to animals
animal_letter = {'F':'cub','E':'elephant','L':'lion','H':'hippo','B':'bear'}
name_to_pos = {'L1':'A', 'L2':'B', 'R2':'D', 'R1':'E', 'A':'A', 'B':'B', 'C':'C', 'D':'D', 'E':'E', 'F':'F'}

def normalize_mapping(target_mapping):
    normalized = {}
    for animal_letter_code, pos in target_mapping.items():
        normalized[animal_letter_code] = name_to_pos.get(pos, pos)
    return normalized

def process_command(target_mapping, positions):
    target_mapping = normalize_mapping(target_mapping)
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
    return moves

def execute_moves(moves, positions, update_display=None, delay=0.5):
    import threading
    def worker():
        for src, dst in moves:
            if hasattr(send_command, 'status_callback') and send_command.status_callback:
                send_command.status_callback("BUSY")

            send_command(f"{src},{dst}", wait_done=True)
            print(f"[DEBUG] Auto-move: {src} -> {dst}")

            positions[dst] = positions[src]
            positions[src] = None

            if update_display:
                update_display()

            time.sleep(delay)

        if hasattr(send_command, 'status_callback') and send_command.status_callback:
            send_command.status_callback(f"DONE - Final Positions: {positions}")

    threading.Thread(target=worker, daemon=True).start()
