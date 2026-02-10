import serial
import time
import threading

# --- Adjust these according to your setup ---
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 9600
READ_TIMEOUT = 1  # seconds

# --- Serial connection (lazy init) ---
ser = None

# --- Status callback for GUI or main ---
status_callback = None  # function to call when Arduino sends Done

def init_serial():
    """Initialize serial connection safely."""
    global ser
    if ser is None:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=READ_TIMEOUT)
            print("[Serial] Connection established")
        except Exception as e:
            print(f"[Serial] Error opening serial port: {e}")

def set_status_callback(callback):
    global status_callback
    status_callback = callback

def send_command(command, wait_done=False):
    init_serial()
    if ser is None:
        print("[Serial] Not connected, cannot send command")
        return

    ser.write((command + "\n").encode())
    print(f"[Serial] Sent -> {command}")

    if wait_done:
        while True:
            line = ser.readline().decode().strip()
            if line:
                print(f"[Serial] Arduino -> {line}")
                if status_callback:
                    status_callback(line)
                if line == "Done" or line == "Invalid":
                    break
            time.sleep(0.05)

def close_serial():
    global ser
    if ser and ser.is_open:
        ser.close()
        print("[Serial] Connection closed")
