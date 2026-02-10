import serial
import time
import os

def speak(text):
    os.system(f'espeak "{text}"')

speak("Starting")
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)  # Wait for Arduino reset

print("Type 1, 2, 3, or 4 and press Enter (Ctrl+C to quit).")

try:
    while True:
        cmd = input("Enter command: ")  # Get from terminal
        if cmd.strip() in ["1", "2", "3", "4"]:
            ser.write((cmd + "\n").encode())  # Send to Arduino
            speak("Moving to" + str(cmd))

            response = ser.readline().decode('utf-8').strip()
            if response:
                print("Arduino says:", response)
        else:
            print("Invalid input. Use 1, 2, 3, or 4.")

except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()
