import subprocess
import webbrowser
import time

# Terminal 1: DepthAI UVC
subprocess.Popen([
    "lxterminal", "-e", "bash -c 'cd ~/depthai-python && source bin/activate && python3 examples/UVC/uvc_rgb.py; exec bash'"
])

# Small delay before starting the next one
time.sleep(3)

# Terminal 2: Animal Game
subprocess.Popen([
    "lxterminal", "-e", "bash -c 'cd ~/animal_game && python3 animal_v4.py; exec bash'"
])

# Wait 5 seconds before opening the web page
time.sleep(10)
webbrowser.open("http://10.130.57.189:4912")
