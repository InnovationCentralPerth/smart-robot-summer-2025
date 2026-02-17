import cv2
import socket
import numpy as np
import threading

# Configuration
UDP_IP = "0.0.0.0"
RGB_PORT = 5005
DEPTH_PORT = 5006

def receive_stream(port, window_name):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, port))
    print(f"Listening for {window_name} on port {port}...")
    
    while True:
        try:
            data, addr = sock.recvfrom(65535)
            nparr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                frame = cv2.resize(frame, (600, 600))
                cv2.imshow(window_name, frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except Exception as e:
            print(f"Error in {window_name}: {e}")
            break
            
    sock.close()
    cv2.destroyWindow(window_name)

# Start threads for both streams
t1 = threading.Thread(target=receive_stream, args=(RGB_PORT, "RGB Object Detection"))
t2 = threading.Thread(target=receive_stream, args=(DEPTH_PORT, "Depth Heatmap"))

t1.start()
t2.start()

print("Press 'q' in any window to stop.")
t1.join()
t2.join()
