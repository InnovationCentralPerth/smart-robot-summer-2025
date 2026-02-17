import cv2
import socket
import numpy as np

# Configuration
UDP_IP = "0.0.0.0"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for video stream on port {UDP_PORT}...")
print("Press 'q' to exit.")

while True:
    try:
        # Receive packet (Max UDP size)
        data, addr = sock.recvfrom(65535)
        
        # Convert bytes to numpy array
        nparr = np.frombuffer(data, np.uint8)
        
        # Decode JPEG
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if frame is not None:
            # Upscale for better viewing on PC monitor
            frame = cv2.resize(frame, (800, 800))
            cv2.imshow("Remote OAK-D Lite View", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    except Exception as e:
        print(f"Error: {e}")

cv2.destroyAllWindows()
