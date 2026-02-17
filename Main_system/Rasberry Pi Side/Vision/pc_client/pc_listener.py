import socket
import json

UDP_IP = "0.0.0.0" # Listen on all interfaces
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for OAK-D Lite data on port {UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024)
    try:
        info = json.loads(data.decode('utf-8'))
        print(f"OBJECT: {info['object'].ljust(15)} | DIST: {info['distance_mm']} mm | CONF: {info['confidence']}%")
    except Exception as e:
        print(f"Raw data: {data}")
