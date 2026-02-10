import tkinter as tk
from tkinter import scrolledtext
import paho.mqtt.client as mqtt
import threading
from collections import deque

BROKER = "localhost"   # change if using remote broker
PORT = 1883
TOPICS = ["stacker/command", "stacker/positions", "stacker/status"]

class MQTTGui:
    def __init__(self, root):
        self.root = root
        self.root.title("Stacker MQTT Dashboard")

        self.text_boxes = {}
        self.entry_boxes = {}
        self.message_queues = {topic: deque(maxlen=10) for topic in TOPICS}

        # Create UI for each topic
        for i, topic in enumerate(TOPICS):
            frame = tk.LabelFrame(root, text=topic, padx=5, pady=5)
            frame.grid(row=0, column=i, padx=10, pady=10)

            # Message display
            text_area = scrolledtext.ScrolledText(frame, width=40, height=15, state="disabled")
            text_area.grid(row=0, column=0, columnspan=2, pady=5)
            self.text_boxes[topic] = text_area

            # Entry for publishing
            entry = tk.Entry(frame, width=30)
            entry.grid(row=1, column=0, pady=5)
            self.entry_boxes[topic] = entry

            # Button for publishing
            btn = tk.Button(frame, text="Send", command=lambda t=topic: self.publish(t))
            btn.grid(row=1, column=1, padx=5)

        # Setup MQTT client
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(BROKER, PORT, 60)

        # Run MQTT in background thread
        threading.Thread(target=self.client.loop_forever, daemon=True).start()

    def on_connect(self, client, userdata, flags, rc):
        for topic in TOPICS:
            client.subscribe(topic)

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()

        # Keep only the last 10 messages
        self.message_queues[topic].append(payload)

        # Refresh text area
        text_area = self.text_boxes[topic]
        text_area.config(state="normal")
        text_area.delete(1.0, tk.END)  # clear old
        for m in self.message_queues[topic]:
            text_area.insert(tk.END, f"{m}\n")
        text_area.see(tk.END)
        text_area.config(state="disabled")

    def publish(self, topic):
        message = self.entry_boxes[topic].get()
        if message:
            self.client.publish(topic, message)
            self.entry_boxes[topic].delete(0, tk.END)

if __name__ == "__main__":
    root = tk.Tk()
    gui = MQTTGui(root)
    root.mainloop()
