"""MQTT client abstraction for stacker communication."""
from __future__ import annotations

import json
import threading
import time
from typing import Dict, Optional

import paho.mqtt.client as mqtt

from ..config import MQTTConfig


class MQTTEventClient:
    """Wrap `paho-mqtt` with higher-level helpers for the stacker topics."""

    def __init__(self, config: Optional[MQTTConfig] = None) -> None:
        self.config = config or MQTTConfig()
        self.client = mqtt.Client()
        self.status_event = threading.Event()
        self.position_event = threading.Event()
        self.last_status: str = ""
        self.current_positions: Dict[str, str] = {"E": "L2", "L": "C", "F": "R2"}

        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def connect(self) -> None:
        """Connect to the broker and start the network loop."""
        print(f"🌐 Connecting to MQTT broker at {self.config.host}:{self.config.port}...")
        try:
            self.client.connect(self.config.host, self.config.port, 60)
            self.client.loop_start()
            time.sleep(1)
            print("✅ MQTT Connection established.")
        except Exception as e:
            print(f"❌ MQTT Connection failed: {e}")

    def disconnect(self) -> None:
        """Stop the loop and disconnect from the broker."""
        self.client.loop_stop()
        self.client.disconnect()
        print("🔌 Disconnected from MQTT.")

    def publish_command(self, command: Dict[str, str]) -> None:
        """Publish a JSON command to the stacker topic."""
        payload = json.dumps(command)
        print(f"📡 Sending command to robot...")
        self.client.publish(self.config.command_topic, payload)


    def wait_for_completion(self, desired_status: str = "DONE", timeout: float = 30) -> bool:
        """Block until the robot reports the desired status and positions are updated."""

        print(f"⌛ Waiting for robot confirmation ({timeout}s)...")
        if not self.status_event.wait(timeout=timeout):
            print("⚠️ Timeout: Robot did not send a status update.")
            return False

        while self.last_status != desired_status:
            self.status_event.clear()
            if not self.status_event.wait(timeout=timeout):
                print("⚠️ Timeout: Robot started but never finished.")
                return False

        self.status_event.clear()
        time.sleep(0.5)
        try:
            self.wait_for_positions(timeout=3.0)
            print("✅ Robot confirmed completion.")
            return True
        except TimeoutError:
            print("⚠️ Command finished, but positions weren't updated.")
            return True

    def wait_for_positions(self, timeout: float) -> None:
        if not self.position_event.wait(timeout=timeout):
            raise TimeoutError("Timed out waiting for position update")
        self.position_event.clear()


    def on_connect(self, client: mqtt.Client, userdata, flags, rc):  # type: ignore[override]
        client.subscribe(self.config.status_topic)
        client.subscribe(self.config.positions_topic)

    def on_message(self, client: mqtt.Client, userdata, msg):  # type: ignore[override]
        topic = msg.topic
        payload = msg.payload.decode()

        if topic == self.config.status_topic:
            self.last_status = payload.strip('"')
            self.status_event.set()
        elif topic == self.config.positions_topic:
            try:
                self.current_positions = json.loads(payload)
                self.position_event.set()
            except json.JSONDecodeError:
                pass


__all__ = ["MQTTEventClient"]