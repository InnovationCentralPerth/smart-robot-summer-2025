import sys
import os
from unittest.mock import MagicMock, patch

# Mock all hardware/network dependencies
sys.modules["paho"] = MagicMock()
sys.modules["paho.mqtt"] = MagicMock()
sys.modules["paho.mqtt.client"] = MagicMock()
sys.modules["dotenv"] = MagicMock()
sys.modules["speech_recognition"] = MagicMock()
sys.modules["pyaudio"] = MagicMock()
sys.modules["tkinter"] = MagicMock()
sys.modules["tkinter.scrolledtext"] = MagicMock()
sys.modules["openai"] = MagicMock()
sys.modules["requests"] = MagicMock()

# Mock the LLM client to return a valid command
mock_llm_response = {"E": "L1", "L": "C", "F": "R1"}

# Add current dir to path
sys.path.append(os.getcwd())

from smart_stacker_frontend.smart_stacker.controllers.voice import HybridVoiceController

def simulate_app():
    print("--- Starting Application Simulation ---")
    
    # 1. Initialize controller
    try:
        controller = HybridVoiceController(safe_mode=True)
        print("✅ Controller initialized.")
    except Exception as e:
        print(f"❌ Failed to initialize controller: {e}")
        return

    # 2. Mock the internal components to avoid real network/io
    controller.mqtt_client.client = MagicMock()
    controller.mqtt_client.status_event = MagicMock()
    controller.mqtt_client.status_event.wait.return_value = True
    controller.mqtt_client.position_event = MagicMock()
    controller.mqtt_client.position_event.wait.return_value = True
    controller.mqtt_client.last_status = "DONE"
    
    # Mock LLM generation
    controller.llm.generate_robot_command = MagicMock(return_value=mock_llm_response)
    
    # 3. Simulate processing a text command
    print("Simulating command: 'put the elephant in the back left'")
    try:
        result = controller.process_text("put the elephant in the back left")
        print(f"✅ Simulation Result: {result.message}")
        print(f"✅ Generated Command: {result.command}")
    except Exception as e:
        print(f"❌ Simulation failed during processing: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    simulate_app()
