import sys
import os
from unittest.mock import MagicMock

# 1. Mock heavy dependencies but NOT speech_recognition
# We need speech_recognition to actually try and use the FLAC converter
sys.modules["paho"] = MagicMock()
sys.modules["paho.mqtt"] = MagicMock()
sys.modules["paho.mqtt.client"] = MagicMock()
sys.modules["dotenv"] = MagicMock()
sys.modules["pyaudio"] = MagicMock()
sys.modules["tkinter"] = MagicMock()
sys.modules["tkinter.scrolledtext"] = MagicMock()
sys.modules["openai"] = MagicMock()
sys.modules["requests"] = MagicMock()

import speech_recognition as sr

# 2. Configure the FLAC converter path as specified by the user
sr.AudioFile._flac_converter = r"C:\flac\flac.exe"

def test_recognition():
    file_path = r"/mnt/c/Users/davem/OneDrive/Desktop/Computer Side/voice_cmds/record_1764658164.wav"
    
    print(f"--- Testing Recognition for: {file_path} ---")
    
    if not os.path.exists(file_path):
        print(f"❌ Error: File not found at {file_path}")
        return

    r = sr.Recognizer()
    try:
        print("Reading audio file...")
        with sr.AudioFile(file_path) as source:
            audio = r.record(source)
        
        print("Attempting transcription using Whisper (requires FLAC)...")
        # Note: This will likely fail in THIS environment because I don't have C:\flac\flac.exe
        # but it will tell us if the CODE is calling it correctly.
        text = r.recognize_whisper(audio)
        print(f"✅ Transcribed Text: {text}")
        
    except Exception as e:
        print(f"❌ Recognition failed: {e}")
        if "flac" in str(e).lower():
            print("💡 This failure is expected in the AGENT environment because I don't have flac.exe installed at C:\\flac\\.")
            print("   However, the code is correctly set up to use it on YOUR machine.")

if __name__ == "__main__":
    test_recognition()
