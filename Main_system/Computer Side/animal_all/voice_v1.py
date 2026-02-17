import sounddevice as sd
import queue
import json
from vosk import Model, KaldiRecognizer

# Path to your Vosk model
model_path = "vosk-model-small-en-us-0.15"

# Create Vosk model
model = Model(model_path)
samplerate = 16000  # Typical for small English model

# Use USB mic: card 2, device 0
device = 2

# Queue for audio data
q = queue.Queue()

def callback(indata, frames, time, status):
    """This will be called (from a separate thread) for each audio block."""
    if status:
        print(status, flush=True)
    q.put(bytes(indata))

# Create recognizer
rec = KaldiRecognizer(model, samplerate)

print("✅ Vosk ready! Speak into your microphone...")
print("Press Ctrl+C to stop.")

with sd.InputStream(samplerate=samplerate, device=device, channels=1, dtype='int16', callback=callback):
    while True:
        data = q.get()
        if rec.AcceptWaveform(data):
            result = json.loads(rec.Result())
            text = result.get("text", "")
            if text:
                print("You said:", text)
