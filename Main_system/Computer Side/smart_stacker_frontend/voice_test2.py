import sounddevice as sd
from scipy.io.wavfile import write
import whisper

fs = 44100
seconds = 5

print("🎙️ Recording...")
recording = sd.rec(int(seconds * fs), samplerate=fs, channels=1)
sd.wait()
write("mic_input.wav", fs, recording)

print("⏳ Transcribing...")
model = whisper.load_model("base")
result = model.transcribe("mic_input.wav")

print("✨ You said:", result["text"])
