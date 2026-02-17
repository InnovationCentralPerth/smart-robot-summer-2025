import speech_recognition as sr

# Force speech_recognition to use your FLAC exe
sr.AudioFile._flac_converter = r"C:\flac\flac.exe"

r = sr.Recognizer()
with sr.Microphone() as source:
    print("Say something:")
    r.adjust_for_ambient_noise(source)
    audio = r.record(source, duration=5)

try:
    text = (r.recognize_whisper(audio))
    print("Transcribed:", text)
except sr.UnknownValueError:
    print("Could not understand audio")
except Exception as e:
    print("Error:", e)
