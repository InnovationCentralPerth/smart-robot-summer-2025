import os

def speak(text):
    os.system(f'espeak "{text}"')

speak("Moving to home position")
