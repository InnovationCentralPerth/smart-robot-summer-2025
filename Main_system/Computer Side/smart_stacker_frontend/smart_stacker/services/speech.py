"""Speech recognition helpers for voice driven commands."""
from __future__ import annotations

from typing import Callable, List, Optional

import speech_recognition as sr
import time
import threading
import os
from io import BytesIO
from openai import OpenAI
from typing import Callable, List, Optional

from ..config import SpeechConfig
from ..utils.audio import suppress_audio_errors

# Force speech_recognition to use the local FLAC executable
sr.AudioFile._flac_converter = r"C:\flac\flac.exe"


class SpeechInterface:
    """High level interface for transcription and wake-word listening."""

    def __init__(self, config: Optional[SpeechConfig] = None) -> None:
        self.config = config or SpeechConfig()
        self.recognizer = sr.Recognizer()
        self.voice_directory = self.config.voice_directory
        self.voice_directory.mkdir(parents=True, exist_ok=True)
        self.microphone_available = self.test_microphone()
        
        # Initialize OpenAI client for high-accuracy Whisper
        self.openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

    def available_voice_files(self) -> List[str]:
        """Return list of .wav files sorted alphabetically."""
        return sorted([file.name for file in self.voice_directory.glob("*.wav")])

    def recognize(self, audio: sr.AudioData) -> Optional[str]:
        """Run speech recognition and handle errors uniformly."""

        try:
            # Use the official OpenAI Whisper API directly (requires internet)
            print("⏳ OpenAI Whisper API processing (Cloud)...")
            
            # Convert audio data to a file-like object for the API
            audio_data = BytesIO(audio.get_wav_data())
            audio_data.name = "audio.wav"

            response = self.openai_client.audio.transcriptions.create(
                model="whisper-1",
                file=audio_data
            )
            return response.text
            
        except Exception as e:
            print(f"   [Debug] API Recognition Error: {e}")
            # Fallback to local whisper if API/Internet fails
            try:
                print("   Falling back to local Whisper...")
                return self.recognizer.recognize_whisper(audio, model="base")
            except Exception as local_e:
                print(f"   [Debug] Local Recognition Error: {local_e}")
                return None


    def transcribe_file(self, filename: str) -> Optional[str]:
        """Convert a wav file inside the voice directory to text."""

        filepath = self.voice_directory / filename
        if not filepath.exists():
            return None

        try:
            with sr.AudioFile(str(filepath)) as source:
                audio = self.recognizer.record(source)
        except Exception:
            return None
        return self.recognize(audio)

    def record_microphone(self, duration: int = 8) -> Optional[str]:
        """Record audio for a fixed duration, display a countdown, and transcribe it."""

        if not self.microphone_available:
            print('❌ Microphone is not available.')
            return None
        
        print(f"🎙️ Starting recording in 1 second. Duration: {duration} seconds.")

        try:
            with suppress_audio_errors():
                with sr.Microphone() as source:
                    # Adjust for ambient noise
                    self.recognizer.adjust_for_ambient_noise(source, duration=1)

                    # --- Countdown Loop ---
                    # Print a clean line to separate logs/output and ensure \r starts cleanly
                    print("⏳ Recording starts now...                                         ")
                    
                    countdown_thread = threading.Thread(
                        target=self._display_countdown, args=(duration,), daemon=True
                    )
                    countdown_thread.start()
                    audio = self.recognizer.record(source, duration=duration)
                    countdown_thread.join()

                    # Clear the countdown line and indicate recording is complete
                    print("   Recording complete!                                            ")  # Spaces to overwrite previous line content
            # Save to file
            timestamp = int(time.time())
            filename = self.voice_directory / f"record_{timestamp}.wav"
            with open(filename, "wb") as f:
                f.write(audio.get_wav_data())
            print(f"💾 Saved recording to: {filename}")

        except Exception:
            return None

        return self.recognize(audio)

    def _display_countdown(self, duration: int) -> None:
        """Show a countdown timer while recording."""

        for remaining in range(duration, 0, -1):
            print(f"   Time remaining: {remaining} seconds   ", end='\r')
            time.sleep(1)
        print(" " * 60, end='\r')

    def continuous_listen(self, handler: Callable[[str], None]) -> None:
        """Continuously listen for wake word and forward commands."""

        if not self.microphone_available:
            print("❌ Microphone not available.")
            return

        print("🎤 Initializing microphone... (This may take a moment)")
        
        with sr.Microphone() as source:
            print("🎤 Adjusting for ambient noise... please wait.")
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
            print("✅ Ready. Listening... (Say 'stop listening' to exit)")

        while True:
            try:
                with sr.Microphone() as source:
                    # Listen for audio (blocking until sound is heard)
                    print("👂 Listening...")
                    audio = self.recognizer.listen(source, timeout=None, phrase_time_limit=5)
                
                print("⏳ Processing...")
                # Use the main recognize method which uses Whisper and FLAC
                transcript = self.recognize(audio)
                
                if not transcript:
                    print("🤷 Didn't catch that.")
                    continue

                transcript = transcript.lower()
                print(f"🗣️ Heard: '{transcript}'")

                if "stop listening" in transcript:
                    print("🛑 Stopping listener.")
                    break
                
                handler(transcript)

            except sr.WaitTimeoutError:
                pass 
            except sr.UnknownValueError:
                print("🤷 Didn't catch that.")
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"⚠️ Error: {e}")
                time.sleep(1)


    def test_microphone(self) -> bool:
        """Return True if a microphone can be initialized."""

        with suppress_audio_errors():
            try:
                mic_list = sr.Microphone.list_microphone_names()
                if not mic_list:
                    return False
                with sr.Microphone() as source:
                    self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                return True
            except Exception:
                return False

    def recognize(self, audio: sr.AudioData) -> Optional[str]:
        """Run speech recognition and handle errors uniformly."""

        try:
            # Use the OpenAI Whisper API for maximum accuracy
            # This requires an internet connection and your API key in .env
            print("⏳ OpenAI Whisper API processing (sending to server)...")
            api_key = os.getenv("OPENAI_API_KEY")
            if not api_key:
                print("⚠️ No API key found in .env, falling back to local recognition.")
                return self.recognizer.recognize_whisper(audio, model="base")
            
            return self.recognizer.recognize_openai(audio, api_key=api_key)
        except Exception as e:
            print(f"   [Debug] API Recognition Error: {e}")
            # Fallback to local whisper if API fails
            try:
                print("   Falling back to local Whisper...")
                return self.recognizer.recognize_whisper(audio, model="base")
            except:
                return None



__all__ = ["SpeechInterface"]