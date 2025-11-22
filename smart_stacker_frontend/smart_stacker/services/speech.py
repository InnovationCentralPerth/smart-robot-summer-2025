"""Speech recognition helpers for voice driven commands."""
from __future__ import annotations

from typing import Callable, List, Optional

import speech_recognition as sr

from ..config import SpeechConfig
from ..utils.audio import suppress_audio_errors


class SpeechInterface:
    """High level interface for transcription and wake-word listening."""

    def __init__(self, config: Optional[SpeechConfig] = None) -> None:
        self.config = config or SpeechConfig()
        self.recognizer = sr.Recognizer()
        self.voice_directory = self.config.voice_directory
        self.voice_directory.mkdir(parents=True, exist_ok=True)
        self.microphone_available = self.test_microphone()

    def available_voice_files(self) -> List[str]:
        """Return list of .wav files sorted alphabetically."""

        return sorted([file.name for file in self.voice_directory.glob("*.wav")])

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
        return self._recognize(audio)

    def record_microphone(self, duration: int = 8) -> Optional[str]:
        """Record audio for a fixed duration and transcribe it."""

        if not self.microphone_available:
            return None

        try:
            with suppress_audio_errors():
                with sr.Microphone() as source:
                    self.recognizer.adjust_for_ambient_noise(source, duration=1)
                    audio = self.recognizer.record(source, duration=duration)
        except Exception:
            return None
        return self._recognize(audio)

    def continuous_listen(self, handler: Callable[[str], None]) -> None:
            """Continuously listen for wake word and forward commands."""

            if not self.microphone_available:
                return

            # 1. Initial adjustment for ambient noise (only needs to be done once)
            with suppress_audio_errors():
                with sr.Microphone() as source:
                    self.recognizer.adjust_for_ambient_noise(source)

            listening = True
            
            # 2. Use the `listening` flag as a clear loop termination condition
            while listening:
                try:
                    # --- Step 1: Listen for the wake word ---
                    with suppress_audio_errors():
                        # Set a short timeout/phrase_time_limit for the wake word listen phase
                        with sr.Microphone() as source:
                            audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=2)
                    
                    # Check for successful transcription
                    transcript = self.recognize(audio).lower()
                    
                    # Use standard conditional logic instead of `if not transcript: continue`
                    if transcript and self.config.wake_word in transcript:
                        
                        # --- Step 2: Listen for the command after wake word is detected ---
                        with suppress_audio_errors():
                            # Use a longer timeout/phrase_time_limit for the command phase
                            with sr.Microphone() as source:
                                command_audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
                        
                        command_text = self.recognize(command_audio)
                        
                        # Check for successful command transcription
                        if command_text:
                            # Use standard conditional logic instead of `if stop_phrase: break`
                            if self.config.listen_stop_phrase in command_text.lower():
                                listening = False  # Set flag to terminate the while loop
                            else:
                                handler(command_text)
                                
                # 3. Handle expected exceptions by letting the loop naturally continue
                except sr.WaitTimeoutError:
                    # This is normal if nothing is said during the wake word phase.
                    # The loop will re-execute naturally.
                    pass
                except KeyboardInterrupt:
                    listening = False  # Set flag to terminate the while loop
                except Exception:
                    # Handle other unexpected errors and allow the loop to continue
                    pass

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
            return self.recognizer.recognize_google(audio)
        except sr.UnknownValueError:
            return None
        except Exception:
            return None


__all__ = ["SpeechInterface"]