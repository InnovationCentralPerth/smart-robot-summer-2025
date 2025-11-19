"""Speech recognition helpers for voice driven commands."""
from __future__ import annotations

from typing import Callable, List, Optional

import speech_recognition as sr

from smart_stacker.config import SpeechConfig
from smart_stacker.utils.audio import suppress_audio_errors


class SpeechInterface:
    """High level interface for transcription and wake-word listening."""

    def __init__(self, config: Optional[SpeechConfig] = None) -> None:
        self.config = config or SpeechConfig()
        self.recognizer = sr.Recognizer()
        self.voice_directory = self.config.voice_directory
        self.voice_directory.mkdir(parents=True, exist_ok=True)
        self.microphone_available = self._test_microphone()

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

        TODO: Need to fix this function.

        if not self.microphone_available:
            return

        with suppress_audio_errors():
            with sr.Microphone() as source:
                self.recognizer.adjust_for_ambient_noise(source)

        listening = True

        while listening:
            with suppress_audio_errors():
                try:
                    with sr.Microphone() as source:
                        audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=2)
                    transcript = self._recognize(audio).lower()
                    if not transcript:
                        continue
                    if self.config.wake_word in transcript:
                        with sr.Microphone() as source:
                            command_audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
                        command_text = self._recognize(command_audio)
                        if not command_text:
                            continue
                        if self.config.listen_stop_phrase in command_text.lower():
                            break
                        handler(command_text)
                except sr.WaitTimeoutError:
                    continue
                except KeyboardInterrupt:
                    break

    def _test_microphone(self) -> bool:
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

    def _recognize(self, audio: sr.AudioData) -> Optional[str]:
        """Run speech recognition and handle errors uniformly."""

        try:
            return self.recognizer.recognize_google(audio)
        except sr.UnknownValueError:
            return None
        except Exception:
            return None


__all__ = ["SpeechInterface"]