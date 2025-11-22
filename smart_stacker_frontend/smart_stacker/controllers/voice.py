"""High level voice controller orchestrating speech, LLM, and MQTT services."""
from __future__ import annotations

import json
from typing import Dict, Optional

from ..config import MQTTConfig, SpeechConfig
from ..models.command import CommandResult
from ..services.llm_client import AnimalStackerLLM
from ..services.mqtt_client import MQTTEventClient
from ..services.speech import SpeechInterface
from ..validators.llm_response_validator import LLMResponseValidator


class HybridVoiceController:
    """Controller that supports wav files, live microphone, and direct text input."""

    def __init__(
        self,
        mqtt_config: Optional[MQTTConfig] = None,
        speech_config: Optional[SpeechConfig] = None,
        safe_mode: bool = True,
    ) -> None:
        self.llm = AnimalStackerLLM()
        self.mqtt_client = MQTTEventClient(config=mqtt_config)
        self.speech = SpeechInterface(config=speech_config)
        self.validator = LLMResponseValidator() if safe_mode else None

    @property
    def current_positions(self) -> Dict[str, str]:
        """Expose the latest positions from the MQTT client."""

        return self.mqtt_client.current_positions

    def connect(self) -> None:
        """Initialize MQTT connectivity."""

        self.mqtt_client.connect()

    def disconnect(self) -> None:
        """Tear down MQTT connectivity."""

        self.mqtt_client.disconnect()

    def available_voice_files(self) -> str:
        """Return a formatted string describing available wav files."""

        files = self.speech.available_voice_files()
        if not files:
            return "No WAV files available"
        lines = ["Available WAV files:"]
        lines.extend(f"  {i}. {file}" for i, file in enumerate(files, 1))
        return "\n".join(lines)

    def show_capabilities(self) -> str:
        """Summarize available command entry modes."""

        parts = ["🎤 VOICE INPUT CAPABILITIES:", "=" * 40]
        files = self.speech.available_voice_files()
        if files:
            parts.append(f"✅ WAV File Processing: {len(files)} files available")
        else:
            parts.append("❌ WAV File Processing: No files in voice_cmds/")
        if self.speech.microphone_available:
            parts.append("✅ Live Microphone: Available")
        else:
            parts.append("❌ Live Microphone: Not available in this environment")
        return "\n".join(parts)

    def process_text(self, user_input: str) -> CommandResult:
        """Process direct text input and publish to MQTT."""

        command = self.llm.generate_robot_command(user_input, self.current_positions)
        return self._dispatch(command)

    def process_voice_file(self, file_index: int) -> Optional[CommandResult]:
        """Process command from a wav file by index in the list."""

        files = self.speech.available_voice_files()
        if file_index < 0 or file_index >= len(files):
            return None
        text = self.speech.transcribe_file(files[file_index])
        if not text:
            return None
        return self._complete_pipeline(text)

    def record_voice(self, duration: int = 8) -> Optional[CommandResult]:
        """Record audio from microphone and process it."""

        text = self.speech.record_microphone(duration)
        if not text:
            return None
        return self._complete_pipeline(text)

    def continuous_mode(self) -> None:
        """Run wake-word continuous listening."""

        def handler(transcript: str) -> None:
            self._complete_pipeline(transcript)

        self.speech.continuous_listen(handler)

    def _complete_pipeline(self, transcript: str) -> Optional[CommandResult]:
        command = self.llm.generate_robot_command(transcript, self.current_positions)
        return self._dispatch(command)

    def _dispatch(self, command: Dict[str, str]) -> CommandResult:
        """Validate, publish, and wait for completion."""

        if self.validator:
            try:
                self.validator.validate(command)
            except ValueError as exception:
                return CommandResult(command=self.current_positions.copy(), is_safe=False, message=str(exception))

        self.mqtt_client.publish_command(command)
        self.mqtt_client.wait_for_completion()
        return CommandResult(command=command, is_safe=True, message="Command executed")

    def metrics(self) -> str:
        """Return formatted metrics for display."""

        return json.dumps(self.llm.metrics(), indent=2)


__all__ = ["HybridVoiceController"]