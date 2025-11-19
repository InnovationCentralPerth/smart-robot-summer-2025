"""Application configuration helpers for the Smart Stacker frontend."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict
import os

from dotenv import load_dotenv

load_dotenv()

DEFAULT_ANIMALS = {"E", "L", "F"}
DEFAULT_POSITIONS = {"L1", "L2", "C", "R1", "R2"}


@dataclass(frozen=True)
class LLMConfig:
    """Configuration for the LLM client."""

    host_url: str = os.getenv("HOST_URL", "http://localhost:11434")
    model_name: str = os.getenv("MODEL", "gemma3:4b")
    prompt_file: Path = Path(os.getenv("PROMPT_FILE", "llm_system_prompt.txt"))
    log_file: Path = Path(os.getenv("LOG_FILE", "default.log"))


@dataclass(frozen=True)
class MQTTConfig:
    """MQTT broker configuration."""

    host: str = os.getenv("MQTT_HOST", "192.168.1.101")
    port: int = int(os.getenv("MQTT_PORT", "1883"))
    status_topic: str = os.getenv("STATUS_TOPIC", "stacker/status")
    positions_topic: str = os.getenv("POSITIONS_TOPIC", "stacker/positions")
    command_topic: str = os.getenv("COMMAND_TOPIC", "stacker/command")


@dataclass(frozen=True)
class SpeechConfig:
    """Settings for voice commands."""

    voice_directory: Path = Path(os.getenv("VOICE_COMMAND_DIR", "voice_cmds"))
    wake_word: str = os.getenv("VOICE_WAKE_WORD", "robot")
    listen_stop_phrase: str = os.getenv("VOICE_STOP_PHRASE", "stop listening")


@dataclass(frozen=True)
class AnimalConfig:
    """Available animals and valid target positions."""

    animals: Dict[str, str] = field(
        default_factory=lambda: {
            "E": "Elephant",
            "L": "Lion",
            "F": "Baby Bear",
        }
    )
    positions: Dict[str, str] = field(
        default_factory=lambda: {
            "L1": "Left Back",
            "L2": "Left Front",
            "C": "Center Front",
            "R1": "Right Back",
            "R2": "Right Front",
        }
    )


__all__ = [
    "LLMConfig",
    "MQTTConfig",
    "SpeechConfig",
    "AnimalConfig",
    "DEFAULT_ANIMALS",
    "DEFAULT_POSITIONS",
]