"""Validation helpers for LLM responses."""
from __future__ import annotations

from typing import Dict, Iterable, Set
import os

from smart_stacker.config import DEFAULT_ANIMALS, DEFAULT_POSITIONS


def load_list_env(key: str, fallback: Iterable[str]) -> Set[str]:
    """Load comma-separated environment variable with fallback values."""

    raw = os.getenv(key)
    if not raw:
        return {item.strip() for item in fallback}
    return {item.strip() for item in raw.split(",") if item.strip()}


class LLMResponseValidator:
    """Validate LLM JSON responses before execution."""

    def __init__(self) -> None:
        self.animal_list = _load_list_env("VALID_ANIMALS", DEFAULT_ANIMALS)
        self.position_list = _load_list_env("VALID_POSITIONS", DEFAULT_POSITIONS)

    def validate_format(self, data: Dict[str, str]) -> None:
        """Validate dictionary shape and contents."""

        if not isinstance(data, dict):
            raise ValueError("Response must be a dictionary.")

        missing = self.animal_list - data.keys()
        if missing:
            raise ValueError(f"Missing animals: {missing}")

        extra = data.keys() - self.animal_list
        if extra:
            raise ValueError(f"Unexpected animals: {extra}")

        for animal, position in data.items():
            if not isinstance(position, str):
                raise ValueError(f"Position for {animal} must be a string.")
            if position not in self.position_list:
                raise ValueError(f"Invalid position '{position}' for {animal}.")

    def validate_no_duplicates(self, data: Dict[str, str]) -> None:
        """Ensure animals are not assigned to the same position."""

        seen: Set[str] = set()
        duplicates: Set[str] = set()
        for position in data.values():
            if position in seen:
                duplicates.add(position)
            seen.add(position)

        if duplicates:
            raise ValueError(f"Duplicate positions found: {duplicates}")

    def validate(self, data: Dict[str, str]) -> None:
        """Run all validation steps."""

        self.validate_format(data)
        self.validate_no_duplicates(data)


__all__ = ["LLMResponseValidator"]