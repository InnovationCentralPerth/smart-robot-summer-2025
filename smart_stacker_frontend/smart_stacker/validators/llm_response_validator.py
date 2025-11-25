"""Validation helpers for LLM responses."""
from __future__ import annotations

from typing import Dict, Iterable, Set
import os

from ..config import DEFAULT_ANIMALS, DEFAULT_POSITIONS


def load_list_env(key: str, fallback: Iterable[str]) -> Set[str]:
    """Load a list-like environment variable and normalize its items.

    Accepts simple comma-separated strings (e.g. "E,L,F") **or** the
    set-style notation seen in the project's default .env file
    (e.g. {"E", "L", "F"}). Whitespace, surrounding quotes, and braces are
    stripped so the returned set contains clean tokens like {"E", "L", "F"}.
    """

    raw = os.getenv(key)
    if not raw:
        return {item.strip() for item in fallback}

    # Remove wrapping braces if present and split on commas
    cleaned = raw.strip().strip("{}")
    items = []
    for item in cleaned.split(","):
        normalized = item.strip().strip("'\"{} ")
        if normalized:
            items.append(normalized)

    return set(items)


class LLMResponseValidator:
    """Validate LLM JSON responses before execution."""

    def __init__(self) -> None:
        self.animal_list = load_list_env("VALID_ANIMALS", DEFAULT_ANIMALS)
        self.position_list = load_list_env("VALID_POSITIONS", DEFAULT_POSITIONS)

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