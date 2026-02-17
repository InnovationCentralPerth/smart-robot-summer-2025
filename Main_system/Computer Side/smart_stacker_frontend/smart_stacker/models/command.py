"""Domain models for command handling."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional


@dataclass
class CommandMetrics:
    """Performance metrics captured for each user command."""

    input_command: str
    processing_time: float
    success: bool
    confidence_score: float
    error_type: Optional[str] = None
    json_output: Optional[Dict[str, str]] = None


@dataclass
class CommandResult:
    """Result returned from validating and dispatching commands."""

    command: Dict[str, str]
    is_safe: bool
    message: str = ""


__all__ = ["CommandMetrics", "CommandResult"]