"""LLM integration responsible for translating natural language to commands."""
from __future__ import annotations

import json
import logging
import re
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import requests

from ..config import AnimalConfig, LLMConfig
from ..models.command import CommandMetrics


class AnimalStackerLLM:
    """Encapsulates prompt loading, LLM calls, and fallback logic."""

    def __init__(self, llm_config: Optional[LLMConfig] = None, animal_config: Optional[AnimalConfig] = None) -> None:
        self.config = llm_config or LLMConfig()
        self.animals = animal_config or AnimalConfig()
        self.metrics_log: List[CommandMetrics] = []
        self.system_prompt = self.load_system_prompt(self.config.prompt_file)
        self.setup_logging(self.config.log_file)

    def generate_robot_command(self, user_input: str, current_positions: Dict[str, str]) -> Dict[str, str]:
        """Convert natural language into a command dictionary."""

        parsed_json, processing_time, _ = self.call_llm(user_input, current_positions)
        metrics = CommandMetrics(
            input_command=user_input,
            processing_time=processing_time,
            success=False,
            confidence_score=0.0,
        )

        if parsed_json and self.validate_command(parsed_json):
            command = self.complete_command(parsed_json, current_positions)
            metrics.success = True
            metrics.confidence_score = self.calculate_confidence(parsed_json, user_input)
            metrics.json_output = command
            self.metrics_log.append(metrics)
            return command

        metrics.error_type = "LLM_PARSING_FAILED"
        metrics.json_output = current_positions.copy()
        self.metrics_log.append(metrics)
        return self.fallback_command(user_input, current_positions)

    def metrics(self) -> Dict[str, float | int | List[str] | str]:
        """Return aggregate metrics for display."""

        if not self.metrics_log:
            return {"message": "No commands processed yet"}

        total_commands = len(self.metrics_log)
        successful_commands = sum(1 for metric in self.metrics_log if metric.success)
        avg_processing_time = sum(metric.processing_time for metric in self.metrics_log) / total_commands
        avg_confidence = sum(metric.confidence_score for metric in self.metrics_log) / total_commands

        return {
            "total_commands_processed": total_commands,
            "success_rate": successful_commands / total_commands,
            "average_processing_time_seconds": round(avg_processing_time, 3),
            "average_confidence_score": round(avg_confidence, 3),
            "error_types": [metric.error_type for metric in self.metrics_log if metric.error_type],
        }

    def load_system_prompt(self, prompt_file: Path) -> str:
        """Load system prompt from disk or fallback to default text."""

        try:
            text = prompt_file.read_text(encoding="utf-8").strip()
            if not text:
                raise ValueError("Prompt file is empty")
            return text
        except FileNotFoundError:
            logging.warning("Prompt file not found. Using default prompt text.")
            return self.default_prompt()

    def default_prompt(self) -> str:
        """Return built-in default prompt."""

        return (
            "You are an intelligent user command interpreter to send commands to an animal stacking robot."
            " Only use animals E, L, F and positions L1, L2, C, R1, R2."
            " Animals must never share a position and unaffected animals remain in place."
            " Respond with JSON only: {\"E\": \"pos\", \"L\": \"pos\", \"F\": \"pos\"}."
        )

    def setup_logging(self, log_file: Path) -> None:
        """Configure logging once for the module."""

        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s - %(levelname)s - %(message)s",
            handlers=[logging.FileHandler(log_file), logging.StreamHandler()],
        )

    def call_llm(self, user_command: str, current_positions: Dict[str, str]) -> Tuple[Optional[Dict[str, str]], float, str]:
        """Invoke the local LLM and return the parsed JSON if possible."""

        start_time = time.time()
        positions_json = json.dumps(current_positions)
        context_prompt = (
            f"Current position of animals is {positions_json}.\n"
            f"User command: \"{user_command}\"\n"
            "Generate robot command JSON:"
        )
        full_prompt = f"{self._system_prompt}\n{context_prompt}"

        try:
            response = requests.post(
                f"{self.config.host_url}/api/generate",
                json={
                    "model": self.config.model_name,
                    "prompt": full_prompt,
                    "stream": False,
                    "options": {"temperature": 0.1, "top_p": 0.9, "num_predict": 100},
                },
                timeout=30,
            )
            processing_time = time.time() - start_time

            if response.status_code == 200:
                llm_output = response.json().get("response", "").strip()
                logging.info("LLM raw response: %s", llm_output)
                parsed_json = self.extract_json(llm_output)
                return parsed_json, processing_time, llm_output

            logging.error("LLM API error: %s", response.status_code)
            return None, processing_time, f"API Error: {response.status_code}"

        except requests.RequestException as exc:
            processing_time = time.time() - start_time
            logging.error("LLM connection error: %s", exc)
            return None, processing_time, f"Connection Error: {exc}"

    def extract_json(self, llm_response: str) -> Optional[Dict[str, str]]:
        """Extract valid JSON from the LLM response."""

        try:
            parsed = json.loads(llm_response)
            if isinstance(parsed, dict):
                return parsed
        except json.JSONDecodeError:
            pass

        for match in re.findall(r"\{[^{}]*\}", llm_response):
            try:
                candidate = json.loads(match)
                if isinstance(candidate, dict) and self.validate_command(candidate):
                    return candidate
            except json.JSONDecodeError:
                continue

        logging.warning("Could not extract valid JSON from: %s", llm_response)
        return None

    def validate_command(self, command: Dict[str, str]) -> bool:
        """Verify animals and positions exist."""

        animal_keys = set(command.keys())
        valid_animals = set(self.animals.animals.keys())
        valid_positions = set(self.animals.positions.keys())

        if not animal_keys.issubset(valid_animals):
            invalid = animal_keys - valid_animals
            logging.warning("Invalid animals in command: %s", invalid)
            return False

        if any(pos not in valid_positions for pos in command.values()):
            invalid_positions = {pos for pos in command.values() if pos not in valid_positions}
            logging.warning("Invalid positions in command: %s", invalid_positions)
            return False

        return True

    def complete_command(self, partial: Dict[str, str], current_positions: Dict[str, str]) -> Dict[str, str]:
        """Ensure every animal has a target position."""

        completed = current_positions.copy()
        completed.update(partial)
        return completed

    def calculate_confidence(self, command: Dict[str, str], user_input: str) -> float:
        """Rough confidence heuristic based on coverage and explicit positions."""

        confidence = 0.7
        if len(command) == len(self.animals.animals):
            confidence += 0.2

        lowered_input = user_input.lower()
        for position in self.animals.positions:
            if position.lower() in lowered_input:
                confidence += 0.05
        return min(confidence, 1.0)

    def fallback_command(self, user_input: str, current_positions: Dict[str, str]) -> Dict[str, str]:
        """Simple heuristics to keep the robot deterministic when LLM fails."""

        if "front" in user_input and "all" in user_input:
            return {"E": "L2", "L": "C", "F": "R2"}
        if "big to small" in user_input or "large to small" in user_input:
            return {"E": "L2", "L": "C", "F": "R2"}
        return current_positions.copy()


__all__ = ["AnimalStackerLLM"]