#!/usr/bin/env python3
"""
Academic LLM Integration Module for Animal Stacker Robot
========================================================

This module demonstrates the integration of OpenAI Large Language Models (LLMs)
for natural language command interpretation in robotic systems.

Academic Context:
- Conference: Indo-Pacific Robotics Conference (IPRAAC)
- Application: Intelligent Pick-and-Place Robot Control
- Focus: Natural Language to Structured Command Translation

Key Concepts Demonstrated:
1. Prompt Engineering for Robotic Command Generation
2. JSON Schema Validation for Safety-Critical Robot Commands
3. Semantic Understanding of Spatial Relationships
4. Context-Aware Command Interpretation
5. Error Handling and Fallback Mechanisms
"""

import json
import time
import logging
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from dotenv import load_dotenv
import os
import re
import openai
from .smart_stacker.validators.llm_response_validator import LLMResponseValidator

# Load dotenv configurations

load_dotenv()

log_file = os.getenv("LOG_FILE", "default.log")
llm_model = os.getenv("MODEL", "gpt-4o-mini")
llm_base_prompt = os.getenv("PROMPT_FILE", "llm_system_prompt.txt")
llm_host = os.getenv("HOST_URL", "https://api.openai.com/v1")


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)

# Set OpenAI API key
openai.api_key = os.getenv("OPENAI_API_KEY")
if llm_host and "openai.com" not in llm_host:
    openai.base_url = llm_host


@dataclass
class CommandMetrics:
    input_command: str
    processing_time: float
    success: bool
    confidence_score: float
    error_type: Optional[str] = None
    json_output: Optional[Dict] = None


class AnimalStackerLLM:
    """OpenAI-based LLM integration for robotic command interpretation."""

    def __init__(self, model_name: str = llm_model, prompt_file: str = llm_base_prompt):
        self.model_name = model_name
        self.prompt_file = prompt_file
        self.metrics_log: List[CommandMetrics] = []

        # LLM response validator
        self.llm_response_validator = LLMResponseValidator()

        # Load system prompt
        self.system_prompt = self.load_system_prompt()

        # Define valid robot parameters
        self.valid_animals = {'E': 'Elephant', 'L': 'Lion', 'F': 'Baby Bear'}
        self.valid_positions = {
            'L1': 'Left Back', 'L2': 'Left Front',
            'C': 'Center Front',
            'R1': 'Right Back', 'R2': 'Right Front'
        }

        # Animal properties for semantic understanding
        self.animal_properties = {
            'E': {'size': 'large', 'weight': 'heavy', 'speed': 'slow'},
            'L': {'size': 'medium', 'weight': 'medium', 'speed': 'fast'},
            'F': {'size': 'small', 'weight': 'light', 'speed': 'slow'}
        }

        logging.info(f"Initialized AnimalStackerLLM with model: {model_name}")
        logging.info(f"Loaded system prompt from: {prompt_file}")

    def load_system_prompt(self) -> str:
        try:
            with open(self.prompt_file, 'r', encoding='utf-8') as f:
                prompt = f.read().strip()
            if not prompt:
                raise ValueError(f"Prompt file is empty: {self.prompt_file}")
            logging.info(f"Successfully loaded prompt ({len(prompt)} chars) from {self.prompt_file}")
            return prompt
        except FileNotFoundError:
            logging.error(f"Prompt file not found: {self.prompt_file}")
            return self.get_default_prompt()
        except Exception as e:
            logging.error(f"Error loading prompt file: {e}")
            return self.get_default_prompt()

    def get_default_prompt(self) -> str:
        return """
You are a command interpreter that converts natural language instructions into robot commands for placing three animals.

ANIMALS: E (Elephant), L (Lion), F (Baby Bear)
POSITIONS: L1, L2, C, R1, R2

GRID LAYOUT:
BACK:  L1   ROBOT   R1
FRONT: L2    C      R2

RULES:
1. Always output ONLY valid JSON in this form: {"E": "...", "L": "...", "F": "..."}
2. Each value must be one of: L1, L2, C, R1, R2
3. No two animals may be assigned to the same position.
4. If an animal is not mentioned or affected, keep its current position.
5. Understand spatial commands (left/right/center/front/back).
6. Do NOT output anything other than the JSON object.

TASK:
Convert user natural-language commands into the JSON robot instruction.
"""

    def call_llm(self, user_command: str, current_positions: Dict[str, str]) -> Tuple[Optional[Dict], float, str]:
        start_time = time.time()
        positions_json = json.dumps(current_positions)
        full_prompt = f"{self.system_prompt}\nCurrent positions: {positions_json}\nUser command: '{user_command}'\nGenerate robot command JSON:"

        logging.info(f"Prompt sent to OpenAI:\n{full_prompt}")

        try:
            response = openai.responses.create(
                model=self.model_name,
                input=full_prompt,
                temperature=0.1,
                max_output_tokens=150,
            )

            processing_time = time.time() - start_time
            llm_output = ""

            if hasattr(response, "output") and response.output:
                for item in response.output:
                    if item.type == "message":
                        for content in item.content:
                            if content["type"] == "output_text":
                                llm_output += content["text"]

            parsed_json = self.extract_json(llm_output)
            return parsed_json, processing_time, llm_output

        except Exception as e:
            processing_time = time.time() - start_time
            logging.error(f"OpenAI API error: {e}")
            return None, processing_time, f"OpenAI Error: {e}"

    def extract_json(self, llm_response: str) -> Optional[Dict]:
        try:
            return json.loads(llm_response)
        except json.JSONDecodeError:
            pass
        json_pattern = r'\{[^{}]*\}'
        matches = re.findall(json_pattern, llm_response)
        for match in matches:
            try:
                parsed = json.loads(match)
                if self.validate_command_json(parsed):
                    return parsed
            except json.JSONDecodeError:
                continue
        logging.warning(f"Could not extract valid JSON from: {llm_response}")
        return None

    def validate_command_json(self, command: Dict) -> bool:
        if not isinstance(command, dict):
            return False
        for animal in command.keys():
            if animal not in self.valid_animals:
                logging.warning(f"Invalid animal in command: {animal}")
                return False
        for position in command.values():
            if position not in self.valid_positions:
                logging.warning(f"Invalid position in command: {position}")
                return False
        return True

    def process_natural_language_command(self, user_input: str, current_positions: Dict[str, str]) -> Dict:
        logging.info(f"Processing command: '{user_input}'")
        parsed_json, processing_time, raw_response = self.call_llm(user_input, current_positions)

        metrics = CommandMetrics(
            input_command=user_input,
            processing_time=processing_time,
            success=False,
            confidence_score=0.0
        )

        if parsed_json:
            try:
                self.llm_response_validator.validate_llm_response(parsed_json)
                if self.validate_command_json(parsed_json):
                    complete_command = self.complete_command(parsed_json, current_positions)
                    metrics.success = True
                    metrics.confidence_score = self.calculate_confidence(parsed_json, user_input)
                    metrics.json_output = complete_command
                    self.metrics_log.append(metrics)
                    return complete_command
            except Exception as e:
                logging.warning(f"LLM output rejected by validator: {e}")

        metrics.error_type = "LLM_PARSING_FAILED"
        logging.warning(f"LLM failed to generate valid command. Using fallback.")
        fallback_command = self.academic_fallback(user_input, current_positions)
        metrics.json_output = fallback_command
        self.metrics_log.append(metrics)
        return fallback_command

    def complete_command(self, partial_command: Dict[str, str], current_positions: Dict[str, str]) -> Dict[str, str]:
        complete_command = current_positions.copy()
        complete_command.update(partial_command)
        return complete_command

    def calculate_confidence(self, command: Dict, user_input: str) -> float:
        confidence = 0.7
        if len(command) == 3:
            confidence += 0.2
        for pos in self.valid_positions.keys():
            if pos.lower() in user_input.lower():
                confidence += 0.05
        return min(confidence, 1.0)

    def academic_fallback(self, user_input: str, current_positions: Dict[str, str]) -> Dict[str, str]:
        logging.info("Using academic rule-based fallback system")
        user_lower = user_input.lower()
        if "front" in user_lower and "all" in user_lower:
            return {"E": "L2", "L": "C", "F": "R2"}
        elif "big to small" in user_lower or "large to small" in user_lower:
            return {"E": "L2", "L": "C", "F": "R2"}
        else:
            logging.info("Maintaining current positions as safe fallback")
            return current_positions.copy()

    def get_academic_metrics(self) -> Dict:
        if not self.metrics_log:
            return {"message": "No commands processed yet"}
        total_commands = len(self.metrics_log)
        successful_commands = sum(1 for m in self.metrics_log if m.success)
        avg_processing_time = sum(m.processing_time for m in self.metrics_log) / total_commands
        avg_confidence = sum(m.confidence_score for m in self.metrics_log) / total_commands
        return {
            "total_commands_processed": total_commands,
            "success_rate": successful_commands / total_commands,
            "average_processing_time_seconds": round(avg_processing_time, 3),
            "average_confidence_score": round(avg_confidence, 3),
            "error_types": [m.error_type for m in self.metrics_log if m.error_type]
        }


# Academic Testing Interface
if __name__ == "__main__":
    llm = AnimalStackerLLM()
    test_commands = [
        "line up animals from big to small",
        "move all animals to the front row",
        "put the elephant in the back left corner",
        "arrange them in size order",
        "move everything to back positions"
    ]
    current_pos = {"E": "L2", "L": "C", "F": "R2"}
    print("=== Academic LLM Integration Demo ===")
    for cmd in test_commands:
        print(f"\nTesting: '{cmd}'")
        result = llm.process_natural_language_command(cmd, current_pos)
        print(f"Result: {result}")
    print("\n=== Academic Performance Metrics ===")
    print(json.dumps(llm.get_academic_metrics(), indent=2))
