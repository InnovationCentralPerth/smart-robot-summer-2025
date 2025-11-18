#!/usr/bin/env python3
"""
Academic LLM Integration Module for Animal Stacker Robot
========================================================

This module demonstrates the integration of Large Language Models (LLMs) 
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

Author: Academic Demo Team
Purpose: Educational demonstration of GenAI in robotics
"""

import json
import requests
import time
import logging
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from dotenv import load_dotenv
import os

# Load dotenv configurations
load_dotenv()

log_file = os.getenv("LOG_FILE", "default.log")
llm_model = os.getenv("MODEL", "gemma3:4b")
llm_host = os.getenv("HOST_URL", "http://localhost:11434")
llm_base_prompt = os.getenv("PROMPT_FILE", "llm_system_prompt.txt")

# Configure logging for academic analysis
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)

@dataclass
class CommandMetrics:
    """Metrics for academic evaluation of LLM performance"""
    input_command: str
    processing_time: float
    success: bool
    confidence_score: float
    error_type: Optional[str] = None
    json_output: Optional[Dict] = None

class AnimalStackerLLM:
    """
    Academic demonstration of LLM integration for robotic command interpretation.
    
    This class showcases how modern LLMs can be used to bridge the gap between
    natural human language and precise robotic control commands.
    """
    
    def __init__(self, model_name: str = llm_model, base_url: str = llm_host,
    
        prompt_file: str = llm_base_prompt):
        self.model_name = model_name
        self.base_url = base_url
        self.prompt_file = prompt_file
        self.metrics_log: List[CommandMetrics] = []
        
        # Load system prompt from file
        self.system_prompt = self.load_system_prompt()
        
        # Academic validation: Define valid robot parameters
        self.valid_animals = {'E': 'Elephant', 'L': 'Lion', 'F': 'Baby Bear'}
        self.valid_positions = {
            'L1': 'Left Back', 'L2': 'Left Front', 
            'C': 'Center Front',
            'R1': 'Right Back', 'R2': 'Right Front'
        }
        
        # Animal characteristics for semantic understanding
        self.animal_properties = {
            'E': {'size': 'large', 'weight': 'heavy', 'speed': 'slow'},
            'L': {'size': 'medium', 'weight': 'medium', 'speed': 'fast'},
            'F': {'size': 'small', 'weight': 'light', 'speed': 'slow'}
        }
        
        logging.info(f"Initialized AnimalStackerLLM with model: {model_name}")
        logging.info(f"Loaded system prompt from: {prompt_file}")
    
    def load_system_prompt(self) -> str:
        """
        Load system prompt from external file for research flexibility.
        
        This allows researchers to:
        - Modify prompts without code changes
        - Test different prompt engineering approaches
        - Compare prompt effectiveness
        - Version control prompt iterations
        """
        try:
            with open(self.prompt_file, 'r', encoding='utf-8') as f:
                prompt = f.read().strip()

            if not prompt:
                raise ValueError(f"Prompt file is empty: {self.prompt_file}")

            logging.info(
                f"Successfully loaded prompt ({len(prompt)} chars) from {self.prompt_file}"
            )
            return prompt

        except FileNotFoundError:
            logging.error(f"Prompt file not found: {self.prompt_file}")
            raise

        except Exception as e:
            logging.error(f"Error loading prompt file: {e}")
            raise
    
    def get_default_prompt(self) -> str:
        """Fallback default prompt if file loading fails"""
        return  """
                    You are an intelligent user command interpreter to send commands to an animal (toy) stacking robot.

                    IMPORTANT RULES:
                    1. Only use these animals: E, L, F
                    2. Only use these positions: L1, L2, C, R1, R2
                    3. Never output a "new position" that is the same for more than one animal
                    e.g. Do not send a JSON command like  {'E': 'L1', 'L': 'C', 'F': 'L1'} as both E and F are sent to L1 position
                    4. Unaffected animals are to remain in same position 
                    5. Understand spatial concepts: left, right, center, front, back
                    6. Respond with JSON ONLY - no explanations or additional text

                    Spatially

                    BACK ROW  - L1  ROBOT  R1
                    FRONT ROW - L2    C    R2            		

                    - Animal properties: (E) Elephant=largest size/heaviest, (L) Lion=medium size/fastest, (F) Baby Bear=smallest size/slowest

                    TASK: Convert natural language commands into JSON robot instructions.

                    OUTPUT FORMAT: Always respond with ONLY valid JSON in this exact format:
                    {"E": "new position", "L": "new position", "F": "new position"}
                    .. where "new position" is one of from Available positions above

                    IMPORTANT RULES:
                    1. Only use these animals: E, L, F
                    2. Only use these positions: L1, L2, C, R1, R2
                    3. Animals must never be commanded to move to occupy the same position
                    e.g. DO NOT SEND COMMAND  {'E': 'L1', 'L': 'C', 'F': 'L1'} as both E and F are sent to L1 position4. Unaffected animals are to remain in same position 
                    5. Understand spatial concepts: left, right, center, front, back
                    6. Respond with JSON ONLY - no explanations or additional text
                """

    def call_llm(self, user_command: str, current_positions: Dict[str, str]) -> Tuple[Optional[Dict], float, str]:
        """
        Academic LLM Integration: Demonstrates HTTP API communication with
        local LLM deployment for real-time robot command generation.

        Returns:
            Tuple of (parsed_json, processing_time, raw_response)
        """
        start_time = time.time()

        # Construct context-aware prompt with current robot state
        # Convert current positions to JSON string for better clarity
        positions_json = json.dumps(current_positions)
        context_prompt = f"""
                            Current position of animals is {positions_json}.\n
                            User command: "{user_command}"\n
                            Generate robot command JSON:
                        """

        # Construct the full prompt that will be sent to LLM
        full_prompt = self.system_prompt() + "\n" + context_prompt

        # Log the complete command string sent to the language model
        logging.info(f"Command string sent to LLM:\n{full_prompt}")

        try:
            # Academic demonstration: REST API call to local LLM
            response = requests.post(
                f"{self.base_url}/api/generate",
                json={
                    "model": self.model_name,
                    "prompt": full_prompt,
                    "stream": False,
                    "options": {
                        "temperature": 0.1,  # Low temperature for consistent robot commands
                        "top_p": 0.9,
                        "num_predict": 100   # Limit output length for efficiency
                    }
                },
                timeout=30
            )
            
            processing_time = time.time() - start_time
            
            if response.status_code == 200:
                llm_output = response.json()['response'].strip()
                logging.info(f"LLM raw response: {llm_output}")
                
                # Extract JSON from LLM response
                parsed_json = self.extract_json(llm_output)
                return parsed_json, processing_time, llm_output
            else:
                logging.error(f"LLM API error: {response.status_code}")
                return None, processing_time, f"API Error: {response.status_code}"
                
        except requests.exceptions.RequestException as e:
            processing_time = time.time() - start_time
            logging.error(f"LLM connection error: {e}")
            return None, processing_time, f"Connection Error: {e}"
        except Exception as e:
            processing_time = time.time() - start_time
            logging.error(f"LLM processing error: {e}")
            return None, processing_time, f"Processing Error: {e}"
    
    def extract_json(self, llm_response: str) -> Optional[Dict]:
        """
        Academic JSON Extraction: Robust parsing of LLM output to extract
        valid robot commands while handling various response formats.
        """
        # Try direct JSON parsing first
        try:
            return json.loads(llm_response)
        except json.JSONDecodeError:
            pass
        
        # Try to find JSON within the response text
        import re
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
        """
        Academic Validation: Safety-critical validation of robot commands
        to ensure only valid, safe operations are executed.
        """
        if not isinstance(command, dict):
            return False
        
        # Validate all keys are valid animals
        for animal in command.keys():
            if animal not in self.valid_animals:
                logging.warning(f"Invalid animal in command: {animal}")
                return False
        
        # Validate all values are valid positions  
        for position in command.values():
            if position not in self.valid_positions:
                logging.warning(f"Invalid position in command: {position}")
                return False
        
        return True
    
    def process_natural_language_command(self, user_input: str, current_positions: Dict[str, str]) -> Dict:
        """
        Academic Main Processing Function: Demonstrates complete pipeline from
        natural language input to validated robot command output.
        
        This showcases the integration of multiple AI/ML concepts:
        - Natural Language Processing
        - Prompt Engineering  
        - JSON Schema Validation
        - Error Handling and Recovery
        - Performance Metrics Collection
        """
        
        logging.info(f"Processing command: '{user_input}'")
        
        # Call LLM for command interpretation
        parsed_json, processing_time, raw_response = self.call_llm(user_input, current_positions)
        
        # Initialize metrics for academic evaluation
        metrics = CommandMetrics(
            input_command=user_input,
            processing_time=processing_time,
            success=False,
            confidence_score=0.0
        )
        
        if parsed_json and self.validate_command_json(parsed_json):
            # Success case: Valid robot command generated
            complete_command = self.complete_command(parsed_json, current_positions)
            metrics.success = True
            metrics.confidence_score = self.calculate_confidence(parsed_json, user_input)
            metrics.json_output = complete_command
            
            logging.info(f"Successfully generated command: {complete_command}")
            self.metrics_log.append(metrics)
            return complete_command
            
        else:
            # Failure case: Use academic fallback strategy
            metrics.error_type = "LLM_PARSING_FAILED"
            logging.warning(f"LLM failed to generate valid command. Using fallback.")
            
            fallback_command = self.academic_fallback(user_input, current_positions)
            metrics.json_output = fallback_command
            self.metrics_log.append(metrics)
            return fallback_command
    
    def complete_command(self, partial_command: Dict[str, str], current_positions: Dict[str, str]) -> Dict[str, str]:
        """
        Academic Command Completion: Ensures all animals have defined positions,
        maintaining safety by preserving current state for unspecified animals.
        """
        complete_command = current_positions.copy()
        complete_command.update(partial_command)
        return complete_command
    
    def calculate_confidence(self, command: Dict, user_input: str) -> float:
        """
        Academic Confidence Scoring: Demonstrates confidence estimation 
        techniques for AI-generated robot commands.
        """
        # Simple heuristic-based confidence (can be enhanced with ML models)
        confidence = 0.7  # Base confidence for valid JSON
        
        # Boost confidence for complete commands
        if len(command) == 3:
            confidence += 0.2
        
        # Boost confidence for specific position mentions
        for pos in self.valid_positions.keys():
            if pos.lower() in user_input.lower():
                confidence += 0.05
        
        return min(confidence, 1.0)
    
    def academic_fallback(self, user_input: str, current_positions: Dict[str, str]) -> Dict[str, str]:
        """
        Academic Fallback Strategy: Demonstrates robust system design with
        graceful degradation when AI components fail.
        """
        logging.info("Using academic rule-based fallback system")
        
        # Simple rule-based interpretation for common commands
        user_lower = user_input.lower()
        
        if "front" in user_lower and "all" in user_lower:
            return {"E": "L2", "L": "C", "F": "R2"}
        elif "big to small" in user_lower or "large to small" in user_lower:
            return {"E": "L2", "L": "C", "F": "R2"}
        else:
            # Safe fallback: maintain current positions
            logging.info("Maintaining current positions as safe fallback")
            return current_positions.copy()
    
    def get_academic_metrics(self) -> Dict:
        """
        Academic Metrics Collection: Provides performance statistics
        for research analysis and poster presentation.
        """
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
    # Demonstration of academic LLM integration
    llm = AnimalStackerLLM()
    
    # Test commands for academic evaluation
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
