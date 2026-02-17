from dotenv import load_dotenv
import os

load_dotenv()

def load_list_env(key: str) -> set:
    """Load comma-separated environment variable as a set."""
    raw = os.getenv(key)
    if not raw:
        raise ValueError(f"Missing environment variable: {key}")
    return {item.strip() for item in raw.split(",") if item.strip()}

class LLMResponseValidator:
    def __init__(self):
        self.animal_list = load_list_env("VALID_ANIMALS")
        self.position_list = load_list_env("VALID_POSITIONS")

    def validate_format(self, data: dict) -> None:
        """Validate JSON structure and required keys."""
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

    def validate_no_duplicates(self, data: dict) -> None:
        """Ensure positions are not duplicated across animals."""
        positions = list(data.values())
        duplicates = {position for position in positions if positions.count(position) > 1}
        if duplicates:
            raise ValueError(f"Duplicate positions found: {duplicates}")

    def validate(self, data: dict) -> None:
        """Run all validation checks."""
        self.validate_format(data)
        self.validate_no_duplicates(data)
