"""Command-line entrypoint for the Smart Stacker frontend."""
from __future__ import annotations
import sys
import os

# Ensure we can import from the current package when running as a script
if __name__ == "__main__" and __package__ is None:
    # Add the parent directory to sys.path so we can import modules
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "smart_stacker_frontend"

import argparse

from .smart_stacker.controllers.voice import HybridVoiceController



def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""

    parser = argparse.ArgumentParser(description="Voice-enabled stacker frontend")
    parser.add_argument("--unsafe", action="store_true", help="Disable safety validation")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    controller = HybridVoiceController(safe_mode=not args.unsafe)
    controller.connect()
    print(controller.show_capabilities())

    running = True

    try:
        command_handlers = {
            "positions": lambda: print(f"Current positions: {controller.current_positions}"),
            "files": lambda: print(controller.available_voice_files()),
            "metrics": lambda: print(controller.metrics()),
            "record": lambda: controller.record_voice(),
            "listen": lambda: controller.continuous_mode(),
        }

        while running:
            user_input = input("🎯 Enter command: ").strip().lower()

            result = None # Initialize result

            if user_input in {"quit", "exit", "q"}:
                running = False
            elif user_input in command_handlers:
                command_handlers[user_input]()
                # No result to process for these commands
            elif user_input.isdigit():
                result = controller.process_voice_file(int(user_input) - 1)
            else:
                result = controller.process_text(user_input)

            if running and result is not None:
                if result.is_safe:
                    print(f"✅ {result.message}: {result.command}")
                else:
                    print(f"⚠️ {result.message}")


    finally:
        controller.disconnect()

if __name__ == "__main__":
    main()