"""Command-line entrypoint for the Smart Stacker frontend."""
from __future__ import annotations

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

            if user_input in {"quit", "exit", "q"}:
                running = False
            elif user_input in command_handlers:
                result = command_handlers[user_input]()
                if user_input == "listen":
                    result = None
            elif user_input.isdigit():
                result = controller.process_voice_file(int(user_input) - 1)
            else:
                result = controller.process_text(user_input)

            if running:
                if result is None:
                    print("No command generated. Try again.")
                elif result.is_safe:
                    print(f"✅ {result.message}: {result.command}")
                else:
                    print(f"⚠️ {result.message}")

    finally:
        controller.disconnect()

if __name__ == "__main__":
    main()