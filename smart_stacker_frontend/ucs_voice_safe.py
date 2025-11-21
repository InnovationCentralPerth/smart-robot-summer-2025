"""Legacy entrypoint preserved for backward compatibility (safe mode)."""
from smart_stacker.controllers.voice import HybridVoiceController

def main() -> None:
    controller = HybridVoiceController(safe_mode=True)
    controller.connect()
    print(controller.show_capabilities())
    try:
        while True:
            user_input = input("🎯 Enter command: ").strip()
            if user_input.lower() in {"quit", "exit", "q"}:
                break
            if user_input.lower() == "positions":
                print(f"Current positions: {controller.current_positions}")
                continue
            if user_input.lower() == "files":
                print(controller.available_voice_files())
                continue
            if user_input.lower() == "metrics":
                print(controller.metrics())
                continue
            if user_input.lower() == "record":
                result = controller.record_voice()
            elif user_input.lower() == "listen":
                controller.continuous_mode()
                continue
            elif user_input.isdigit():
                result = controller.process_voice_file(int(user_input) - 1)
            else:
                result = controller.process_text(user_input)

            if result is None:
                print("No command generated. Try again." )
            elif result.is_safe:
                print(f"✅ {result.message}: {result.command}")
            else:
                print(f"⚠️ {result.message}")
    finally:
        controller.disconnect()

if __name__ == "__main__":
    main()