"""
CLI Controller for Braccio Robot.

Provides a threaded command-line interface for natural language input.
"""

import threading
import queue


class CLIController:
    """CLI controller for natural language commands."""
    
    def __init__(self):
        self.command_queue = queue.Queue()
        self.running = True
        self.thread = threading.Thread(target=self._input_loop, daemon=True)
        self.thread.start()

    def _input_loop(self):
        print("\n" + "="*60)
        print("  BRACCIO + LLM - Natural Language Control")
        print("  ")
        print("  Enter natural language commands, e.g.:")
        print("    'put the green cube on top of the red cube'")
        print("    'move the red cube next to the green cube'")
        print("    'stack the cubes'")
        print("  ")
        print("  Special commands:")
        print("    home  -> Return to home position")
        print("    exit  -> Quit")
        print("="*60)
        while self.running:
            try:
                user_input = input("\n> ").strip()
                if user_input.lower() == "exit": 
                    self.running = False
                    break
                if user_input:
                    self.command_queue.put(user_input)
            except: 
                pass

    def get_command(self):
        if not self.command_queue.empty(): 
            return self.command_queue.get()
        return None
