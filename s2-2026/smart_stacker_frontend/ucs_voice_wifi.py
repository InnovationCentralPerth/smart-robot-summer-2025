#!/usr/bin/env python3
"""
Enhanced Voice-Enabled UCS with Live Microphone Support
=======================================================

Hybrid system supporting both:
1. Pre-recorded WAV files (guaranteed to work)
2. Live microphone input (when available in WSL2)

For IPRAAC Conference demonstration
"""

import json
import time
import os
import threading
import queue
import sys
import contextlib
import paho.mqtt.client as mqtt
from llm_integration import AnimalStackerLLM
import speech_recognition as sr

@contextlib.contextmanager
def suppress_audio_errors():
    """Context manager to suppress verbose ALSA/JACK audio system errors"""
    # Save the original stderr file descriptor
    stderr_fd = sys.stderr.fileno()

    # Open /dev/null
    with open(os.devnull, 'w') as devnull:
        # Save a copy of the original stderr
        old_stderr = os.dup(stderr_fd)

        try:
            # Redirect stderr to /dev/null at the file descriptor level
            os.dup2(devnull.fileno(), stderr_fd)
            yield
        finally:
            # Restore the original stderr
            os.dup2(old_stderr, stderr_fd)
            os.close(old_stderr)

class HybridVoiceUCS:
    def __init__(self, broker_host="192.168.1.101", broker_port=1883):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.client = mqtt.Client()
        self.status_received = False
        self.last_status = None
        self.current_positions = {"E": "L2", "L": "C", "F": "R2"}
        self.position_update_received = False
        self.initial_setup_complete = False

        # Initialize LLM
        self.llm = AnimalStackerLLM()

        # Initialize speech recognition
        self.recognizer = sr.Recognizer()

        # Voice command capabilities
        self.voice_commands_dir = "voice_cmds"
        self.available_voice_files = self.scan_voice_files()
        self.microphone_available = self.test_microphone_access()

        # Recording state
        self.is_recording = False
        self.audio_queue = queue.Queue()

        # Setup MQTT
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
    def scan_voice_files(self):
        """Scan for available voice command files"""
        voice_files = []
        if os.path.exists(self.voice_commands_dir):
            for file in os.listdir(self.voice_commands_dir):
                if file.endswith('.wav'):
                    voice_files.append(file)
        return sorted(voice_files)
    
    def test_microphone_access(self, verbose=True):
        """Test if we can access microphone for live recording"""
        if verbose and not self.initial_setup_complete:
            print("🎤 Testing microphone access...")

        # Wrap the entire test in error suppression
        with suppress_audio_errors():
            try:
                # Quick test to see if microphone is accessible
                mic_list = sr.Microphone.list_microphone_names()
                if not mic_list:
                    if verbose and not self.initial_setup_complete:
                        print("❌ Microphone Failed")
                    return False

                # Try to initialize microphone
                with sr.Microphone() as source:
                    self.recognizer.adjust_for_ambient_noise(source, duration=0.5)

                if verbose and not self.initial_setup_complete:
                    print("✅ Microphone OK")
                return True

            except Exception as e:
                if verbose and not self.initial_setup_complete:
                    print("❌ Microphone Failed")
                return False
    
    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected to MQTT broker with result code {rc}")
        client.subscribe("stacker/status")
        client.subscribe("stacker/positions")
        
    def on_message(self, client, userdata, msg):
        """Handle incoming MQTT messages"""
        topic = msg.topic
        message = msg.payload.decode()
        
        if topic == "stacker/status":
            print(f"Status update: {message}")
            #self.last_status = message
            self.last_status = message.strip('"')  # Remove surrounding quotes
            self.status_received = True
        elif topic == "stacker/positions":
            try:
                positions = json.loads(message)
                self.current_positions = positions
                self.position_update_received = True
            except json.JSONDecodeError:
                print(f"Error: Invalid JSON in positions: {message}")
    
    def connect(self):
        """Connect to MQTT broker"""
        self.client.connect(self.broker_host, self.broker_port, 60)
        self.client.loop_start()
        time.sleep(1)
    
    def show_capabilities(self):
        """Show available voice input methods"""
        print("🎤 VOICE INPUT CAPABILITIES:")
        print("=" * 40)

        if self.available_voice_files:
            print(f"✅ WAV File Processing: {len(self.available_voice_files)} files available")
        else:
            print("❌ WAV File Processing: No files in voice_cmds/")

        if self.microphone_available:
            print("✅ Live Microphone: Available")
        else:
            print("❌ Live Microphone: Not available in WSL2")

        print()
        
    def process_voice_file(self, filename):
        """Process pre-recorded WAV file"""
        filepath = os.path.join(self.voice_commands_dir, filename)
        
        if not os.path.exists(filepath):
            print(f"❌ Voice file not found: {filepath}")
            return None
            
        print(f"🎤 Processing voice file: {filename}")
        print("🔄 Converting speech to text...")
        
        try:
            with sr.AudioFile(filepath) as source:
                audio = self.recognizer.record(source)
            
            text = self.recognizer.recognize_google(audio)
            print(f"🗣️  Recognized speech: '{text}'")
            return text
            
        except Exception as e:
            print(f"❌ Error processing voice file: {e}")
            return None
    
    def record_live_audio(self, duration=8):
        """Record live audio from microphone"""
        if not self.microphone_available:
            print("❌ Live microphone not available")
            return None

        print(f"🎤 Recording from microphone for {duration} seconds...")
        print("🗣️  Please speak your command!")

        # Wrap the entire audio operation in error suppression
        with suppress_audio_errors():
            try:
                with sr.Microphone() as source:
                    # Adjust for ambient noise
                    print("🎧 Calibrating...")
                    self.recognizer.adjust_for_ambient_noise(source, duration=1)

                    # Record audio for exact duration
                    print("🔴 Recording... Speak now!")
                    # Use record() for precise duration control
                    audio = self.recognizer.record(source, duration=duration)

                    print("🔄 Processing speech...")
                    text = self.recognizer.recognize_google(audio)
                    print(f"✅ Recognized: '{text}'")
                    return text

            except sr.UnknownValueError:
                print("❌ Could not understand audio")
                return None
            except Exception as e:
                print("❌ Recording failed")
                return None
    
    def continuous_recording_mode(self):
        """Continuous listening mode for hands-free operation"""
        if not self.microphone_available:
            print("❌ Continuous mode requires working microphone")
            return
        
        print("🔄 CONTINUOUS LISTENING MODE")
        print("=" * 40)
        print("🎤 Say 'robot' to activate, then give your command")
        print("🛑 Say 'stop listening' to exit this mode")

        # Initial calibration with error suppression
        with suppress_audio_errors():
            with sr.Microphone() as source:
                self.recognizer.adjust_for_ambient_noise(source)

        while True:
            # Wrap each listening cycle in error suppression
            with suppress_audio_errors():
                try:
                    with sr.Microphone() as source:
                        print("👂 Listening for wake word 'robot'...")
                        audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=2)

                    text = self.recognizer.recognize_google(audio).lower()

                    if "robot" in text:
                        print("✅ Wake word detected! Listening for command...")

                        with sr.Microphone() as source:
                            command_audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)

                        command_text = self.recognizer.recognize_google(command_audio)
                        print(f"🎯 Command: '{command_text}'")

                        if "stop listening" in command_text.lower():
                            print("🛑 Exiting continuous mode")
                            break

                        # Process the command
                        command_json = self.llm.process_natural_language_command(command_text, self.current_positions)
                        self.send_command(command_json)
                        self.wait_for_completion()

                except sr.WaitTimeoutError:
                    continue  # Keep listening
                except sr.UnknownValueError:
                    continue  # Ignore unclear audio
                except KeyboardInterrupt:
                    print("🛑 Continuous mode interrupted")
                    break
                except Exception as e:
                    print("❌ Error in continuous mode")
                    continue
    
    def demo_voice_pipeline(self, voice_text):
        """Demonstrate the complete voice-to-robot pipeline"""
        print("*" * 20)
        print("🎓 VOICE-TO-ROBOT PIPELINE DEMO")
        print("*" * 20)

        print(f"1️⃣  VOICE INPUT: '{voice_text}'")
        print(f"2️⃣  CURRENT STATE: {self.current_positions}")

        print("3️⃣  LLM PROCESSING:")
        print("   🧠 Sending to Gemma 3:4B model...")

        command_json = self.llm.process_natural_language_command(voice_text, self.current_positions)
        print(f"   ✅ Generated: {command_json}")

        print(f"4️⃣  ROBOT COMMAND:")
        print(f"   📡 Publishing to MQTT: {json.dumps(command_json)}")

        return command_json
    
    def send_command(self, command_json):
        """Send command to robot"""
        command_str = json.dumps(command_json)
        print(f"📡 Sending: {command_str}")
        self.client.publish("stacker/command", command_str)
    
    def wait_for_completion(self):
        """Wait for robot completion"""
        print("5️⃣  ROBOT EXECUTION:")
        print("   ⏳ Waiting for completion...")
        self.status_received = False

        while not self.status_received or self.last_status != "DONE":
            time.sleep(0.5)

        print("   ✅ Command completed!")

        #self.position_update_received = False
        #while not self.position_update_received:
        #    time.sleep(0.1)

        print(f"6️⃣  FINAL RESULT:")
        print(f"   📍 Updated positions: {self.current_positions}")
        print("   🎉 Pipeline complete!")

        # Show help after each command completion
        self.show_brief_help()
    
    def show_help(self):
        """Show comprehensive help"""
        print("=== HYBRID VOICE-ENABLED ROBOT CONTROL ===")
        print("🎤 VOICE INPUT MODES:")

        if self.available_voice_files:
            print("  📁 WAV File Mode:")
            print("    - Enter file number to process")
            print("    - Guaranteed to work in all environments")

        if self.microphone_available:
            print("  🎙️  Live Microphone Mode:")
            print("    - 'record' - Record single command")
            print("    - 'listen' - Continuous listening mode")
            print("    - 'wake' - Wake word activation mode")

        print("📝 DIRECT INPUT:")
        print("  - Type natural language commands")
        print("  - Example: 'arrange animals by size'")

        print("🔧 COMMANDS:")
        print("  - 'positions' - Show current positions")
        print("  - 'files' - List available WAV files")
        print("  - 'capabilities' - Show voice input capabilities")
        print("  - 'metrics' - Show LLM performance")
        print("  - 'help' - This help")
        print("  - 'quit' - Exit")

    def show_brief_help(self):
        """Show brief help after command completion"""
        print("\n📋 NEXT COMMANDS:")
        if self.available_voice_files:
            print(f"  📁 WAV files: Enter 1-{len(self.available_voice_files)}")
        if self.microphone_available:
            print("  🎙️  Voice: 'record' | 'listen'")
        print("  📝 Text: Type natural language")
        print("  🔧 Other: 'positions' | 'help' | 'quit'")
    
    def run_hybrid_demo(self):
        """Main hybrid demonstration loop"""
        print("=" * 60)
        print("🎤 HYBRID VOICE-ENABLED ROBOT CONTROL")
        print("📍 Indo-Pacific Robotics Conference (IPRAAC)")
        print("=" * 60)
        
        self.show_capabilities()
        self.show_help()
        
        if self.available_voice_files:
            print("🎤 Available WAV Files:")
            for i, file in enumerate(self.available_voice_files, 1):
                print(f"  {i}. {file}")

        print(f"🔗 Connected to MQTT: {self.broker_host}:{self.broker_port}")
        print("🤖 Robot ready for voice commands!")

        # Mark initial setup as complete to suppress future microphone test logs
        self.initial_setup_complete = True

        while True:
            try:
                print("─" * 50)
                user_input = input("🎯 Enter command: ").strip()
                
                if user_input.lower() in ['quit', 'exit', 'q']:
                    break
                elif user_input.lower() == 'positions':
                    print(f"Current positions: {self.current_positions}")
                    continue
                elif user_input.lower() == 'files':
                    if self.available_voice_files:
                        print("Available WAV files:")
                        for i, file in enumerate(self.available_voice_files, 1):
                            print(f"  {i}. {file}")
                    else:
                        print("No WAV files available")
                    continue
                elif user_input.lower() == 'capabilities':
                    self.show_capabilities()
                    continue
                elif user_input.lower() == 'help':
                    self.show_help()
                    continue
                elif user_input.lower() == 'metrics':
                    metrics = self.llm.get_academic_metrics()
                    print("=== LLM PERFORMANCE METRICS ===")
                    print(json.dumps(metrics, indent=2))
                    continue
                elif user_input.lower() == 'record' and self.microphone_available:
                    voice_text = self.record_live_audio()
                    if voice_text:
                        command_json = self.demo_voice_pipeline(voice_text)
                    else:
                        continue
                elif user_input.lower() == 'listen' and self.microphone_available:
                    self.continuous_recording_mode()
                    continue
                elif user_input.isdigit():
                    # WAV file processing
                    file_index = int(user_input) - 1
                    if 0 <= file_index < len(self.available_voice_files):
                        filename = self.available_voice_files[file_index]
                        voice_text = self.process_voice_file(filename)
                        if voice_text:
                            command_json = self.demo_voice_pipeline(voice_text)
                        else:
                            continue
                    else:
                        print(f"❌ Invalid file number. Choose 1-{len(self.available_voice_files)}")
                        continue
                else:
                    # Direct text input
                    print("🔤 Processing text input...")
                    command_json = self.llm.process_natural_language_command(user_input, self.current_positions)
                
                # Execute command
                self.send_command(command_json)
                self.wait_for_completion()
                
            except KeyboardInterrupt:
                print("🛑 Shutting down...")
                break
            except Exception as e:
                print(f"❌ Error: {e}")
        
        # Final metrics
        print("📊 Final Performance Report:")
        print(json.dumps(self.llm.get_academic_metrics(), indent=2))
        
        self.client.loop_stop()
        self.client.disconnect()
        print("👋 Session ended.")

if __name__ == "__main__":
    print("🎤 Initializing Hybrid Voice Robot Control...")
    
    # Ensure voice commands directory exists
    if not os.path.exists("voice_cmds"):
        print("⚠️  Creating voice_cmds directory")
        os.makedirs("voice_cmds")
    
    ucs = HybridVoiceUCS()
    ucs.connect()
    ucs.run_hybrid_demo()
