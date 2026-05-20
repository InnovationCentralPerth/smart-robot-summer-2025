# S2-2026: Intelligent Voice-Controlled Animal Stacker Robot System

## 📋 Project Overview

An AI-powered robotic control system that enables natural language and voice control of a tabletop pick-and-place manipulator robot (Arduino Braccio). This project bridges human conversational commands to precise robotic manipulation through distributed microservices architecture and modern generative AI technologies.

**Target Application:** Indo-Pacific Robotics Conference (IPRAAC) demonstration

---

## 🎯 Key Features

- **Natural Language Processing**: Understand conversational commands like "arrange the animals from largest to smallest"
- **Voice Recognition**: Live microphone input or pre-recorded WAV file processing
- **Local LLM Integration**: Gemma 3:4B model via Ollama for low-latency AI
- **MQTT-Based Communication**: Distributed microservices architecture
- **Real-time Position Tracking**: Dynamic state management for all game objects
- **Cross-Platform Support**: WSL2 development, Linux/Raspberry Pi production

---

## 🏗️ System Architecture

```
┌─────────────────────┐    MQTT     ┌─────────────────────┐    MQTT     ┌─────────────────────┐
│  User Command       │◄───────────►│    MQTT Broker      │◄───────────►│  Stacker Controller │
│  Station (UCS)      │             │   (Message Hub)     │             │      (SC)           │
└─────────────────────┘             └─────────────────────┘             └─────────────────────┘
        ▼                                   ▼                                    ▼
• Voice Recognition              Topics:                      • Robot Control
• LLM Processing           • stacker/command                 • Movement Simulation
• Natural Language         • stacker/status                  • Position Updates
• Command Analysis         • stacker/positions              • Status Reporting
```

### Components

#### 1. **User Command Station (UCS)** - `smart_stacker_frontend/ucs_voice_*.py`
Primary interface and intelligence hub for voice-controlled robot operation.

**Features:**
- Hybrid voice input (live microphone + WAV file fallback)
- Speech-to-text via Google Speech Recognition
- LLM-powered natural language interpretation
- Command generation and validation
- Interactive CLI with real-time feedback

**Files:**
- `ucs_voice_wifi.py` - WiFi-enabled voice processing
- `ucs_voice_safe.py` - Safe mode with fallback support

#### 2. **LLM Integration Module** - `smart_stacker_frontend/llm_integration.py`
Artificial intelligence engine for command interpretation.

**Capabilities:**
- Conversational natural language understanding
- Spatial relationship interpretation (left/right, front/back, size-based)
- JSON schema enforcement for safety
- Performance metrics collection
- Ollama server integration (Gemma 3:4B model)

#### 3. **Stacker Controller** - `MQTT8/`
Robot hardware interface and movement simulator.

**Responsibilities:**
- Subscribes to `stacker/command` topic
- Simulates 3-5 second movement delays
- Tracks current positions in real-time
- Publishes status updates ("BUSY"/"DONE")
- Hardware abstraction for Arduino Braccio

**Key Files:**
- `main.py` - Main controller loop
- `gui.py` - Graphical interface
- `command_processor.py` - Command parsing and validation
- `serial_comm.py` - Arduino communication

#### 4. **Arduino Firmware** - `Braccio_11_wloop/Braccio_11_wloop.ino`
Braccio robotic arm control code with serial communication interface.

#### 5. **Development Archives** - `animal_all/`
Version history and experimental variations (v1-v5, animal_game.py, sounds implementations)

---

## 🎮 Game Mechanics

### Animals (Robotic Objects)
| Animal | Symbol | Size | Speed | Properties |
|--------|--------|------|-------|-----------|
| **Elephant** | E | Largest | Slowest | Heavy, requires most power |
| **Lion** | L | Medium | Fastest | Balanced |
| **Frog** | F | Smallest | Slowest | Light, quick placement |

### Available Positions
```
    L1  ROBOT  R1
    L2    C    R2
```

- **L1, L2**: Left row (back to front)
- **R1, R2**: Right row (back to front)
- **C**: Center position (front of robot)

### Command Examples
```
User: "line up animals from big to small from the left"
→ {"E": "L1", "L": "L2", "F": "C"}

User: "put elephant in back right"
→ {"E": "R1", "L": "L2", "F": "C"}
```

---

## 🔧 Technology Stack

| Component | Technology |
|-----------|-----------|
| **Language** | Python 3.12 |
| **Voice Recognition** | SpeechRecognition (Google Speech API) |
| **LLM** | Ollama + Gemma 3:4B |
| **Message Broker** | MQTT (paho-mqtt) |
| **Audio Processing** | PyAudio (cross-platform microphone) |
| **Hardware Target** | Arduino Braccio |
| **HTTP Requests** | requests (for LLM API) |

---

## 📦 Installation & Setup

### Prerequisites
- Python 3.12+
- Ollama with Gemma 3:4B model installed
- MQTT broker (mosquitto or cloud-based)
- Arduino IDE (for Braccio firmware)

### Dependencies
Install required packages:
```bash
pip install -r requirements.txt
```

**requirements.txt contents:**
```
paho-mqtt>=2.0.0
requests>=2.31.0
SpeechRecognition>=3.10.0
PyAudio>=0.2.13
```

### Environment Configuration
Create or configure `.env` file:

```env
# Logging
LOG_FILE=llm_robot_commands.log

# LLM Configuration
MODEL=gemma3:4b
HOST_URL=http://localhost:11434
PROMPT_FILE=llm_system_prompt.txt

# Valid Objects & Positions
VALID_ANIMALS={"E", "L", "F"}
VALID_POSITIONS={"L1", "L2", "C", "R1", "R2"}
```

---

## 🚀 Running the System

### 1. Start MQTT Broker
```bash
mosquitto
```

### 2. Start Stacker Controller
```bash
cd MQTT8
python main.py
```

### 3. Start User Command Station
```bash
cd smart_stacker_frontend
python ucs_voice_wifi.py   # or ucs_voice_safe.py
```

### 4. Upload Arduino Firmware (Optional)
```bash
# Use Arduino IDE to upload Braccio_11_wloop.ino to Arduino board
```

---

## 📡 MQTT Message Format

### Command Topic: `stacker/command`
```json
{
  "E": "L1",
  "L": "C",
  "F": "R2"
}
```

### Status Topic: `stacker/status`
```
"BUSY"    // Robot is moving
"DONE"    // Movement complete
```

### Positions Topic: `stacker/positions`
```json
{
  "E": "L2",
  "L": "C",
  "F": "R2"
}
```

---

## 📚 Project Structure

```
s2-2026/
├── README.md                           # This file
├── requirements.txt                    # Python dependencies
├── .env                                # Environment configuration
│
├── smart_stacker_frontend/             # UCS - Voice Interface & LLM
│   ├── ucs_voice_wifi.py              # WiFi-enabled voice control
│   ├── ucs_voice_safe.py              # Safe mode with fallbacks
│   ├── llm_integration.py             # LLM processing engine
│   ├── llm_system_prompt.txt          # AI system prompt
│   ├── voice_stacker_readme.txt       # Detailed documentation
│   ├── main.py                         # Entry point (currently empty)
│   ├── readme.md                       # Frontend documentation
│   └── validators/                     # Command validation modules
│
├── MQTT8/                              # SC - Stacker Controller
│   ├── main.py                         # Main controller loop
│   ├── gui.py                          # GUI interface
│   ├── command_processor.py            # Command parsing
│   ├── llm_pub.py                      # LLM publisher
│   ├── llm_sub.py                      # LLM subscriber
│   ├── llm_sender.py                   # LLM message sender
│   ├── mqtt_gui.py                     # MQTT GUI interface
│   ├── mqtt_sub.py                     # MQTT subscriber
│   ├── serial_comm.py                  # Serial communication (Arduino)
│   ├── run.py                          # Alternative runner
│   └── __pycache__/                    # Python cache
│
├── Braccio_11_wloop/                   # Arduino Firmware
│   └── Braccio_11_wloop.ino           # Robotic arm control sketch
│
├── animal_all/                         # Development Archives & Variations
│   ├── animal_game.py                  # Game logic
│   ├── animal_ard.py                   # Arduino integration variant
│   ├── animal_v1.py through v5.py     # Version iterations
│   ├── arduino_v1.py, v2.py           # Arduino communication variants
│   ├── llm_v1.py through v3.py        # LLM integration versions
│   ├── voice_v1.py, v2.py             # Voice processing versions
│   ├── sounds_animal.py                # Animal sound effects
│   ├── sounds2.py                      # Sound processing (v2)
│   ├── mq_post.py                      # MQTT publish utility
│   ├── mq_rec.py                       # MQTT receive utility
│   └── run.py                          # Runner script
│
├── ICP Robotics.pdf                   # Conference presentation materials
└── ICP_Report_Robot.docx.pdf          # Technical report
```

---

## 🧠 Intelligent Features

### Adaptive Voice Input
- Automatic microphone detection and calibration
- Ambient noise adjustment for WSL2/Linux
- Fallback to WAV file processing
- Wake word activation for hands-free operation

### Conversational AI
- Natural language command interpretation
- Context-aware position tracking
- Semantic understanding of animal properties
- Intelligent command validation

### Safety Features
- Collision detection (prevents multiple animals at same position)
- Position validation
- Command structure verification
- Error handling and user guidance

### Academic Metrics
- Command processing time measurement
- LLM response validation
- Success/failure rate tracking
- Performance analytics

---

## 📖 Usage Examples

### Voice Command: Natural Language
```
User: "Arrange animals from biggest to smallest, left to right"
System: Interprets, generates → {"E": "L1", "L": "L2", "F": "C"}
Result: Robot executes placement sequence
```

### Voice Command: Spatial Instructions
```
User: "Put the elephant in the back left corner"
System: → {"E": "L1", "L": "L2", "F": "C"}
```

### Text Command: Direct Input
```
User: "make an L shape with the animals"
System: → {"E": "L1", "L": "R1", "F": "C"}
```

---

## 🎓 Educational & Research Value

This system demonstrates:

1. **Human-Robot Interaction (HRI)**
   - Natural language interfaces for robotic control
   - Voice-driven automation
   - Accessibility improvements

2. **Distributed Systems**
   - Microservices architecture
   - MQTT-based inter-process communication
   - Scalable, modular design

3. **Generative AI Integration**
   - Local LLM deployment
   - Prompt engineering for robotics
   - Context-aware command interpretation

4. **Cross-Platform Development**
   - WSL2 development workflow
   - Linux production deployment
   - Hardware abstraction

---

## 🔮 Future Enhancements

### Planned Features
- [ ] Hardware integration with real Braccio arm
- [ ] Computer vision for position verification
- [ ] Multi-robot coordination
- [ ] Advanced NLU (spaCy, transformers)
- [ ] Mobile app interface
- [ ] Cloud-based LLM options

### Academic Extensions
- [ ] Reinforcement learning for path optimization
- [ ] Predictive command completion
- [ ] Multi-modal input (voice + gesture)
- [ ] Formal verification methods
- [ ] Performance benchmarking

---

## 📝 Documentation

- **Detailed System Overview**: See `smart_stacker_frontend/voice_stacker_readme.txt`
- **LLM Prompt Engineering**: See `smart_stacker_frontend/llm_system_prompt.txt`
- **Conference Materials**: See `ICP Robotics.pdf` and `ICP_Report_Robot.docx.pdf`

---

## 🤝 Contributing

This is a research and demonstration project created for:
- Innovation Central Perth (ICP)
- Summer 2025 Robotics Program
- Indo-Pacific Robotics Conference (IPRAAC)

---

## 📄 License

Please check the parent repository for license information.

---

## 🔗 References

- **Conference**: Indo-Pacific Robotics Conference (IPRAAC)
- **Organization**: Innovation Central Perth (ICP)
- **Platform**: Arduino Braccio Robotic Arm
- **AI Model**: Gemma 3:4B via Ollama
- **Communication**: MQTT Protocol

---

## 📞 Support & Questions

For questions about this project, refer to:
1. Detailed documentation in `voice_stacker_readme.txt`
2. Code comments and inline documentation
3. Conference presentation materials (PDFs)
4. Issue tracker in the main repository

---

**System designed for educational and research purposes**  
**Conference Demonstration: IPRAAC 2025**  
**Last Updated: May 2026**
