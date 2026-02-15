# Braccio Plan-Seq-Learn Verification

## Project Overview
This project verifies the **Planning** phase of the **Plan-Seq-Learn (PSL)** architecture using a Braccio robot in NVIDIA Isaac Sim. The system uses a Large Language Model (LLM) to decompose high-level natural language commands (e.g., "stack the cubes") into atomic robot actions (`grab`, `place`, `wait`), which are then executed by a finite state machine.

- **Platform**: Isaac Sim 5.1.0 on Ubuntu 24.04
- **Robot**: Braccio (5-DOF robotic arm)

## Capabilities
- **Natural Language Control**: Control the robot using English commands via CLI.
- **Task Decomposition**: LLM (running locally via Ollama) breaks down complex goals into sequential steps.
- **Automatic Execution**: A finite state machine executes the plan using 2D Inverse Kinematics.

## Project Structure

```
braccio/
├── main.py               # Entry point — launches Isaac Sim and runs the simulation loop
├── kinematics.py          # Forward/inverse kinematics solver and robot dimension constants
├── grab_controller.py     # Finite state machine for grab, place, and home sequences
├── scene_cfg.py           # Isaac Sim scene definition (room, robot, cubes) and physics helpers
├── cli_controller.py      # Threaded CLI for natural language user input
├── llm_planner.py         # LLM-based task planner (Ollama client, scene descriptor, task queue)
├── config/
│   └── braccio_cfg.py     # URDF-based Braccio articulation config (actuators, joint limits)
├── prompts/
│   └── task_decomposition.txt  # System prompt template for LLM task decomposition
├── __init__.py            # Gym environment registration (for RL experiments)
├── .env                   # Environment variables (API keys, model config)
├── logs/                  # Runtime log output
└── archieves/             # Archived experiment scripts
```

## Environment Setup
- **OS**: Ubuntu 24.04 LTS
- **Simulation**: NVIDIA Isaac Sim 5.1.0
- **External Dependencies**:
  - `ollama` (for local LLM inference)
  - `mistral` model (run `ollama pull mistral`)

## Verification Context & Simplifications
**Note:** Due to time constraints, this project specifically targets the verification of the **Planning** logic. Several physical and perceptual aspects are simplified ("cheated") to isolate the planning variable:

1.  **Perception ("God Oracle")**:
    - **No Computer Vision**: There is no camera processing, YOLO, or neural network perception implemented.
    - **Cheating**: Object coordinates are fetched directly from the Isaac Sim simulation engine (`scene.data.root_pos_w`). The robot has perfect, instantaneous knowledge of where everything is.

2.  **Robot Control**:
    - **Perfect Servos**: Actuators are assumed to have perfectly sufficient torque and zero backlash.
    - **Friction Hacking**: High-friction physics materials are applied to the gripper to prevent objects from slipping, simulating a "perfect" grasp without complex contact dynamics.
    - **Simplified Physics**: Base joint limits are unlocked/modified to allow easier rotation logic.

3.  **Objects**:
    - Objects are simplified to standard 4cm cubes.
    - No complex real-world objects (cups, pills, tools) are currently supported.

## Limitations

### Sim2Real Limitations
Transferring this system to a real Braccio robot involves significant challenges not present in this simulation:
- **Perception Gap**: The real world does not provide a "God Oracle". You must implement a robust Computer Vision pipeline to detect objects.
- **Physics Fidelity**: Real servos suffer from backlash, torque limits, and cable drag. Real grippers might crush fragile objects or fail to grip smooth ones. The simulation uses infinite friction constraints that do not exist in reality.
- **Safety**: The simulation moves at maximum speed. A real robot requires velocity ramping and collision avoidance updates to prevent hardware damage.

### Architecture Limitations
- **Pre-Defined Atomic Actions**: The robot is strictly limited to pre-defined primitives: `grab`, `place`, and `wait`. It cannot perform continuous manipulation tasks like pouring, unscrewing, or sliding.
- **Coordinate Identification (Depth/Stereo)**: 
  - In this simulation, "Place on red cube" is solved by querying the engine: `GetPosition("RedCube") + offset`.
  - In a real-world architecture using depth or stereo cameras, identifying the target coordinate requires a complex pipeline:
    1.  **Detection**: Identifying the pixel bounding box of the target object (e.g., via YOLO).
    2.  **Depth Estimation**: correlating those pixels to depth values from the stereo/depth map.
    3.  **Surface Normal Estimation**: Calculating the flat surface suitable for placement.
    4.  **Coordinate Transformation**: Converting the camera-relative (X,Y,Z) coordinate into the robot's base coordinate system using Hand-Eye Calibration matrices.
  - The current architecture assumes this entire pipeline is instantaneous and error-free.

## Contribution: How Verified Planning Helps the Real World
Despite the physical simplifications, this project provides a valid verification of the **Semantic Reasoning Layer** of the PSL architecture.

1.  **Task Logic Verification**: It proves that an LLM can correctly understand spatial relationships ("on top of", "next to") and sequence dependencies (e.g., verifying that the robot must `grab A` before it can `place A`).
2.  **Scalability**: The planning module is modular. In a real-world deployment, the "God Oracle" perception module can be replaced with a real "Computer Vision" module, and the "Sim Control" with "Real Control," without changing the high-level planning logic verified here.