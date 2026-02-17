"""
LLM Planner Module for Plan-Seq-Learn Architecture.

This module provides:
- OllamaClient: Communicate with local Ollama LLM
- SceneDescriptor: Generate text descriptions from Isaac Sim scene
- TaskPlanner: Decompose user commands into atomic tasks
- TaskQueue: FIFO queue for sequential task execution
- CoordinateResolver: Convert LLM offsets to world coordinates
"""

import json
import math
import asyncio
import httpx
from dataclasses import dataclass, field
from typing import Literal, Optional
from collections import deque
from pathlib import Path


# ============================================================================
#  DATA STRUCTURES
# ============================================================================

@dataclass
class Task:
    """A single atomic robot task with LLM-generated coordinates."""
    action: Literal["grab", "place", "move_to", "wait"]
    target: str  # Object name: "red_cube", "green_cube"
    
    # For "place" action: LLM specifies offset from reference object
    reference: Optional[str] = None  # Reference object (e.g., "green_cube", "table")
    offset: Optional[tuple[float, float, float]] = None  # (dx, dy, dz) in meters
    
    # Final world coordinates (resolved at execution)
    coordinates: Optional[tuple[float, float, float]] = None
    
    def __repr__(self):
        if self.action == "grab":
            return f"Task(grab {self.target})"
        elif self.action == "place":
            return f"Task(place {self.target} at {self.reference} + {self.offset})"
        else:
            return f"Task({self.action} {self.target})"


@dataclass
class TaskQueue:
    """Queue of tasks to execute sequentially."""
    tasks: deque[Task] = field(default_factory=deque)
    current_index: int = 0
    status: Literal["pending", "executing", "completed", "failed"] = "pending"
    
    def add(self, task: Task):
        self.tasks.append(task)
    
    def next(self) -> Optional[Task]:
        """Get the next task to execute."""
        if self.current_index < len(self.tasks):
            task = self.tasks[self.current_index]
            self.current_index += 1
            return task
        return None
    
    def peek(self) -> Optional[Task]:
        """Look at the next task without advancing."""
        if self.current_index < len(self.tasks):
            return self.tasks[self.current_index]
        return None
    
    def is_empty(self) -> bool:
        return self.current_index >= len(self.tasks)
    
    def reset(self):
        self.current_index = 0
        self.status = "pending"
    
    def __len__(self):
        return len(self.tasks)


@dataclass
class ObjectState:
    """State of an object in the scene."""
    name: str
    position: tuple[float, float, float]
    size: tuple[float, float, float]  # Dimensions (width, depth, height)
    color: str
    prim_key: str  # Key in Isaac Sim scene (e.g., "cube", "red_cube")


# ============================================================================
#  OBJECT REGISTRY
# ============================================================================

# Known objects with their properties
# prim_key must match the scene dictionary keys in main.py
OBJECT_REGISTRY = {
    "green_cube": {
        "size": (0.04, 0.04, 0.04),
        "color": "green",
        "prim_key": "cube",  # Key in scene dict (OfficeSceneCfg.cube)
    },
    "red_cube": {
        "size": (0.04, 0.04, 0.04),
        "color": "red",
        "prim_key": "red_cube",  # Key in scene dict (OfficeSceneCfg.red_cube)
    },
}


# ============================================================================
#  SCENE DESCRIPTOR
# ============================================================================

class SceneDescriptor:
    """Generates text descriptions from Isaac Sim scene state."""
    
    def __init__(self, scene, registry: dict = None):
        self.scene = scene
        self.registry = registry or OBJECT_REGISTRY
    
    def _get_all_positions(self) -> dict:
        """Get positions of all known objects."""
        positions = {}
        for obj_name, obj_info in self.registry.items():
            prim_key = obj_info.get("prim_key")
            if prim_key and prim_key in self.scene.keys():
                try:
                    pos = self.scene[prim_key].data.root_pos_w[0]
                    positions[obj_name] = {
                        "x": float(pos[0]),
                        "y": float(pos[1]),
                        "z": float(pos[2]),
                        "size": obj_info["size"],
                        "color": obj_info["color"]
                    }
                except (KeyError, IndexError):
                    pass
        return positions
    
    def _detect_spatial_relationships(self, positions: dict) -> list:
        """
        Detect spatial relationships between objects.
        Returns a list of relationship strings.
        """
        relationships = []
        obj_names = list(positions.keys())
        
        for i, name_a in enumerate(obj_names):
            pos_a = positions[name_a]
            size_a = pos_a["size"]
            
            for name_b in obj_names[i+1:]:
                pos_b = positions[name_b]
                size_b = pos_b["size"]
                
                # Check horizontal proximity (XY distance)
                dx = abs(pos_a["x"] - pos_b["x"])
                dy = abs(pos_a["y"] - pos_b["y"])
                xy_distance = (dx**2 + dy**2)**0.5
                
                # Objects are "aligned" if XY distance is less than half the cube size
                xy_threshold = max(size_a[0], size_b[0]) * 0.75
                
                if xy_distance < xy_threshold:
                    # Objects are vertically aligned - check stacking
                    z_diff = pos_a["z"] - pos_b["z"]
                    stack_threshold = max(size_a[2], size_b[2]) * 0.8
                    
                    if z_diff > stack_threshold:
                        relationships.append(f"{name_a} is ON TOP OF {name_b}")
                    elif z_diff < -stack_threshold:
                        relationships.append(f"{name_b} is ON TOP OF {name_a}")
        
        return relationships
    
    def describe(self) -> str:
        """Generate a text description of the current scene state with spatial relationships."""
        positions = self._get_all_positions()
        
        if not positions:
            return "Scene is empty or no known objects detected."
        
        # Object descriptions
        descriptions = []
        for obj_name, pos_info in positions.items():
            descriptions.append(
                f"- {obj_name}: position=({pos_info['x']:.3f}, {pos_info['y']:.3f}, {pos_info['z']:.3f}), "
                f"size={pos_info['size'][0]}x{pos_info['size'][1]}x{pos_info['size'][2]}m, color={pos_info['color']}"
            )
        
        result = "Objects in scene:\n" + "\n".join(descriptions)
        
        # Add spatial relationships
        relationships = self._detect_spatial_relationships(positions)
        if relationships:
            result += "\n\nSpatial relationships:\n" + "\n".join(f"- {r}" for r in relationships)
        
        return result
    
    def get_object_position(self, obj_name: str) -> Optional[tuple[float, float, float]]:
        """Get the current world position of an object."""
        obj_info = self.registry.get(obj_name)
        if not obj_info:
            print(f"[SceneDescriptor] Unknown object: {obj_name}")
            return None
        
        prim_key = obj_info.get("prim_key")
        if prim_key and prim_key in self.scene.keys():
            try:
                pos = self.scene[prim_key].data.root_pos_w[0]
                return (float(pos[0]), float(pos[1]), float(pos[2]))
            except (KeyError, IndexError) as e:
                print(f"[SceneDescriptor] Error getting position for {obj_name}: {e}")
        
        return None


# ============================================================================
#  OLLAMA CLIENT
# ============================================================================

class OllamaClient:
    """Async client to communicate with local Ollama API."""
    
    def __init__(self, base_url: str = "http://localhost:11434", model: str = "mistral"):
        self.base_url = base_url
        self.model = model
    
    async def generate(self, prompt: str, timeout: float = 600.0) -> str:
        """Send a prompt to Ollama and get the response."""
        url = f"{self.base_url}/api/generate"
        payload = {
            "model": self.model,
            "prompt": prompt,
            "stream": False,
            "options": {
                "temperature": 0.1,  # Low temperature for more deterministic output
                "num_predict": 2048,  # Ensure full response for complex task plans
            }
        }
        
        async with httpx.AsyncClient(timeout=timeout) as client:
            try:
                response = await client.post(url, json=payload)
                response.raise_for_status()
                result = response.json()
                # DeepSeek-R1 uses 'thinking' field for chain-of-thought, 'response' for final answer
                # We need to check both since JSON might be in either
                text_response = result.get("response", "")
                thinking = result.get("thinking", "")
                
                # Combine both - JSON could be in thinking or response
                combined = f"{thinking}\n{text_response}"
                
                if not combined.strip():
                    print(f"[OllamaClient] Warning: Empty response from model")
                
                return combined
            except httpx.TimeoutException:
                print("[OllamaClient] Request timed out")
                return ""
            except httpx.HTTPStatusError as e:
                print(f"[OllamaClient] HTTP error: {e}")
                return ""
            except Exception as e:
                print(f"[OllamaClient] Error: {e}")
                return ""
    
    def generate_sync(self, prompt: str, timeout: float = 60.0) -> str:
        """Synchronous wrapper for generate()."""
        return asyncio.run(self.generate(prompt, timeout))


# ============================================================================
#  TASK PLANNER
# ============================================================================

class TaskPlanner:
    """Prompts LLM and parses structured task output."""
    
    PROMPT_TEMPLATE = """You are a robotic task planner for a Braccio arm in Isaac Sim.

PHYSICAL RULES:
1. PRECONDITION: To pick up an object, its top must be CLEAR (nothing on top of it).
2. PRECONDITION: The robot hand must be EMPTY to pick up an object.
3. EFFECT: After placing an object somewhere, the robot hand becomes EMPTY.
4. EFFECT: When you move an object off another, the object below becomes CLEAR.

CURRENT SCENE:
{scene_description}

USER COMMAND: {user_command}

REASONING STEPS (think step by step):
1. What is the goal? Which object needs to end up where?
2. Check: Is the object I need to grab CLEAR? If not, first move the blocking object aside.
3. Execute: grab the object, then place it at the destination.

OUTPUT FORMAT (JSON array only):
- grab: {{"action": "grab", "target": "OBJECT_TO_PICK_UP"}}
- place: {{"action": "place", "target": "OBJECT_YOU_ARE_HOLDING", "reference": "DESTINATION_OBJECT", "offset": [dx, dy, dz]}}

IMPORTANT:
- "target" = the object currently in the robot's gripper (what you just grabbed)
- "reference" = where to place it (another object name, or "origin" for absolute position)
- "origin" means world origin (0,0,0) - use offset like [0.1, -0.2, 0.02] with it

OFFSET VALUES (meters):
- on top: offset=[0, 0, 0.05]
- "aside" / "out of the way" to origin: offset=[0.1, -0.2, 0.02]
- "next to" / "beside" (DEFAULT to RIGHT): offset=[0.05, 0, 0]
- "left of" / "-x": offset=[-0.05, 0, 0]
- "right of" / "x": offset=[0.05, 0, 0]
- "in front of" / "-y": offset=[0, -0.05, 0]
- "behind" / "y": offset=[0, 0.05, 0]

EXAMPLE - "put green_cube next to red_cube":
[
  {{"action": "grab", "target": "green_cube"}},
  {{"action": "place", "target": "green_cube", "reference": "red_cube", "offset": [0.05, 0, 0]}}
]

EXAMPLE - "put green_cube on top of red_cube" when red_cube is ON TOP OF green_cube:
[
  {{"action": "grab", "target": "red_cube"}},
  {{"action": "place", "target": "red_cube", "reference": "origin", "offset": [0, -0.25, 0.02]}},
  {{"action": "grab", "target": "green_cube"}},
  {{"action": "place", "target": "green_cube", "reference": "red_cube", "offset": [0, 0, 0.05]}}
]

Output ONLY the JSON array, no explanation:
TASKS:"""
    
    def __init__(self, client: OllamaClient = None):
        self.client = client or OllamaClient()
    
    async def plan(self, scene_description: str, user_command: str) -> TaskQueue:
        """Generate a task queue from scene description and user command."""
        prompt = self.PROMPT_TEMPLATE.format(
            scene_description=scene_description,
            user_command=user_command
        )
        
        print(f"\n[TaskPlanner] Sending to LLM...")
        response = await self.client.generate(prompt)
        print(f"[TaskPlanner] LLM response:\n{response}")
        
        return self._parse_response(response)
    
    def plan_sync(self, scene_description: str, user_command: str) -> TaskQueue:
        """Synchronous wrapper for plan()."""
        return asyncio.run(self.plan(scene_description, user_command))
    
    def _parse_response(self, response: str) -> TaskQueue:
        """Parse LLM response into TaskQueue, handling mixed text/JSON."""
        queue = TaskQueue()
        tasks_data = []
        
        # Robust parser: find all top-level [...] JSON arrays
        brace_count = 0
        current_start = -1
        
        for i, char in enumerate(response):
            if char == '[':
                if brace_count == 0:
                    current_start = i
                brace_count += 1
            elif char == ']':
                brace_count -= 1
                if brace_count == 0 and current_start != -1:
                    # Found a complete bracket block
                    block = response[current_start : i + 1]
                    # Remove comments (// ... and /* ... */) before parsing
                    import re
                    block_clean = re.sub(r'//.*?(?=\n|$)', '', block)  # Remove // comments
                    block_clean = re.sub(r'/\*.*?\*/', '', block_clean, flags=re.DOTALL)  # Remove /* */ comments
                    try:
                        data = json.loads(block_clean)
                        if isinstance(data, list):
                            # Filter: Only accept dicts (valid tasks), ignore lists of numbers (e.g. offsets)
                            valid_tasks = [item for item in data if isinstance(item, dict)]
                            if valid_tasks:
                                tasks_data.extend(valid_tasks)
                                print(f"[TaskPlanner] Found task block: {len(valid_tasks)} items")
                    except json.JSONDecodeError:
                        pass # Not valid JSON, skip
                    current_start = -1
        
        # If simple parsing failed, fallback to original logic (just in case)
        if not tasks_data:
            start_idx = response.find('[')
            end_idx = response.rfind(']')
            if start_idx != -1 and end_idx != -1:
                try:
                    data = json.loads(response[start_idx:end_idx+1])
                    if isinstance(data, list):
                        valid_tasks = [item for item in data if isinstance(item, dict)]
                        tasks_data = valid_tasks
                except:
                    pass

        if not tasks_data:
            print("[TaskPlanner] No valid JSON tasks found in response")
            return queue

        # Convert dictionaries to Task objects
        for task_dict in tasks_data:
            action = task_dict.get("action")
            target = task_dict.get("target")
            
            # Map unexpected actions
            if action == "release":
                print(f"[TaskPlanner] Mapping 'release' to 'wait' (place handles release)")
                continue # Skip release, as place already releases
            
            if not action or not target:
                print(f"[TaskPlanner] Skipping invalid task: {task_dict}")
                continue
            
            task = Task(
                action=action,
                target=target,
                reference=task_dict.get("reference"),
                offset=tuple(task_dict.get("offset", [])) if task_dict.get("offset") else None
            )
            
            queue.add(task)
            print(f"[TaskPlanner] Added task: {task}")
        
        return queue


# ============================================================================
#  COORDINATE RESOLVER
# ============================================================================

class CoordinateResolver:
    """Converts LLM offsets to world coordinates."""
    
    def __init__(self, scene_descriptor: SceneDescriptor):
        self.scene_descriptor = scene_descriptor
    
    def resolve(self, task: Task) -> Task:
        """
        Resolve world coordinates for a task.
        
        For grab: use object's current position
        For place: reference position + LLM-provided offset
        """
        if task.action == "grab":
            pos = self.scene_descriptor.get_object_position(task.target)
            if pos:
                task.coordinates = pos
                print(f"[CoordinateResolver] grab {task.target} at {pos}")
            else:
                print(f"[CoordinateResolver] Could not find position for {task.target}")
        
        elif task.action == "place":
            if not task.reference or not task.offset:
                print(f"[CoordinateResolver] Place task missing reference or offset: {task}")
                return task
            
            if task.reference == "origin" or task.reference == "robot_base":
                ref_pos = (0.0, 0.0, 0.0)
            else:
                ref_pos = self.scene_descriptor.get_object_position(task.reference)
            
            if ref_pos:
                task.coordinates = (
                    ref_pos[0] + task.offset[0],
                    ref_pos[1] + task.offset[1],
                    ref_pos[2] + task.offset[2]
                )
                print(f"[CoordinateResolver] place {task.target} at {ref_pos} + {task.offset} = {task.coordinates}")
            else:
                print(f"[CoordinateResolver] Could not find reference position for {task.reference}")
        
        return task
    
    def validate_reachable(self, coords: tuple, robot_reach: float = 0.30) -> bool:
        """Check if target is within Braccio arm reach."""
        if not coords:
            return False
        distance_2d = math.sqrt(coords[0]**2 + coords[1]**2)
        return 0.08 < distance_2d < robot_reach and 0.01 < coords[2] < 0.25


# ============================================================================
#  LLM CONTROLLER (High-level orchestrator)
# ============================================================================

class LLMController:
    """
    High-level controller that orchestrates the Plan-Seq-Learn pipeline.
    
    Usage:
        controller = LLMController(scene)
        controller.process_command("put the green cube on the red cube")
        
        # In main loop:
        task = controller.get_next_task()
        if task:
            # Execute task with GrabController
    """
    
    def __init__(self, scene, model: str = "mistral"):
        self.scene = scene
        self.scene_descriptor = SceneDescriptor(scene)
        self.client = OllamaClient(model=model)
        self.planner = TaskPlanner(self.client)
        self.resolver = CoordinateResolver(self.scene_descriptor)
        
        self.task_queue: Optional[TaskQueue] = None
        self.current_task: Optional[Task] = None
    
    def process_command(self, user_command: str) -> bool:
        """
        Process a natural language command.
        
        Returns True if tasks were generated successfully.
        """
        print(f"\n{'='*60}")
        print(f"  Processing: {user_command}")
        print(f"{'='*60}")
        
        # Step 1: Describe current scene
        scene_description = self.scene_descriptor.describe()
        print(f"\n[Scene]\n{scene_description}")
        
        # Step 2: Plan tasks via LLM
        self.task_queue = self.planner.plan_sync(scene_description, user_command)
        
        if len(self.task_queue) == 0:
            print("[LLMController] No tasks generated")
            return False
        
        print(f"\n[LLMController] Generated {len(self.task_queue)} tasks")
        self.task_queue.status = "executing"
        return True
    
    def get_next_task(self) -> Optional[Task]:
        """
        Get the next task to execute with resolved coordinates.
        
        Returns None if no more tasks or queue is empty.
        """
        if not self.task_queue or self.task_queue.is_empty():
            return None
        
        task = self.task_queue.next()
        if task:
            # Resolve coordinates right before execution (uses current scene state)
            self.resolver.resolve(task)
            self.current_task = task
        
        return task
    
    def task_completed(self):
        """Mark current task as completed."""
        if self.task_queue and self.task_queue.is_empty():
            self.task_queue.status = "completed"
            print("[LLMController] All tasks completed!")
    
    def task_failed(self, reason: str = ""):
        """Mark current task as failed."""
        if self.task_queue:
            self.task_queue.status = "failed"
            print(f"[LLMController] Task failed: {reason}")
    
    def has_pending_tasks(self) -> bool:
        """Check if there are more tasks to execute."""
        return self.task_queue is not None and not self.task_queue.is_empty()


# ============================================================================
#  TESTING
# ============================================================================

if __name__ == "__main__":
    # Test the planner without Isaac Sim
    print("Testing LLM Planner (standalone)")
    print("=" * 60)
    
    # Mock scene description
    mock_scene = """Objects in scene:
- green_cube: position=(0.000, -0.200, 0.020), size=0.04x0.04x0.04m, color=green
- red_cube: position=(0.000, -0.250, 0.020), size=0.04x0.04x0.04m, color=red"""
    
    # Test command
    test_command = "put the red cube next to the green cube"
    
    print(f"\nScene:\n{mock_scene}")
    print(f"\nCommand: {test_command}")
    print("\nSending to LLM...")
    
    planner = TaskPlanner()
    queue = planner.plan_sync(mock_scene, test_command)
    
    print(f"\nGenerated {len(queue)} tasks:")
    for i, task in enumerate(queue.tasks):
        print(f"  {i+1}. {task}")
