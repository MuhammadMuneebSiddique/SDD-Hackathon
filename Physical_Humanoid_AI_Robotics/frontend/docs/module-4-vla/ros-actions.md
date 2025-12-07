---
sidebar_position: 4
title: 'Natural Language Task Planning'
---

# Natural Language Task Planning

This section covers connecting language understanding to ROS 2 actions, enabling robots to execute complex tasks based on natural language commands through sophisticated task planning and execution systems.

## Learning Objectives

After completing this section, you will be able to:
- Translate natural language commands to ROS 2 action sequences
- Implement hierarchical task planning for complex robot behaviors
- Create robust task execution frameworks with error handling
- Integrate perception feedback into task planning
- Implement adaptive task planning based on environmental changes
- Design multimodal task planning combining vision, language, and action

## Introduction to Natural Language Task Planning

Natural Language Task Planning (NLTP) is the process of converting high-level natural language instructions into executable sequences of robot actions. This involves several key components:

1. **Language Understanding**: Interpreting the semantic meaning of natural language commands
2. **Task Decomposition**: Breaking down complex tasks into primitive actions
3. **Action Grounding**: Mapping abstract concepts to concrete robot actions
4. **Plan Execution**: Executing the action sequence with appropriate monitoring
5. **Feedback Integration**: Adapting plans based on perception and execution results

### Key Challenges in NLTP

- **Semantic Ambiguity**: Natural language often contains ambiguous references
- **Spatial Reasoning**: Understanding spatial relationships and navigation
- **Temporal Dependencies**: Managing the temporal aspects of task execution
- **Context Awareness**: Incorporating environmental and situational context
- **Error Recovery**: Handling failures and unexpected situations gracefully

## Task Planning Architecture

### Hierarchical Task Planner

```python
import json
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum
import uuid
from datetime import datetime

class TaskStatus(Enum):
    PENDING = "pending"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"

class ActionType(Enum):
    NAVIGATE = "navigate"
    GRASP = "grasp"
    PLACE = "place"
    DETECT = "detect"
    SPEAK = "speak"
    WAIT = "wait"
    PERCEIVE = "perceive"
    MOVE_ARM = "move_arm"
    OPEN_GRIPPER = "open_gripper"
    CLOSE_GRIPPER = "close_gripper"
    FOLLOW_PATH = "follow_path"

@dataclass
class PrimitiveAction:
    """Lowest level executable action"""
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    action_type: ActionType
    parameters: Dict[str, Any]
    description: str
    priority: int = 1
    timeout: float = 30.0  # seconds
    preconditions: List[str] = field(default_factory=list)
    effects: List[str] = field(default_factory=list)

@dataclass
class CompoundTask:
    """Higher-level task composed of primitive actions or subtasks"""
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    description: str
    subtasks: List[Any]  # Union[PrimitiveAction, 'CompoundTask']
    status: TaskStatus = TaskStatus.PENDING
    created_at: datetime = field(default_factory=datetime.now)
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    parent_id: Optional[str] = None

class TaskPlanner:
    def __init__(self):
        self.tasks: Dict[str, CompoundTask] = {}
        self.primitive_actions: Dict[str, PrimitiveAction] = {}
        self.execution_context = {}

    def parse_command_to_tasks(self, command: str, context: Dict[str, Any]) -> CompoundTask:
        """Parse natural language command into hierarchical task structure"""
        # This would typically call an LLM or use rule-based parsing
        # For demonstration, we'll create a simple task structure

        if "bring me" in command.lower() or "pick up" in command.lower():
            return self.create_fetch_task(command, context)
        elif "go to" in command.lower() or "navigate to" in command.lower():
            return self.create_navigation_task(command, context)
        elif "put" in command.lower() or "place" in command.lower():
            return self.create_placement_task(command, context)
        else:
            return self.create_generic_task(command, context)

    def create_fetch_task(self, command: str, context: Dict[str, Any]) -> CompoundTask:
        """Create a task to fetch an object"""
        task = CompoundTask(
            name="fetch_object",
            description=f"Fetch object as requested in command: {command}",
        )

        # Decompose into subtasks
        # 1. Detect object
        detect_action = PrimitiveAction(
            action_type=ActionType.DETECT,
            parameters={"target_object": self.extract_object_from_command(command)},
            description="Detect the target object in the environment"
        )

        # 2. Navigate to object
        navigate_action = PrimitiveAction(
            action_type=ActionType.NAVIGATE,
            parameters={"target_location": "object_location"},
            description="Navigate to the location of the target object"
        )

        # 3. Grasp object
        grasp_action = PrimitiveAction(
            action_type=ActionType.GRASP,
            parameters={"object": self.extract_object_from_command(command)},
            description="Grasp the target object"
        )

        # 4. Navigate to destination (user location)
        return_navigate_action = PrimitiveAction(
            action_type=ActionType.NAVIGATE,
            parameters={"target_location": "user_location"},
            description="Navigate to the user location"
        )

        # 5. Place object
        place_action = PrimitiveAction(
            action_type=ActionType.PLACE,
            parameters={"object": self.extract_object_from_command(command)},
            description="Place the object for the user"
        )

        task.subtasks = [detect_action, navigate_action, grasp_action, return_navigate_action, place_action]
        return task

    def create_navigation_task(self, command: str, context: Dict[str, Any]) -> CompoundTask:
        """Create a navigation task"""
        task = CompoundTask(
            name="navigate",
            description=f"Navigate to location as requested in command: {command}",
        )

        # Extract target location from command
        target_location = self.extract_location_from_command(command)

        navigate_action = PrimitiveAction(
            action_type=ActionType.NAVIGATE,
            parameters={"target_location": target_location},
            description=f"Navigate to {target_location}"
        )

        task.subtasks = [navigate_action]
        return task

    def create_placement_task(self, command: str, context: Dict[str, Any]) -> CompoundTask:
        """Create a placement task"""
        task = CompoundTask(
            name="place_object",
            description=f"Place object as requested in command: {command}",
        )

        # Extract object and location from command
        target_object = self.extract_object_from_command(command)
        target_location = self.extract_location_from_command(command)

        # 1. Navigate to placement location
        navigate_action = PrimitiveAction(
            action_type=ActionType.NAVIGATE,
            parameters={"target_location": target_location},
            description=f"Navigate to {target_location} for placement"
        )

        # 2. Place object
        place_action = PrimitiveAction(
            action_type=ActionType.PLACE,
            parameters={"object": target_object, "location": target_location},
            description=f"Place {target_object} at {target_location}"
        )

        task.subtasks = [navigate_action, place_action]
        return task

    def create_generic_task(self, command: str, context: Dict[str, Any]) -> CompoundTask:
        """Create a generic task for unrecognized commands"""
        task = CompoundTask(
            name="generic",
            description=f"Generic task for command: {command}",
        )

        # For generic tasks, we might need more sophisticated parsing
        # This is a simplified version
        speak_action = PrimitiveAction(
            action_type=ActionType.SPEAK,
            parameters={"text": f"I'm not sure how to execute: {command}. Can you be more specific?"},
            description="Ask for clarification"
        )

        task.subtasks = [speak_action]
        return task

    def extract_object_from_command(self, command: str) -> str:
        """Extract object name from command (simplified)"""
        # In practice, this would use NLP techniques or LLM
        # This is a basic example
        object_keywords = [
            "cup", "bottle", "book", "phone", "toy", "keys",
            "apple", "banana", "water", "coffee", "sandwich"
        ]

        command_lower = command.lower()
        for keyword in object_keywords:
            if keyword in command_lower:
                return keyword

        return "unknown_object"

    def extract_location_from_command(self, command: str) -> str:
        """Extract location from command (simplified)"""
        # In practice, this would use NLP techniques or LLM
        location_keywords = [
            "kitchen", "bedroom", "living room", "office", "bathroom",
            "dining room", "hallway", "garage", "garden", "front door",
            "back door", "table", "chair", "couch", "shelf", "counter"
        ]

        command_lower = command.lower()
        for keyword in location_keywords:
            if keyword.replace(" ", "_") in command_lower.replace(" ", "_"):
                return keyword.replace(" ", "_")

        return "unknown_location"
```

## Advanced Task Planning with LLM Integration

### LLM-Guided Task Planner

```python
import openai
import json
from typing import List, Dict, Any

class LLMGuidedTaskPlanner(TaskPlanner):
    def __init__(self, llm_api_key: str, model_name: str = "gpt-4-turbo"):
        super().__init__()
        self.client = openai.OpenAI(api_key=llm_api_key)
        self.model_name = model_name

        # Define the robot's capabilities and action space
        self.robot_capabilities = {
            "navigation": {
                "actions": ["navigate", "move_to", "go_to", "travel_to"],
                "parameters": ["target_location", "path_type"]
            },
            "manipulation": {
                "actions": ["grasp", "pick_up", "take", "hold", "place", "put_down"],
                "parameters": ["object", "location", "gripper_position"]
            },
            "perception": {
                "actions": ["detect", "find", "locate", "identify", "recognize"],
                "parameters": ["target", "search_area", "confidence_threshold"]
            },
            "communication": {
                "actions": ["speak", "say", "tell", "announce"],
                "parameters": ["text", "volume", "language"]
            }
        }

    def plan_with_llm(self, command: str, context: Dict[str, Any] = None) -> CompoundTask:
        """Use LLM to generate task plan from natural language command"""
        # Construct detailed prompt for the LLM
        prompt = self._construct_planning_prompt(command, context)

        try:
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {
                        "role": "system",
                        "content": self._get_system_prompt()
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                temperature=0.1,
                response_format={"type": "json_object"}
            )

            # Parse the LLM response
            plan_data = json.loads(response.choices[0].message.content)
            return self._parse_plan_from_llm(plan_data)

        except Exception as e:
            print(f"LLM planning error: {e}")
            # Fallback to rule-based planning
            return self.parse_command_to_tasks(command, context or {})

    def _construct_planning_prompt(self, command: str, context: Dict[str, Any] = None) -> str:
        """Construct detailed prompt for LLM-based task planning"""
        prompt_parts = []

        # Add robot capabilities
        prompt_parts.append(f"""
        Robot Capabilities:
        {json.dumps(self.robot_capabilities, indent=2)}
        """)

        # Add environmental context
        if context:
            prompt_parts.append(f"""
            Environmental Context:
            {json.dumps(context, indent=2)}
            """)

        # Add the command
        prompt_parts.append(f"""
        Natural Language Command:
        "{command}"

        Please decompose this command into a hierarchical task structure with the following JSON format:

        {{
            "task_name": "name_of_main_task",
            "description": "brief_description_of_the_task",
            "subtasks": [
                {{
                    "action_type": "action_name",
                    "parameters": {{"param1": "value1", "param2": "value2"}},
                    "description": "what_this_action_does",
                    "dependencies": ["action_id_1", "action_id_2"]  // optional
                }}
            ]
        }}

        Important considerations:
        1. Use only the actions defined in the robot capabilities
        2. Ensure temporal dependencies are properly represented
        3. Include necessary preconditions and effects
        4. Make the plan executable and realistic
        5. Consider safety and environmental constraints
        """)

        return "\n".join(prompt_parts)

    def _get_system_prompt(self) -> str:
        """Get the system prompt for LLM guidance"""
        return """
        You are an expert task planner for a humanoid robot. Your role is to decompose natural language commands into executable task sequences. Consider the following:

        1. The robot has specific capabilities for navigation, manipulation, perception, and communication
        2. Tasks should be broken down into primitive actions that the robot can execute
        3. Consider spatial and temporal relationships in the environment
        4. Account for the robot's current state and environmental context
        5. Ensure the task sequence is logically coherent and executable
        6. Include necessary error handling and validation steps

        Always return valid JSON with the specified structure.
        """

    def _parse_plan_from_llm(self, plan_data: Dict[str, Any]) -> CompoundTask:
        """Parse the LLM-generated plan into a CompoundTask"""
        task = CompoundTask(
            name=plan_data.get("task_name", "unnamed_task"),
            description=plan_data.get("description", "No description provided")
        )

        for subtask_data in plan_data.get("subtasks", []):
            try:
                action_type = ActionType(subtask_data["action_type"])
                action = PrimitiveAction(
                    action_type=action_type,
                    parameters=subtask_data.get("parameters", {}),
                    description=subtask_data.get("description", "")
                )
                task.subtasks.append(action)
            except ValueError:
                print(f"Invalid action type: {subtask_data['action_type']}")
                # Skip invalid actions or use fallback

        return task
```

## Task Execution Framework

### Robust Task Executor

```python
import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor
from typing import Callable, Optional

class TaskExecutor:
    def __init__(self, task_planner: TaskPlanner):
        self.planner = task_planner
        self.executor = ThreadPoolExecutor(max_workers=4)
        self.current_task: Optional[CompoundTask] = None
        self.cancel_flag = False
        self.perception_callback: Optional[Callable] = None

    def execute_task(self, task: CompoundTask) -> bool:
        """Execute a compound task with error handling and monitoring"""
        self.current_task = task
        self.cancel_flag = False
        task.started_at = datetime.now()
        task.status = TaskStatus.EXECUTING

        try:
            success = self._execute_subtasks(task.subtasks)

            task.completed_at = datetime.now()
            task.status = TaskStatus.COMPLETED if success else TaskStatus.FAILED

            return success

        except Exception as e:
            task.status = TaskStatus.FAILED
            print(f"Task execution failed: {e}")
            return False
        finally:
            self.current_task = None

    def _execute_subtasks(self, subtasks: List[Any]) -> bool:
        """Execute a list of subtasks in sequence"""
        for i, subtask in enumerate(subtasks):
            if self.cancel_flag:
                return False

            if isinstance(subtask, PrimitiveAction):
                success = self._execute_primitive_action(subtask)
                if not success:
                    print(f"Failed to execute primitive action: {subtask.description}")
                    return False
            elif isinstance(subtask, CompoundTask):
                success = self.execute_task(subtask)
                if not success:
                    print(f"Failed to execute compound task: {subtask.name}")
                    return False

        return True

    def _execute_primitive_action(self, action: PrimitiveAction) -> bool:
        """Execute a primitive action"""
        print(f"Executing action: {action.description}")

        try:
            # Execute based on action type
            if action.action_type == ActionType.NAVIGATE:
                return self._execute_navigation(action)
            elif action.action_type == ActionType.GRASP:
                return self._execute_grasp(action)
            elif action.action_type == ActionType.PLACE:
                return self._execute_place(action)
            elif action.action_type == ActionType.DETECT:
                return self._execute_detection(action)
            elif action.action_type == ActionType.SPEAK:
                return self._execute_speak(action)
            elif action.action_type == ActionType.WAIT:
                return self._execute_wait(action)
            elif action.action_type == ActionType.PERCEIVE:
                return self._execute_perceive(action)
            elif action.action_type == ActionType.MOVE_ARM:
                return self._execute_arm_movement(action)
            elif action.action_type == ActionType.OPEN_GRIPPER:
                return self._execute_gripper_action(action, "open")
            elif action.action_type == ActionType.CLOSE_GRIPPER:
                return self._execute_gripper_action(action, "close")
            else:
                print(f"Unknown action type: {action.action_type}")
                return False

        except Exception as e:
            print(f"Error executing action {action.description}: {e}")
            return False

    def _execute_navigation(self, action: PrimitiveAction) -> bool:
        """Execute navigation action"""
        target_location = action.parameters.get('target_location', 'unknown')
        print(f"Navigating to {target_location}")

        # In practice, this would call navigation services
        # For simulation, we'll just return success
        return True

    def _execute_grasp(self, action: PrimitiveAction) -> bool:
        """Execute grasp action"""
        obj_name = action.parameters.get('object', 'unknown_object')
        print(f"Attempting to grasp {obj_name}")

        # In practice, this would call manipulation services
        # Check if object is reachable and graspable
        return True

    def _execute_place(self, action: PrimitiveAction) -> bool:
        """Execute place action"""
        obj_name = action.parameters.get('object', 'unknown_object')
        location = action.parameters.get('location', 'unknown_location')
        print(f"Placing {obj_name} at {location}")

        # In practice, this would call placement services
        return True

    def _execute_detection(self, action: PrimitiveAction) -> bool:
        """Execute detection action"""
        target = action.parameters.get('target', 'environment')
        print(f"Detecting {target}")

        # In practice, this would call perception services
        # Update context with detected objects
        return True

    def _execute_speak(self, action: PrimitiveAction) -> bool:
        """Execute speech action"""
        text = action.parameters.get('text', '')
        print(f"Speaking: {text}")

        # In practice, this would call TTS services
        return True

    def _execute_wait(self, action: PrimitiveAction) -> bool:
        """Execute wait action"""
        duration = action.parameters.get('duration', 1.0)
        print(f"Waiting for {duration} seconds")

        # In practice, this would sleep or wait for conditions
        import time
        time.sleep(duration)
        return True

    def _execute_perceive(self, action: PrimitiveAction) -> bool:
        """Execute perception action"""
        target = action.parameters.get('target', 'environment')
        print(f"Perceiving {target}")

        # In practice, this would call perception services
        # Update context with perceived information
        return True

    def _execute_arm_movement(self, action: PrimitiveAction) -> bool:
        """Execute arm movement action"""
        joints = action.parameters.get('joints', [])
        print(f"Moving arm to joints: {joints}")

        # In practice, this would call arm control services
        return True

    def _execute_gripper_action(self, action: PrimitiveAction, action_type: str) -> bool:
        """Execute gripper action (open/close)"""
        print(f"{action_type.capitalize()}ing gripper")

        # In practice, this would call gripper control services
        return True

    def cancel_current_task(self):
        """Cancel the currently executing task"""
        self.cancel_flag = True
        print("Task cancellation requested")

    def set_perception_callback(self, callback: Callable):
        """Set callback for perception updates during task execution"""
        self.perception_callback = callback
```

## Perception-Driven Task Adaptation

### Adaptive Task Planner

```python
class AdaptiveTaskPlanner(LLMGuidedTaskPlanner):
    def __init__(self, llm_api_key: str, model_name: str = "gpt-4-turbo"):
        super().__init__(llm_api_key, model_name)
        self.perception_buffer = []
        self.context_updates = []

    def adapt_task_plan(self, original_task: CompoundTask, perception_update: Dict[str, Any]) -> CompoundTask:
        """Adapt task plan based on perception updates"""
        # Add perception update to buffer
        self.perception_buffer.append(perception_update)

        # Check if adaptation is needed
        if self._needs_adaptation(original_task, perception_update):
            return self._generate_adapted_plan(original_task, perception_update)

        return original_task

    def _needs_adaptation(self, task: CompoundTask, perception: Dict[str, Any]) -> bool:
        """Check if current task needs adaptation based on perception"""
        # Check for obstacles in navigation path
        if any(isinstance(subtask, PrimitiveAction) and
               subtask.action_type == ActionType.NAVIGATE for subtask in task.subtasks):
            if perception.get('obstacle_detected', False):
                return True

        # Check if target object is no longer available
        if any(isinstance(subtask, PrimitiveAction) and
               subtask.action_type == ActionType.GRASP for subtask in task.subtasks):
            target_obj = self._get_target_object(task)
            if target_obj and not self._object_still_available(target_obj, perception):
                return True

        # Check for environmental changes
        if perception.get('environment_changed', False):
            return True

        return False

    def _get_target_object(self, task: CompoundTask) -> Optional[str]:
        """Get the target object from task"""
        for subtask in task.subtasks:
            if (isinstance(subtask, PrimitiveAction) and
                subtask.action_type in [ActionType.GRASP, ActionType.DETECT] and
                'object' in subtask.parameters):
                return subtask.parameters['object']
        return None

    def _object_still_available(self, obj_name: str, perception: Dict[str, Any]) -> bool:
        """Check if object is still available in perception"""
        detected_objects = perception.get('objects', [])
        return obj_name in [obj.get('name') for obj in detected_objects]

    def _generate_adapted_plan(self, original_task: CompoundTask, perception: Dict[str, Any]) -> CompoundTask:
        """Generate adapted plan based on perception update"""
        # Create context for adaptation
        adaptation_context = {
            'original_task': self._serialize_task(original_task),
            'perception_update': perception,
            'environment_state': self._get_current_environment_state()
        }

        # Use LLM to generate adapted plan
        prompt = f"""
        Original task: {json.dumps(adaptation_context['original_task'], indent=2)}

        Perception update: {json.dumps(adaptation_context['perception_update'], indent=2)}

        Current environment state: {json.dumps(adaptation_context['environment_state'], indent=2)}

        The original task needs to be adapted due to the perception update. Please generate a new task plan that:
        1. Addresses the changed conditions
        2. Maintains the original goal if possible
        3. Incorporates the new information from perception
        4. Is executable with the robot's capabilities

        Return the adapted plan in the same JSON format as before.
        """

        try:
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,
                response_format={"type": "json_object"}
            )

            plan_data = json.loads(response.choices[0].message.content)
            return self._parse_plan_from_llm(plan_data)

        except Exception as e:
            print(f"Adaptation planning error: {e}")
            # Return original task if adaptation fails
            return original_task

    def _serialize_task(self, task: CompoundTask) -> Dict[str, Any]:
        """Serialize task to dictionary for LLM processing"""
        return {
            'name': task.name,
            'description': task.description,
            'subtasks': [
                {
                    'action_type': subtask.action_type.value,
                    'parameters': subtask.parameters,
                    'description': subtask.description
                } if isinstance(subtask, PrimitiveAction) else {
                    'compound_task': self._serialize_task(subtask)
                }
                for subtask in task.subtasks
            ]
        }

    def _get_current_environment_state(self) -> Dict[str, Any]:
        """Get current environment state from perception buffer"""
        # Aggregate recent perception data
        recent_perceptions = self.perception_buffer[-10:]  # Last 10 updates

        state = {
            'objects': [],
            'obstacles': [],
            'free_spaces': [],
            'robot_position': None,
            'time': datetime.now().isoformat()
        }

        for perception in recent_perceptions:
            if 'objects' in perception:
                state['objects'].extend(perception['objects'])
            if 'obstacles' in perception:
                state['obstacles'].extend(perception['obstacles'])

        # Remove duplicates and return state
        state['objects'] = list({obj.get('name'): obj for obj in state['objects']}.values())
        return state
```

## ROS 2 Integration

### ROS 2 Task Execution Node

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from moveit_msgs.action import MoveGroup
from nav2_msgs.action import NavigateToPose

from typing import Dict, Any
import json

class ROS2TaskExecutionNode(Node):
    def __init__(self):
        super().__init__('ros2_task_execution_node')

        # Initialize LLM-guided planner
        self.planner = AdaptiveTaskPlanner(
            llm_api_key="your-openai-api-key",
            model_name="gpt-4-turbo"
        )

        # Initialize task executor
        self.executor = TaskExecutor(self.planner)

        # ROS 2 publishers and subscribers
        self.command_sub = self.create_subscription(
            String, '/natural_language_command', self.command_callback, 10
        )
        self.status_pub = self.create_publisher(
            String, '/task_execution_status', 10
        )
        self.feedback_pub = self.create_publisher(
            String, '/task_execution_feedback', 10
        )

        # Perception subscribers
        self.perception_sub = self.create_subscription(
            String, '/perception_update', self.perception_callback, 10
        )

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.move_group_client = ActionClient(self, MoveGroup, 'move_group')

        # Task execution variables
        self.current_task = None
        self.task_cancel_requested = False

        self.get_logger().info("ROS2 Task Execution Node initialized")

    def command_callback(self, msg: String):
        """Process natural language command"""
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # Get current context (could be from various sensors/other nodes)
        context = self._get_current_context()

        # Plan task using LLM
        try:
            task = self.planner.plan_with_llm(command, context)
            self.get_logger().info(f"Generated task: {task.name}")

            # Execute task
            self.execute_task_async(task)

        except Exception as e:
            self.get_logger().error(f"Task planning/execution error: {e}")
            self._publish_status(f"Error processing command: {e}")

    def perception_callback(self, msg: String):
        """Handle perception updates for task adaptation"""
        try:
            perception_data = json.loads(msg.data)

            # If we have a current task, adapt it if necessary
            if self.current_task:
                adapted_task = self.planner.adapt_task_plan(self.current_task, perception_data)

                if adapted_task != self.current_task:
                    self.get_logger().info("Task adapted based on perception update")
                    self.current_task = adapted_task

        except json.JSONDecodeError:
            self.get_logger().error("Invalid perception data format")

    def execute_task_async(self, task):
        """Execute task asynchronously"""
        self.current_task = task

        # Publish task start
        self._publish_status(f"Starting task execution: {task.name}")

        # Execute in separate thread to not block the ROS2 node
        executor_thread = threading.Thread(target=self._execute_task_thread, args=(task,))
        executor_thread.daemon = True
        executor_thread.start()

    def _execute_task_thread(self, task):
        """Execute task in separate thread"""
        success = self.executor.execute_task(task)

        # Publish completion status
        if success:
            self._publish_status(f"Task completed successfully: {task.name}")
        else:
            self._publish_status(f"Task failed: {task.name}")

    def _get_current_context(self) -> Dict[str, Any]:
        """Get current environmental context"""
        # In practice, this would gather data from various sensors and nodes
        context = {
            'robot_state': {
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                'battery_level': 0.8,
                'gripper_status': 'open'
            },
            'environment': {
                'known_objects': ['cup', 'book', 'phone'],
                'known_locations': ['kitchen', 'living_room', 'bedroom'],
                'map': 'current_map'
            },
            'capabilities': ['navigation', 'manipulation', 'perception', 'speech']
        }
        return context

    def _publish_status(self, status: str):
        """Publish task execution status"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

    def _publish_feedback(self, feedback: str):
        """Publish task execution feedback"""
        feedback_msg = String()
        feedback_msg.data = feedback
        self.feedback_pub.publish(feedback_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ROS2TaskExecutionNode()

    try:
        # Use multi-threaded executor to handle callbacks and task execution
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ROS2 Task Execution Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Error Handling and Recovery

### Task Recovery Manager

```python
from enum import Enum
from typing import List, Dict, Any, Callable

class RecoveryStrategy(Enum):
    RETRY = "retry"
    SKIP = "skip"
    REPLAN = "replan"
    ABORT = "abort"
    FALLBACK = "fallback"

class TaskRecoveryManager:
    def __init__(self):
        self.recovery_strategies = {
            'navigation_failure': RecoveryStrategy.REPLAN,
            'grasp_failure': RecoveryStrategy.SKIP,
            'perception_failure': RecoveryStrategy.WAIT_AND_RETRY,
            'communication_failure': RecoveryStrategy.SKIP
        }

        self.failure_history = []
        self.max_retries = 3

    def handle_failure(self, failed_action: PrimitiveAction, error: Exception) -> RecoveryStrategy:
        """Determine recovery strategy based on failure type"""
        failure_type = self._classify_failure(failed_action, error)

        # Log failure
        self.failure_history.append({
            'action': failed_action,
            'error': str(error),
            'failure_type': failure_type,
            'timestamp': datetime.now()
        })

        # Determine recovery strategy
        strategy = self.recovery_strategies.get(failure_type, RecoveryStrategy.ABORT)

        # Apply additional logic based on context
        if strategy == RecoveryStrategy.RETRY:
            retry_count = self._get_retry_count(failed_action)
            if retry_count >= self.max_retries:
                strategy = RecoveryStrategy.REPLAN

        return strategy

    def _classify_failure(self, action: PrimitiveAction, error: Exception) -> str:
        """Classify the type of failure"""
        error_str = str(error).lower()

        if action.action_type == ActionType.NAVIGATE:
            if "obstacle" in error_str or "collision" in error_str:
                return "navigation_failure"
            elif "goal" in error_str or "unreachable" in error_str:
                return "navigation_failure"

        elif action.action_type == ActionType.GRASP:
            if "object" in error_str or "grasp" in error_str:
                return "grasp_failure"

        elif action.action_type == ActionType.PERCEIVE or action.action_type == ActionType.DETECT:
            if "not_found" in error_str or "no_detection" in error_str:
                return "perception_failure"

        return "general_failure"

    def _get_retry_count(self, action: PrimitiveAction) -> int:
        """Get number of times this action has been retried"""
        count = 0
        for failure in reversed(self.failure_history[-10:]):  # Check last 10 failures
            if failure['action'].id == action.id:
                count += 1
            else:
                break
        return count

    def execute_recovery_strategy(self, strategy: RecoveryStrategy, failed_action: PrimitiveAction,
                                 current_task: CompoundTask) -> bool:
        """Execute the chosen recovery strategy"""
        if strategy == RecoveryStrategy.RETRY:
            return self._retry_action(failed_action)
        elif strategy == RecoveryStrategy.SKIP:
            return self._skip_action(failed_action, current_task)
        elif strategy == RecoveryStrategy.REPLAN:
            return self._replan_task(current_task, failed_action)
        elif strategy == RecoveryStrategy.ABORT:
            return False  # Task execution should be aborted
        elif strategy == RecoveryStrategy.FALLBACK:
            return self._execute_fallback(failed_action)
        else:
            return False

    def _retry_action(self, action: PrimitiveAction) -> bool:
        """Retry the failed action"""
        print(f"Retrying action: {action.description}")
        # In practice, this would reset action state and retry
        return True

    def _skip_action(self, action: PrimitiveAction, task: CompoundTask) -> bool:
        """Skip the failed action and continue with the rest of the task"""
        print(f"Skipping action: {action.description}")
        # Mark action as skipped in task context
        return True

    def _replan_task(self, original_task: CompoundTask, failed_action: PrimitiveAction) -> bool:
        """Generate a new plan excluding the failed action"""
        print(f"Replanning task due to failure in: {failed_action.description}")
        # In practice, this would call the planner with constraints
        return True

    def _execute_fallback(self, action: PrimitiveAction) -> bool:
        """Execute fallback behavior for the action"""
        print(f"Executing fallback for action: {action.description}")
        # Execute alternative behavior
        return True
```

## Lab Activity: Natural Language Task Planning System

Create a complete natural language task planning system with:
1. LLM-guided task decomposition
2. Hierarchical task structure
3. Robust execution framework
4. Perception-driven adaptation
5. Error handling and recovery
6. ROS 2 integration

### Steps:
1. Set up LLM-guided task planner with OpenAI API
2. Implement hierarchical task structure with primitive and compound tasks
3. Create robust task executor with error handling
4. Implement perception-driven task adaptation
5. Add comprehensive error handling and recovery mechanisms
6. Integrate with ROS 2 for real robot execution
7. Test with various natural language commands
8. Validate task execution in simulation

### Expected Outcome:
- Functional natural language task planning system
- LLM-guided task decomposition working
- Hierarchical task structure implemented
- Robust execution with error handling
- Perception-driven adaptation operational
- ROS 2 integration working
- Lab activity completed successfully

## Best Practices

1. **Modular Design**: Keep task planning, execution, and adaptation separate
2. **Error Handling**: Implement comprehensive error handling at all levels
3. **Context Awareness**: Maintain rich context for better planning decisions
4. **Safety**: Always validate actions before execution
5. **Recovery**: Plan for failure scenarios with appropriate recovery strategies
6. **Testing**: Test with diverse natural language inputs and scenarios
7. **Monitoring**: Log all task executions for analysis and improvement

## Checklist

- [ ] LLM-guided task planner implemented
- [ ] Hierarchical task structure created
- [ ] Robust task execution framework operational
- [ ] Perception-driven adaptation working
- [ ] Error handling and recovery mechanisms in place
- [ ] ROS 2 integration completed
- [ ] Task planning system validated
- [ ] Lab activity completed successfully

## Next Steps

In the next module, we'll explore the capstone project that integrates all concepts learned throughout the course, creating an autonomous humanoid robot system that can take voice commands, plan navigation, identify objects, and manipulate them to complete complex tasks.