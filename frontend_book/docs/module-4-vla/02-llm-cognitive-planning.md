---
sidebar_position: 2
title: "LLM-Based Cognitive Planning"
description: "Use Large Language Models to convert natural language commands into robot action plans"
keywords: [llm, gpt, planning, robotics, action, ros2, openai, cognitive]
---

# LLM-Based Cognitive Planning

## What You'll Learn

- Understand how LLMs serve as cognitive planners for robots
- Design effective system prompts for action generation
- Generate structured action plans in JSON format
- Parse LLM outputs into executable action primitives
- Map action primitives to ROS 2 action interfaces
- Implement safety validation for generated plans

:::info Prerequisites
This chapter builds on **Chapter 1** (voice commands available on `/voice_command` topic).

**Required**: OpenAI API key or alternative LLM access
:::

## LLMs as Robot Planners

Large Language Models (LLMs) like GPT-4 excel at understanding natural language and reasoning about tasks. This makes them powerful **cognitive planners** for robots—translating human intent into structured action sequences.

### From Language to Action

Consider this voice command: *"Go to the kitchen and get me a glass of water"*

A human understands this involves multiple steps:
1. Navigate to kitchen
2. Find a glass
3. Fill with water
4. Return to user

LLMs can perform this same reasoning, outputting a structured plan the robot can execute.

### The LLM Planning Pipeline

```
┌─────────────────┐
│ Voice Command   │  "Go to the kitchen and wave hello"
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ System Prompt   │  Robot capabilities, output format, safety rules
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ LLM (GPT-4)     │  Reasoning and planning
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ JSON Action Plan│  Structured, parseable output
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Action Executor │  ROS 2 action clients
└─────────────────┘
```

### Capabilities and Limitations

**LLMs Excel At**:
- Understanding complex, multi-step instructions
- Handling ambiguous or incomplete commands
- Generating structured output formats
- Reasoning about sequences and dependencies

**LLMs Struggle With**:
- Real-time sensor feedback
- Precise spatial reasoning
- Physical constraints they haven't learned
- Consistent output format without prompting

## Prompt Engineering for Robotics

The **system prompt** defines the LLM's role, capabilities, and output format. A well-designed prompt is essential for reliable robot planning.

### System Prompt Design

```text title="system_prompt.txt"
You are a robot task planner for a humanoid robot. Your job is to convert
natural language commands into structured action plans.

## Robot Capabilities

The robot can perform these actions:

1. **navigate**: Move to a named location or coordinates
   - Parameters: target (string) OR coordinates (x, y, theta)
   - Example: navigate to "kitchen" or navigate to (2.5, 3.0, 0.0)

2. **gesture**: Perform a gesture
   - Parameters: gesture_name (wave, point, nod, shake_head)
   - Parameters: duration_sec (optional, default 3.0)

3. **pick**: Pick up an object
   - Parameters: object_name (string)
   - Requires: robot must be near the object

4. **place**: Place a held object
   - Parameters: location (string)
   - Requires: robot must be holding an object

5. **speak**: Say something out loud
   - Parameters: text (string to speak)

6. **wait**: Pause for a duration
   - Parameters: duration_sec (float)

## Known Locations

- living_room: (0.0, 0.0, 0.0)
- kitchen: (5.0, 2.0, 1.57)
- bedroom: (-3.0, 4.0, 3.14)
- front_door: (0.0, -5.0, -1.57)
- table: (2.0, 1.0, 0.0)

## Output Format

Always respond with valid JSON in this exact format:

{
  "understood": true,
  "command_summary": "Brief description of what was requested",
  "actions": [
    {
      "action_type": "navigate|gesture|pick|place|speak|wait",
      "parameters": { ... action-specific parameters ... },
      "description": "Human-readable description"
    }
  ],
  "warnings": ["Any concerns about the command"]
}

## Safety Rules

1. Never generate actions that could harm humans
2. If a command is unclear, set "understood": false and ask for clarification
3. If a command is impossible, explain why in "warnings"
4. Always validate that required preconditions are met

## Examples

Command: "Go to the kitchen"
Response:
{
  "understood": true,
  "command_summary": "Navigate to the kitchen",
  "actions": [
    {
      "action_type": "navigate",
      "parameters": {"target": "kitchen"},
      "description": "Move to the kitchen"
    }
  ],
  "warnings": []
}

Command: "Wave at the person by the door"
Response:
{
  "understood": true,
  "command_summary": "Navigate to front door and wave",
  "actions": [
    {
      "action_type": "navigate",
      "parameters": {"target": "front_door"},
      "description": "Move to the front door"
    },
    {
      "action_type": "gesture",
      "parameters": {"gesture_name": "wave", "duration_sec": 3.0},
      "description": "Wave greeting"
    }
  ],
  "warnings": []
}
```

### Key Prompt Elements

1. **Role Definition**: Clearly state the LLM is a robot planner
2. **Capability List**: Enumerate exactly what actions are available
3. **Output Schema**: Define the exact JSON structure expected
4. **Examples**: Provide input-output examples for consistency
5. **Safety Rules**: Constrain dangerous or impossible actions

## Generating Action Plans

Now let's implement the LLM planner as a ROS 2 node:

```python title="llm_planner_node.py"
#!/usr/bin/env python3
"""ROS 2 node that uses LLM to generate action plans from voice commands."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from openai import OpenAI

# System prompt (loaded from file in production)
SYSTEM_PROMPT = """You are a robot task planner for a humanoid robot. Convert natural language commands into structured action plans.

## Robot Capabilities
- navigate: Move to location. Parameters: target (string) or coordinates (x, y, theta)
- gesture: Perform gesture. Parameters: gesture_name (wave|point|nod), duration_sec
- pick: Pick object. Parameters: object_name
- place: Place object. Parameters: location
- speak: Say text. Parameters: text
- wait: Pause. Parameters: duration_sec

## Known Locations
- living_room, kitchen, bedroom, front_door, table

## Output Format (JSON only)
{
  "understood": true/false,
  "command_summary": "brief description",
  "actions": [
    {"action_type": "...", "parameters": {...}, "description": "..."}
  ],
  "warnings": []
}

Always respond with valid JSON only. No additional text."""


class LLMPlannerNode(Node):
    """Subscribes to voice commands, generates action plans via LLM."""

    def __init__(self):
        super().__init__('llm_planner_node')

        # Parameters
        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('model', 'gpt-4o-mini')

        api_key = self.get_parameter('openai_api_key').value
        if not api_key:
            api_key = os.environ.get('OPENAI_API_KEY', '')

        if not api_key:
            self.get_logger().error('OpenAI API key not set!')
            raise ValueError('OPENAI_API_KEY required')

        self.model = self.get_parameter('model').value
        self.client = OpenAI(api_key=api_key)

        # Subscribers and publishers
        self.subscription = self.create_subscription(
            String,
            '/voice_command',
            self.command_callback,
            10
        )
        self.plan_publisher = self.create_publisher(String, '/action_plan', 10)

        self.get_logger().info(f'LLM Planner ready (model: {self.model})')

    def command_callback(self, msg):
        """Process incoming voice command."""
        command = msg.data
        self.get_logger().info(f"Received command: '{command}'")

        try:
            # Call LLM
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": command}
                ],
                temperature=0.1,  # Low temperature for consistency
                max_tokens=500
            )

            plan_text = response.choices[0].message.content

            # Parse JSON
            plan = json.loads(plan_text)

            # Log result
            action_count = len(plan.get('actions', []))
            self.get_logger().info(
                f"Generated plan with {action_count} actions: "
                f"{plan.get('command_summary', 'No summary')}"
            )

            # Publish plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_publisher.publish(plan_msg)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse LLM response: {e}')
        except Exception as e:
            self.get_logger().error(f'LLM call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Expected Output**:
```
[INFO] [llm_planner_node]: LLM Planner ready (model: gpt-4o-mini)
[INFO] [llm_planner_node]: Received command: 'go to the kitchen and wave'
[INFO] [llm_planner_node]: Generated plan with 2 actions: Navigate to kitchen and wave
```

## Parsing and Validating LLM Output

LLM outputs can be inconsistent. We need robust parsing and validation:

```python title="action_parser.py"
#!/usr/bin/env python3
"""Parse and validate LLM-generated action plans."""

import json
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from enum import Enum


class ActionType(Enum):
    NAVIGATE = "navigate"
    GESTURE = "gesture"
    PICK = "pick"
    PLACE = "place"
    SPEAK = "speak"
    WAIT = "wait"


@dataclass
class ActionPrimitive:
    """Single executable action."""
    action_type: ActionType
    parameters: Dict[str, Any]
    description: str

    def __repr__(self):
        return f"{self.action_type.value}({self.parameters})"


@dataclass
class ActionPlan:
    """Complete action plan from LLM."""
    understood: bool
    command_summary: str
    actions: List[ActionPrimitive]
    warnings: List[str]


class ActionParser:
    """Parses and validates LLM action plans."""

    VALID_GESTURES = {"wave", "point", "nod", "shake_head"}
    KNOWN_LOCATIONS = {"living_room", "kitchen", "bedroom", "front_door", "table"}

    def parse(self, plan_json: str) -> Optional[ActionPlan]:
        """Parse JSON string into ActionPlan."""
        try:
            data = json.loads(plan_json)
        except json.JSONDecodeError as e:
            print(f"JSON parse error: {e}")
            return None

        # Validate required fields
        if not isinstance(data.get('understood'), bool):
            print("Missing or invalid 'understood' field")
            return None

        if not data['understood']:
            return ActionPlan(
                understood=False,
                command_summary=data.get('command_summary', 'Command not understood'),
                actions=[],
                warnings=data.get('warnings', ['Command was not understood'])
            )

        # Parse actions
        actions = []
        for action_data in data.get('actions', []):
            action = self._parse_action(action_data)
            if action:
                actions.append(action)

        return ActionPlan(
            understood=True,
            command_summary=data.get('command_summary', ''),
            actions=actions,
            warnings=data.get('warnings', [])
        )

    def _parse_action(self, data: Dict) -> Optional[ActionPrimitive]:
        """Parse single action from dict."""
        try:
            action_type = ActionType(data['action_type'])
        except (KeyError, ValueError):
            print(f"Invalid action type: {data.get('action_type')}")
            return None

        params = data.get('parameters', {})

        # Validate parameters based on action type
        if not self._validate_params(action_type, params):
            return None

        return ActionPrimitive(
            action_type=action_type,
            parameters=params,
            description=data.get('description', '')
        )

    def _validate_params(self, action_type: ActionType, params: Dict) -> bool:
        """Validate parameters for action type."""
        if action_type == ActionType.NAVIGATE:
            if 'target' in params:
                return params['target'] in self.KNOWN_LOCATIONS
            if 'coordinates' in params:
                coords = params['coordinates']
                return (isinstance(coords, (list, tuple)) and
                        len(coords) == 3 and
                        all(isinstance(c, (int, float)) for c in coords))
            return False

        elif action_type == ActionType.GESTURE:
            gesture = params.get('gesture_name')
            return gesture in self.VALID_GESTURES

        elif action_type == ActionType.PICK:
            return 'object_name' in params

        elif action_type == ActionType.PLACE:
            return 'location' in params

        elif action_type == ActionType.SPEAK:
            return 'text' in params and isinstance(params['text'], str)

        elif action_type == ActionType.WAIT:
            duration = params.get('duration_sec', 1.0)
            return isinstance(duration, (int, float)) and duration > 0

        return False


# Example usage
if __name__ == '__main__':
    parser = ActionParser()

    # Example LLM output
    llm_response = '''
    {
        "understood": true,
        "command_summary": "Navigate to kitchen and wave",
        "actions": [
            {
                "action_type": "navigate",
                "parameters": {"target": "kitchen"},
                "description": "Move to the kitchen"
            },
            {
                "action_type": "gesture",
                "parameters": {"gesture_name": "wave", "duration_sec": 3.0},
                "description": "Wave greeting"
            }
        ],
        "warnings": []
    }
    '''

    plan = parser.parse(llm_response)

    if plan and plan.understood:
        print(f"Summary: {plan.command_summary}")
        print(f"Parsed actions: {plan.actions}")
    else:
        print("Failed to parse plan")
```

**Expected Output**:
```
Summary: Navigate to kitchen and wave
Parsed actions: [navigate({'target': 'kitchen'}), gesture({'gesture_name': 'wave', 'duration_sec': 3.0})]
```

## Mapping Actions to ROS 2

Once parsed, actions must be executed via ROS 2 action clients:

```python title="action_executor.py"
#!/usr/bin/env python3
"""Execute parsed action plans via ROS 2 action clients."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import json
import math

# Import our parser
from action_parser import ActionParser, ActionType, ActionPlan


class ActionExecutor(Node):
    """Executes action plans by calling ROS 2 action servers."""

    # Location coordinates (x, y, theta)
    LOCATIONS = {
        'living_room': (0.0, 0.0, 0.0),
        'kitchen': (5.0, 2.0, 1.57),
        'bedroom': (-3.0, 4.0, 3.14),
        'front_door': (0.0, -5.0, -1.57),
        'table': (2.0, 1.0, 0.0),
    }

    def __init__(self):
        super().__init__('action_executor')

        # Parser
        self.parser = ActionParser()

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriber for action plans
        self.subscription = self.create_subscription(
            String,
            '/action_plan',
            self.plan_callback,
            10
        )

        # Status publisher
        self.status_pub = self.create_publisher(String, '/execution_status', 10)

        self.get_logger().info('Action Executor ready')

    def plan_callback(self, msg):
        """Handle incoming action plan."""
        plan = self.parser.parse(msg.data)

        if not plan or not plan.understood:
            self.publish_status('error', 'Failed to parse plan')
            return

        self.get_logger().info(f'Executing plan: {plan.command_summary}')
        self.execute_plan(plan)

    def execute_plan(self, plan: ActionPlan):
        """Execute actions sequentially."""
        for i, action in enumerate(plan.actions):
            self.get_logger().info(
                f'Executing action {i+1}/{len(plan.actions)}: '
                f'{action.action_type.value}'
            )
            self.publish_status('executing', f'Action {i+1}: {action.description}')

            success = self.execute_action(action)

            if not success:
                self.publish_status('error', f'Action {i+1} failed')
                return

        self.publish_status('completed', 'All actions completed')

    def execute_action(self, action) -> bool:
        """Execute single action."""
        if action.action_type == ActionType.NAVIGATE:
            return self.execute_navigate(action.parameters)
        elif action.action_type == ActionType.GESTURE:
            return self.execute_gesture(action.parameters)
        elif action.action_type == ActionType.WAIT:
            return self.execute_wait(action.parameters)
        elif action.action_type == ActionType.SPEAK:
            return self.execute_speak(action.parameters)
        else:
            self.get_logger().warn(f'Unhandled action: {action.action_type}')
            return True  # Skip but don't fail

    def execute_navigate(self, params) -> bool:
        """Send navigation goal."""
        # Get coordinates
        if 'target' in params:
            coords = self.LOCATIONS.get(params['target'])
            if not coords:
                self.get_logger().error(f"Unknown location: {params['target']}")
                return False
        else:
            coords = params['coordinates']

        # Wait for Nav2
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 not available')
            return False

        # Create goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(coords[0])
        goal.pose.pose.position.y = float(coords[1])

        # Convert theta to quaternion (simplified)
        theta = float(coords[2])
        goal.pose.pose.orientation.z = math.sin(theta / 2)
        goal.pose.pose.orientation.w = math.cos(theta / 2)

        # Send goal
        self.get_logger().info(f'Navigating to {coords}')
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

        result = future.result()
        if result and result.accepted:
            self.get_logger().info('Navigation goal accepted')
            return True
        else:
            self.get_logger().error('Navigation goal rejected')
            return False

    def execute_gesture(self, params) -> bool:
        """Execute gesture (placeholder for actual gesture server)."""
        gesture = params.get('gesture_name', 'wave')
        duration = params.get('duration_sec', 3.0)

        self.get_logger().info(f'Performing gesture: {gesture} for {duration}s')

        # In real implementation, call gesture action server
        # For now, just wait
        import time
        time.sleep(duration)

        return True

    def execute_wait(self, params) -> bool:
        """Wait for specified duration."""
        duration = params.get('duration_sec', 1.0)
        import time
        time.sleep(duration)
        return True

    def execute_speak(self, params) -> bool:
        """Speak text (placeholder for TTS)."""
        text = params.get('text', '')
        self.get_logger().info(f'Speaking: "{text}"')
        # In real implementation, call TTS service
        return True

    def publish_status(self, status: str, message: str):
        """Publish execution status."""
        msg = String()
        msg.data = json.dumps({'status': status, 'message': message})
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Expected Output**:
```
[INFO] [action_executor]: Action Executor ready
[INFO] [action_executor]: Executing plan: Navigate to kitchen and wave
[INFO] [action_executor]: Executing action 1/2: navigate
[INFO] [action_executor]: Navigating to (5.0, 2.0, 1.57)
[INFO] [action_executor]: Navigation goal accepted
[INFO] [action_executor]: Executing action 2/2: gesture
[INFO] [action_executor]: Performing gesture: wave for 3.0s
```

## Safety Considerations

LLM-generated plans require safety validation before execution.

### Key Safety Challenges

1. **Impossible Commands**: "Fly to the moon"
2. **Dangerous Actions**: "Run into the wall at full speed"
3. **Missing Context**: "Pick up the thing" (which thing?)
4. **Hallucinated Capabilities**: Actions the robot can't perform

### Safety Validation Layers

| Layer | Check | Response |
|-------|-------|----------|
| **Prompt** | Capability constraints | LLM won't generate invalid actions |
| **Parser** | Schema validation | Reject malformed plans |
| **Executor** | Precondition check | Verify action is possible |
| **Runtime** | Sensor feedback | Abort if unsafe condition detected |

### Implementing Safety Checks

```python
def validate_plan_safety(plan: ActionPlan) -> List[str]:
    """Check plan for safety issues."""
    issues = []

    for i, action in enumerate(plan.actions):
        # Check for unknown locations
        if action.action_type == ActionType.NAVIGATE:
            target = action.parameters.get('target')
            if target and target not in KNOWN_LOCATIONS:
                issues.append(f"Action {i+1}: Unknown location '{target}'")

        # Check for dangerous velocity
        if action.action_type == ActionType.NAVIGATE:
            coords = action.parameters.get('coordinates', [])
            if len(coords) >= 2:
                distance = math.sqrt(coords[0]**2 + coords[1]**2)
                if distance > 20.0:  # Max range
                    issues.append(f"Action {i+1}: Target too far ({distance:.1f}m)")

    return issues
```

:::warning LLM Safety Best Practices
1. **Never trust LLM output blindly** - always validate
2. **Limit capabilities** in the system prompt
3. **Log all commands** for audit
4. **Implement emergency stop** that bypasses LLM
5. **Test edge cases** thoroughly
:::

## Key Takeaways

1. **LLMs as Planners**: LLMs excel at converting natural language to structured action sequences

2. **Prompt Engineering**: The system prompt defines capabilities, output format, and safety rules

3. **JSON Output**: Use strict JSON schemas for reliable parsing

4. **Action Primitives**: Define a fixed set of actions the robot can perform:
   - `navigate`, `gesture`, `pick`, `place`, `speak`, `wait`

5. **Validation Pipeline**: Parse → Validate → Execute with checks at each stage

6. **Three Key Challenges in LLM-Robotics**:
   - **Consistency**: LLMs can produce different outputs for same input
   - **Grounding**: LLMs don't have access to real-world sensor state
   - **Safety**: Generated actions must be validated before execution

---

**Next Chapter**: [Capstone: Voice-Controlled Humanoid](./03-capstone-voice-humanoid.md) - Integrate everything into a complete voice-controlled robot system.
