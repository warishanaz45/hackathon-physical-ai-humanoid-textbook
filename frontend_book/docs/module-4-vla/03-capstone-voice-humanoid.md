---
sidebar_position: 3
title: "Capstone: Voice-Controlled Humanoid"
description: "Build an end-to-end voice-controlled humanoid robot system integrating speech, LLM planning, and ROS 2 execution"
keywords: [capstone, humanoid, voice, vla, integration, ros2, nav2, gesture]
---

# Capstone: Voice-Controlled Humanoid

## What You'll Learn

- Integrate voice input, LLM planning, and robot execution into a complete system
- Configure the VLA pipeline via ROS 2 launch files
- Execute navigation commands through voice
- Implement gesture execution via joint controllers
- Demonstrate multi-step autonomous task execution
- Test and validate the complete system

:::info Prerequisites
This chapter requires:
- **Chapter 1**: Voice capture and Whisper transcription
- **Chapter 2**: LLM planner and action executor
- **Module 3**: Nav2 navigation stack configured

Ensure you have a working simulation environment with a humanoid robot.
:::

## Capstone Overview

This capstone project brings together everything you've learned to create a fully voice-controlled humanoid robot. By the end, you'll speak a command like *"Go to the table and wave hello"* and watch your robot execute it autonomously.

### What You'll Build

```
┌────────────────────────────────────────────────────────────┐
│                   VLA Capstone System                       │
├────────────────────────────────────────────────────────────┤
│                                                             │
│   🎤 Microphone                                             │
│        │                                                    │
│        ▼                                                    │
│   ┌─────────────┐    ┌─────────────┐    ┌─────────────┐   │
│   │  Whisper    │───▶│  LLM        │───▶│  Action     │   │
│   │  Node       │    │  Planner    │    │  Executor   │   │
│   └─────────────┘    └─────────────┘    └─────────────┘   │
│        │                   │                   │           │
│        ▼                   ▼                   ▼           │
│   /voice_command     /action_plan      ┌─────────────┐    │
│                                        │  Nav2       │    │
│                                        │  Gesture    │    │
│                                        │  Servers    │    │
│                                        └─────────────┘    │
│                                              │             │
│                                              ▼             │
│                                         🤖 Robot          │
└────────────────────────────────────────────────────────────┘
```

### End-to-End Pipeline

1. **Voice Capture**: Microphone → VAD → Audio buffer
2. **Transcription**: Whisper → Text command
3. **Planning**: LLM → JSON action plan
4. **Execution**: Action executor → Nav2 + Gestures
5. **Feedback**: Status updates → User

## Integrating the VLA Pipeline

Let's create a launch file that starts all components:

```python title="vla_pipeline.launch.py"
#!/usr/bin/env python3
"""Launch file for complete VLA pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Declare arguments
    whisper_model = DeclareLaunchArgument(
        'whisper_model',
        default_value='base',
        description='Whisper model size: tiny, base, small, medium, large'
    )

    openai_model = DeclareLaunchArgument(
        'openai_model',
        default_value='gpt-4o-mini',
        description='OpenAI model for planning'
    )

    # Voice command publisher (Whisper)
    whisper_node = Node(
        package='vla_pipeline',
        executable='voice_command_publisher',
        name='voice_command_publisher',
        parameters=[{
            'model_size': LaunchConfiguration('whisper_model'),
            'confidence_threshold': 0.7,
        }],
        output='screen'
    )

    # LLM Planner
    llm_planner = Node(
        package='vla_pipeline',
        executable='llm_planner_node',
        name='llm_planner',
        parameters=[{
            'model': LaunchConfiguration('openai_model'),
        }],
        output='screen'
    )

    # Action Executor
    action_executor = Node(
        package='vla_pipeline',
        executable='action_executor',
        name='action_executor',
        output='screen'
    )

    # Gesture Action Server
    gesture_server = Node(
        package='vla_pipeline',
        executable='gesture_action_server',
        name='gesture_server',
        output='screen'
    )

    # Status Monitor (optional)
    status_monitor = Node(
        package='vla_pipeline',
        executable='status_monitor',
        name='status_monitor',
        output='screen'
    )

    return LaunchDescription([
        whisper_model,
        openai_model,
        whisper_node,
        llm_planner,
        action_executor,
        gesture_server,
        status_monitor,
    ])
```

**Expected Output** (when launched):
```
[INFO] [launch]: All log files can be found below /home/user/.ros/log/
[INFO] [voice_command_publisher]: Loading Whisper base model...
[INFO] [voice_command_publisher]: Voice command publisher ready
[INFO] [llm_planner]: LLM Planner ready (model: gpt-4o-mini)
[INFO] [action_executor]: Action Executor ready
[INFO] [gesture_server]: Gesture Action Server ready
[INFO] [status_monitor]: Status Monitor ready
```

### Launch the Pipeline

```bash
# Terminal 1: Start simulation (from Module 3)
ros2 launch humanoid_sim gazebo.launch.py

# Terminal 2: Start Nav2 (from Module 3)
ros2 launch nav2_bringup navigation_launch.py

# Terminal 3: Start VLA pipeline
export OPENAI_API_KEY="your-key-here"
ros2 launch vla_pipeline vla_pipeline.launch.py
```

## Navigation via Voice Commands

With Nav2 running from Module 3, voice commands can trigger navigation:

### How Voice Becomes Navigation

1. User says: *"Go to the kitchen"*
2. Whisper transcribes → `"go to the kitchen"`
3. LLM plans → `{"action_type": "navigate", "parameters": {"target": "kitchen"}}`
4. Executor looks up kitchen coordinates → `(5.0, 2.0, 1.57)`
5. Sends goal to Nav2 → Robot moves!

### Testing Navigation

```bash
# Monitor navigation status
ros2 topic echo /execution_status

# Speak a command
# "Go to the living room"
# "Navigate to the front door"
# "Move to the table"
```

### Integration with Module 3 Nav2

The action executor from Chapter 2 uses the `NavigateToPose` action from Nav2:

```python
# From action_executor.py
from nav2_msgs.action import NavigateToPose

self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
```

This connects directly to the Nav2 stack you configured in Module 3, Chapter 3.

## Gesture Execution

Humanoids can do more than navigate—they can gesture! Let's implement a simple gesture action server:

```python title="gesture_action_server.py"
#!/usr/bin/env python3
"""Simple gesture action server for humanoid robot."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
import time

# Custom action definition would go here
# For simplicity, we'll use a service-like approach
from std_srvs.srv import Trigger
from std_msgs.msg import String


class GestureActionServer(Node):
    """Executes humanoid gestures."""

    # Gesture definitions (joint configurations)
    GESTURES = {
        'wave': {
            'description': 'Wave greeting with right arm',
            'duration': 3.0,
            'joints': {
                'right_shoulder_pitch': [-0.5, 0.0, -0.5, 0.0],  # keyframes
                'right_shoulder_roll': [0.3, 0.3, 0.3, 0.3],
                'right_elbow': [-0.5, -0.3, -0.5, -0.3],
            }
        },
        'nod': {
            'description': 'Nod head yes',
            'duration': 2.0,
            'joints': {
                'head_pitch': [0.2, -0.1, 0.2, 0.0],
            }
        },
        'point': {
            'description': 'Point forward with right arm',
            'duration': 2.0,
            'joints': {
                'right_shoulder_pitch': [-1.0],
                'right_elbow': [0.0],
            }
        },
        'shake_head': {
            'description': 'Shake head no',
            'duration': 2.0,
            'joints': {
                'head_yaw': [0.3, -0.3, 0.3, -0.3, 0.0],
            }
        }
    }

    def __init__(self):
        super().__init__('gesture_action_server')

        # Subscriber for gesture commands
        self.subscription = self.create_subscription(
            String,
            '/gesture_command',
            self.gesture_callback,
            10
        )

        # Publisher for joint commands (simplified)
        self.joint_pub = self.create_publisher(String, '/joint_commands', 10)

        # Status publisher
        self.status_pub = self.create_publisher(String, '/gesture_status', 10)

        self.get_logger().info('Gesture Action Server ready')
        self.get_logger().info(f'Available gestures: {list(self.GESTURES.keys())}')

    def gesture_callback(self, msg):
        """Handle gesture command."""
        import json
        try:
            data = json.loads(msg.data)
            gesture_name = data.get('gesture_name', '')
            duration = data.get('duration_sec', None)
        except json.JSONDecodeError:
            gesture_name = msg.data
            duration = None

        self.execute_gesture(gesture_name, duration)

    def execute_gesture(self, gesture_name: str, duration: float = None):
        """Execute a gesture by name."""
        gesture = self.GESTURES.get(gesture_name)

        if not gesture:
            self.get_logger().error(f'Unknown gesture: {gesture_name}')
            self.publish_status('error', f'Unknown gesture: {gesture_name}')
            return False

        actual_duration = duration or gesture['duration']

        self.get_logger().info(
            f"Executing gesture: {gesture_name} ({gesture['description']})"
        )
        self.publish_status('executing', f"Performing {gesture_name}")

        # In real implementation, send joint trajectories
        # Here we simulate the gesture duration
        self.simulate_gesture(gesture, actual_duration)

        self.get_logger().info(f'Gesture {gesture_name} complete')
        self.publish_status('completed', f'{gesture_name} finished')
        return True

    def simulate_gesture(self, gesture: dict, duration: float):
        """Simulate gesture execution."""
        joints = gesture['joints']
        num_keyframes = max(len(v) for v in joints.values())
        keyframe_duration = duration / num_keyframes

        for i in range(num_keyframes):
            # In real implementation:
            # - Interpolate joint positions
            # - Send to joint trajectory controller
            # - Wait for execution

            joint_positions = {}
            for joint, keyframes in joints.items():
                if i < len(keyframes):
                    joint_positions[joint] = keyframes[i]

            self.get_logger().debug(f'Keyframe {i+1}: {joint_positions}')
            time.sleep(keyframe_duration)

    def publish_status(self, status: str, message: str):
        """Publish gesture status."""
        import json
        msg = String()
        msg.data = json.dumps({'status': status, 'message': message})
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GestureActionServer()

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
[INFO] [gesture_action_server]: Gesture Action Server ready
[INFO] [gesture_action_server]: Available gestures: ['wave', 'nod', 'point', 'shake_head']
[INFO] [gesture_action_server]: Executing gesture: wave (Wave greeting with right arm)
[INFO] [gesture_action_server]: Gesture wave complete
```

## Multi-Step Command Demo

Now let's test the complete system with a multi-step command:

```python title="capstone_demo.py"
#!/usr/bin/env python3
"""Capstone demonstration script."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class CapstoneDemoNode(Node):
    """Demonstrates complete VLA pipeline with sample commands."""

    DEMO_COMMANDS = [
        "Go to the table and wave hello",
        "Navigate to the kitchen",
        "Go to the front door and nod",
        "Move to the living room and point forward",
    ]

    def __init__(self):
        super().__init__('capstone_demo')

        # Publishers
        self.voice_pub = self.create_publisher(String, '/voice_command', 10)

        # Subscribers for status
        self.create_subscription(
            String, '/action_plan', self.plan_callback, 10
        )
        self.create_subscription(
            String, '/execution_status', self.status_callback, 10
        )

        self.current_plan = None
        self.execution_complete = False

        self.get_logger().info('Capstone Demo Node ready')

    def plan_callback(self, msg):
        """Log received action plan."""
        plan = json.loads(msg.data)
        self.current_plan = plan
        actions = plan.get('actions', [])
        self.get_logger().info(f"Plan received: {len(actions)} actions")
        for i, action in enumerate(actions):
            self.get_logger().info(
                f"  {i+1}. {action['action_type']}: {action.get('description', '')}"
            )

    def status_callback(self, msg):
        """Log execution status."""
        status = json.loads(msg.data)
        self.get_logger().info(f"Status: {status['status']} - {status['message']}")

        if status['status'] in ['completed', 'error']:
            self.execution_complete = True

    def send_command(self, command: str):
        """Send a voice command to the pipeline."""
        self.get_logger().info(f"\n{'='*50}")
        self.get_logger().info(f"DEMO COMMAND: '{command}'")
        self.get_logger().info('='*50)

        self.execution_complete = False

        msg = String()
        msg.data = command
        self.voice_pub.publish(msg)

    def run_demo(self):
        """Run through demo commands."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("  CAPSTONE DEMO: Voice-Controlled Humanoid")
        self.get_logger().info("="*60 + "\n")

        for i, command in enumerate(self.DEMO_COMMANDS):
            self.get_logger().info(f"\n--- Demo {i+1}/{len(self.DEMO_COMMANDS)} ---")
            self.send_command(command)

            # Wait for execution (with timeout)
            timeout = 30.0
            start = time.time()
            while not self.execution_complete and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)

            if not self.execution_complete:
                self.get_logger().warn("Execution timeout, moving to next command")

            time.sleep(2.0)  # Brief pause between commands

        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("  DEMO COMPLETE")
        self.get_logger().info("="*60)


def main(args=None):
    rclpy.init(args=args)
    node = CapstoneDemoNode()

    # Give other nodes time to start
    time.sleep(2.0)

    try:
        node.run_demo()
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
============================================================
  CAPSTONE DEMO: Voice-Controlled Humanoid
============================================================

--- Demo 1/4 ---
==================================================
DEMO COMMAND: 'Go to the table and wave hello'
==================================================
[INFO] Plan received: 2 actions
  1. navigate: Move to the table
  2. gesture: Wave greeting
[INFO] Status: executing - Action 1: Move to the table
[INFO] Status: executing - Action 2: Wave greeting
[INFO] Status: completed - All actions completed

--- Demo 2/4 ---
...

============================================================
  DEMO COMPLETE
============================================================
```

## Testing and Validation

### End-to-End Testing Approach

| Test | Input | Expected Output |
|------|-------|-----------------|
| Single navigate | "Go to kitchen" | Robot at kitchen location |
| Single gesture | "Wave hello" | Robot waves |
| Multi-step | "Go to table and nod" | Navigate then nod |
| Unknown location | "Go to the moon" | Error with warning |
| Unclear speech | (mumbling) | Request repeat |

### Common Issues and Fixes

| Issue | Symptom | Fix |
|-------|---------|-----|
| No audio | "Listening..." but nothing | Check `arecord -l`, mic permissions |
| Low confidence | Commands rejected | Speak clearly, reduce background noise |
| LLM timeout | Long delay, then error | Check internet, API key |
| Nav2 not ready | "Nav2 not available" | Ensure Nav2 launched first |
| Wrong location | Robot goes elsewhere | Verify LOCATIONS dict matches map |

### Performance Considerations

- **Whisper latency**: 0.5-2s depending on model size
- **LLM latency**: 0.5-3s for API calls
- **Navigation**: Depends on distance and obstacles
- **Total pipeline**: ~3-10s from speech to action start

## Key Takeaways

1. **Integration is Key**: The VLA pipeline requires careful coordination between components

2. **Launch Files**: Use ROS 2 launch to start all nodes together with proper configuration

3. **Status Monitoring**: Track execution status for debugging and user feedback

4. **Graceful Failures**: Handle errors at each stage without crashing the system

5. **Test Incrementally**: Verify each component before integration

6. **Real-World Considerations**:
   - Network latency affects LLM calls
   - Audio quality impacts transcription
   - Environment affects navigation

## Where to Go from Here

### Advanced Topics

- **Vision Integration**: Add camera input for object recognition
- **Continuous Dialogue**: Multi-turn conversations with context
- **Local LLMs**: Run Llama or Mistral for offline operation
- **Emotion Recognition**: Adjust robot behavior based on user tone

### Real Hardware Deployment

1. **Audio**: Industrial microphones with noise cancellation
2. **Edge Computing**: NVIDIA Jetson for local inference
3. **Safety**: Emergency stop, obstacle detection, speed limits
4. **Testing**: Extensive real-world validation before deployment

### Further Reading

- [OpenAI Whisper Documentation](https://github.com/openai/whisper)
- [OpenAI API Reference](https://platform.openai.com/docs/api-reference)
- [ROS 2 Action Servers](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
- [Nav2 Documentation](https://navigation.ros.org/)
- [MoveIt 2 for Manipulation](https://moveit.ros.org/)

---

**Congratulations!** You've completed Module 4 and built a voice-controlled humanoid robot system. This capstone integrates speech recognition, language understanding, and robotic action—the core of modern human-robot interaction.

The skills you've learned—voice capture, LLM planning, action execution—form the foundation for building intelligent, conversational robots that can operate naturally in human environments.
