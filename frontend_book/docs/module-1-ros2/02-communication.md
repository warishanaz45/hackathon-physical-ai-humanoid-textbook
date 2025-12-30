---
sidebar_position: 2
title: "ROS 2 Communication Patterns"
description: "Master ROS 2 nodes, topics, and services with practical Python examples"
keywords: [ROS 2, nodes, topics, services, rclpy, publish-subscribe]
---

# ROS 2 Communication Patterns

## What You'll Learn

By the end of this chapter, you will be able to:

- Create and run ROS 2 nodes using Python (rclpy)
- Publish messages to topics and subscribe to receive them
- Implement services for request-response communication
- Choose between topics and services for different scenarios
- Build an agent controller pattern for AI integration

---

## ROS 2 Nodes

A **node** is the fundamental building block of ROS 2 applications. Each node is an independent process that performs a specific task:

- **Sensor nodes**: Read data from cameras, IMUs, encoders
- **Controller nodes**: Compute motor commands
- **Planner nodes**: Generate paths and trajectories
- **Agent nodes**: Make high-level decisions using AI

### Minimal Node Example

Here's the simplest possible ROS 2 node in Python:

```python title="minimal_node.py"
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from ROS 2!')

def main():
    rclpy.init()
    node = MinimalNode()
    rclpy.spin(node)  # Keep node running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output:**

```text
[INFO] [minimal_node]: Hello from ROS 2!
```

### Key Concepts

- `rclpy.init()` — Initialize the ROS 2 Python client library
- `Node('name')` — Create a node with a unique name
- `rclpy.spin(node)` — Keep the node running and processing callbacks
- `rclpy.shutdown()` — Clean up resources when done

---

## Topics: Publish-Subscribe Communication

Topics enable **one-to-many** or **many-to-many** communication. Publishers send messages without knowing who's listening; subscribers receive without knowing who's sending.

### Publisher Node

This node publishes joint position commands at 10 Hz:

```python title="publisher_node.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')

        # Create publisher on '/joint_commands' topic
        self.publisher = self.create_publisher(
            Float64,           # Message type
            '/joint_commands', # Topic name
            10                 # Queue size
        )

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_command)
        self.position = 0.0

    def publish_command(self):
        msg = Float64()
        msg.data = self.position
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data:.2f}')
        self.position += 0.1  # Increment position

def main():
    rclpy.init()
    node = JointCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output:**

```text
[INFO] [joint_command_publisher]: Published: 0.00
[INFO] [joint_command_publisher]: Published: 0.10
[INFO] [joint_command_publisher]: Published: 0.20
...
```

### Subscriber Node

This node receives and processes the joint commands:

```python title="subscriber_node.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class JointCommandSubscriber(Node):
    def __init__(self):
        super().__init__('joint_command_subscriber')

        # Create subscription to '/joint_commands' topic
        self.subscription = self.create_subscription(
            Float64,               # Message type
            '/joint_commands',     # Topic name
            self.command_callback, # Callback function
            10                     # Queue size
        )

    def command_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data:.2f}')
        # Process the command (e.g., send to motor)

def main():
    rclpy.init()
    node = JointCommandSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output (when publisher is running):**

```text
[INFO] [joint_command_subscriber]: Received command: 0.00
[INFO] [joint_command_subscriber]: Received command: 0.10
[INFO] [joint_command_subscriber]: Received command: 0.20
...
```

### Running Publisher and Subscriber

Open two terminals and run:

```bash
# Terminal 1: Start the publisher
ros2 run your_package publisher_node

# Terminal 2: Start the subscriber
ros2 run your_package subscriber_node
```

You can also inspect topics from the command line:

```bash
# List all active topics
ros2 topic list

# See messages on a topic
ros2 topic echo /joint_commands

# Check topic info (type, publishers, subscribers)
ros2 topic info /joint_commands
```

---

## Services: Request-Response Communication

Services provide **synchronous** communication for operations that need a response. A client sends a request and waits for the server to return a response.

### When to Use Services vs Topics

| Scenario | Use Topics | Use Services |
|----------|------------|--------------|
| Continuous sensor data | ✅ | ❌ |
| Motor commands at fixed rate | ✅ | ❌ |
| Get robot configuration | ❌ | ✅ |
| Trigger a one-time action | ❌ | ✅ |
| Query current state | ❌ | ✅ |

### Service Definition

Services have a **request** and **response** structure. Here's a custom service for getting joint positions:

```text title="srv/GetJointPosition.srv"
string joint_name
---
float64 position
bool success
string message
```

### Service Server

```python title="joint_position_server.py"
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool  # Using built-in service type

class JointServer(Node):
    def __init__(self):
        super().__init__('joint_server')

        # Create service
        self.srv = self.create_service(
            SetBool,                 # Service type
            '/enable_motors',        # Service name
            self.enable_callback     # Handler function
        )
        self.motors_enabled = False
        self.get_logger().info('Joint server ready')

    def enable_callback(self, request, response):
        self.motors_enabled = request.data
        response.success = True
        response.message = f'Motors {"enabled" if request.data else "disabled"}'
        self.get_logger().info(response.message)
        return response

def main():
    rclpy.init()
    node = JointServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client

```python title="joint_position_client.py"
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class JointClient(Node):
    def __init__(self):
        super().__init__('joint_client')

        # Create client
        self.client = self.create_client(SetBool, '/enable_motors')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def enable_motors(self, enable: bool):
        request = SetBool.Request()
        request.data = enable

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()

def main():
    rclpy.init()
    client = JointClient()

    # Enable motors
    result = client.enable_motors(True)
    client.get_logger().info(f'Result: {result.message}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output:**

```text
# Server terminal:
[INFO] [joint_server]: Joint server ready
[INFO] [joint_server]: Motors enabled

# Client terminal:
[INFO] [joint_client]: Result: Motors enabled
```

---

## When to Use Topics vs Services

| Criteria | Topics | Services |
|----------|--------|----------|
| **Communication pattern** | One-to-many, continuous | One-to-one, request-response |
| **Timing** | Asynchronous | Synchronous (blocking) |
| **Data flow** | Streaming | On-demand |
| **Typical frequency** | High (10-1000 Hz) | Low (on request) |
| **Failure handling** | Messages may be dropped | Timeout and retry possible |

### Common Patterns in Humanoid Robots

| Data Type | Pattern | Typical Rate |
|-----------|---------|--------------|
| Joint positions | Topic | 100-1000 Hz |
| Camera images | Topic | 30-60 Hz |
| IMU readings | Topic | 100-400 Hz |
| Enable/disable motors | Service | On demand |
| Get robot state | Service | On demand |
| Trigger calibration | Service | One-time |
| Set control mode | Service | On demand |

---

## Agent Controller Pattern

AI agents that control humanoid robots need to:

1. **Receive observations** (sensor data via topics)
2. **Query state** (robot configuration via services)
3. **Send commands** (motor commands via topics or actions)

Here's a pattern that integrates with AI agent frameworks:

```python title="agent_controller.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from example_interfaces.srv import SetBool

class AgentController(Node):
    """
    Bridge between AI agent and ROS 2 robot.

    Observations: Subscribes to sensor topics
    Actions: Publishes commands to motor topics
    Services: Queries and configures robot state
    """

    def __init__(self):
        super().__init__('agent_controller')

        # Store latest observations for agent
        self.latest_joint_state = None

        # Subscribe to robot state (observations)
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publish commands (actions)
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )

        # Service client for motor control
        self.enable_client = self.create_client(
            SetBool,
            '/enable_motors'
        )

        self.get_logger().info('Agent controller initialized')

    def joint_state_callback(self, msg):
        """Store latest joint state for agent to query."""
        self.latest_joint_state = {
            'names': list(msg.name),
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'efforts': list(msg.effort)
        }

    def get_observation(self) -> dict:
        """
        Called by AI agent to get current robot state.

        Returns:
            Dictionary with joint positions, velocities, etc.
        """
        if self.latest_joint_state is None:
            return {'error': 'No joint state received yet'}
        return self.latest_joint_state

    def send_action(self, joint_positions: list):
        """
        Called by AI agent to send motor commands.

        Args:
            joint_positions: List of target joint positions
        """
        msg = Float64MultiArray()
        msg.data = joint_positions
        self.cmd_pub.publish(msg)
        self.get_logger().debug(f'Sent action: {joint_positions}')

    def enable_robot(self, enable: bool) -> bool:
        """
        Enable or disable robot motors.

        Args:
            enable: True to enable, False to disable

        Returns:
            True if successful
        """
        if not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Enable service not available')
            return False

        request = SetBool.Request()
        request.data = enable
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success

def main():
    rclpy.init()
    controller = AgentController()

    # Example: Enable robot
    if controller.enable_robot(True):
        controller.get_logger().info('Robot enabled')

    # Example: Run control loop
    rate = controller.create_rate(10)  # 10 Hz
    try:
        while rclpy.ok():
            rclpy.spin_once(controller)

            # Get observation for AI agent
            obs = controller.get_observation()

            # AI agent would compute action here
            # action = agent.compute_action(obs)

            # Send action to robot
            # controller.send_action(action)

            rate.sleep()
    except KeyboardInterrupt:
        pass

    controller.enable_robot(False)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Integration with AI Frameworks

This controller pattern can be integrated with:

- **OpenAI Gym/Gymnasium**: Wrap as a custom environment
- **LangChain agents**: Expose as tools the agent can call
- **Custom RL frameworks**: Use `get_observation()` and `send_action()` in training loop

---

## Key Takeaways

1. **Nodes are independent processes** that communicate via topics and services
2. **Topics use publish-subscribe** for continuous, one-to-many data streams
3. **Services use request-response** for on-demand queries and commands
4. **Choose topics** for sensor data and motor commands at fixed rates
5. **Choose services** for configuration, state queries, and triggered actions
6. **The agent controller pattern** bridges AI agents with ROS 2 robots

---

## Next Steps

In the next chapter, [URDF: Describing Robot Structure](./03-urdf.md), you'll learn how to:

- Define robot links and joints in URDF format
- Create a simple humanoid robot description
- Visualize your robot in RViz
- Prepare for simulation in Gazebo
