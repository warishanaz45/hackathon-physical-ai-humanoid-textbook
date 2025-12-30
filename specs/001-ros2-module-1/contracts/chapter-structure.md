# Content Contract: Module 1 Chapters

**Feature**: `001-ros2-module-1`
**Date**: 2025-12-29
**Purpose**: Define the expected structure and content for each chapter

---

## Chapter 1: Introduction to ROS 2 for Physical AI

**File**: `book/docs/module-1-ros2/01-introduction.md`
**Reading Time**: ~30 minutes
**Prerequisites**: None (entry point)

### Required Sections

```markdown
---
sidebar_position: 1
title: "Introduction to ROS 2 for Physical AI"
description: "Learn what ROS 2 is, why it's the nervous system for humanoid robots, and how DDS enables real-time communication"
tags: [ros2, introduction, dds, humanoid]
---

## What You'll Learn
- What ROS 2 is and its role in robotics
- Why ROS 2 matters for humanoid robots
- Core DDS concepts: publish-subscribe, QoS, discovery
- Key differences between ROS 1 and ROS 2

## The Nervous System Analogy
[Explain how ROS 2 coordinates sensors, actuators, and decision-making like a nervous system]

## What is ROS 2?
[Definition accessible to AI/ML developers]

## Why ROS 2 for Humanoid Robots?
- Real-time requirements
- Multi-joint coordination
- Sensor fusion
- Distributed systems

## DDS: The Foundation
### Publish-Subscribe Pattern
[Explanation with diagram]

### Quality of Service (QoS)
[Reliability, durability, deadline]

### Automatic Discovery
[No roscore needed]

## ROS 1 vs ROS 2
| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Real-time | Limited | Native support |
| Security | None | DDS Security |
| Multi-robot | Complex | Built-in |
| Master node | Required (roscore) | None needed |

## Key Takeaways
- ROS 2 is middleware, not an operating system
- DDS provides reliable, real-time communication
- No central master means better fault tolerance
- Designed for production robotics

## Next Steps
[Link to Chapter 2: Communication Model]
```

---

## Chapter 2: ROS 2 Communication Model

**File**: `book/docs/module-1-ros2/02-communication.md`
**Reading Time**: ~60 minutes (includes code execution)
**Prerequisites**: Chapter 1 concepts

### Required Sections

```markdown
---
sidebar_position: 2
title: "ROS 2 Communication Model"
description: "Master nodes, topics, and services in ROS 2 with hands-on rclpy examples for agent controller patterns"
tags: [ros2, nodes, topics, services, rclpy, communication]
---

## What You'll Learn
- How nodes work as computational units
- Topics for asynchronous publish-subscribe
- Services for synchronous request-response
- Building a basic agent controller with rclpy

## ROS 2 Nodes
[Definition and purpose]

### Creating a Node
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
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected Output**:
```
[INFO] [minimal_node]: Hello from ROS 2!
```

## Topics: Publish-Subscribe
[Explanation of async, many-to-many]

### Publisher Example
```python title="publisher_node.py"
# [Complete publisher code]
```

### Subscriber Example
```python title="subscriber_node.py"
# [Complete subscriber code]
```

**Expected Output**:
```
[INFO] [publisher]: Publishing: 'Hello World: 0'
[INFO] [subscriber]: Received: 'Hello World: 0'
```

## Services: Request-Response
[Explanation of sync, one-to-one]

### When to Use Topics vs Services
| Use Case | Topics | Services |
|----------|--------|----------|
| Streaming sensor data | ✅ | ❌ |
| One-time configuration | ❌ | ✅ |
| Continuous state | ✅ | ❌ |
| Query with response | ❌ | ✅ |

## Agent Controller Pattern
[Receive sensor data → Make decision → Send command]

```python title="agent_controller.py"
# [Complete agent controller example]
```

## Key Takeaways
- Nodes are the building blocks
- Topics for continuous data streams
- Services for request-response patterns
- rclpy provides Python bindings

## Next Steps
[Link to Chapter 3: URDF]
```

---

## Chapter 3: Robot Structure with URDF

**File**: `book/docs/module-1-ros2/03-urdf.md`
**Reading Time**: ~45 minutes (includes visualization)
**Prerequisites**: Chapters 1-2 concepts

### Required Sections

```markdown
---
sidebar_position: 3
title: "Robot Structure with URDF"
description: "Create humanoid robot descriptions with URDF for simulation in Gazebo and visualization in RViz"
tags: [ros2, urdf, humanoid, simulation, rviz, gazebo]
---

## What You'll Learn
- What URDF is and why it matters
- Links and joints in robot descriptions
- Creating a simple humanoid URDF
- Visualizing in RViz
- Simulation readiness with Gazebo

## What is URDF?
[Unified Robot Description Format - XML format]

## Links: The Rigid Bodies
```xml title="link_example.urdf"
<link name="torso">
  <visual>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </visual>
</link>
```

## Joints: Connecting Links
```xml title="joint_example.urdf"
<joint name="shoulder" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57"/>
</joint>
```

### Joint Types
| Type | Motion | Example |
|------|--------|---------|
| fixed | None | Head to neck |
| revolute | Rotation with limits | Elbow |
| continuous | Unlimited rotation | Wheel |
| prismatic | Linear sliding | Gripper |

## Simple Humanoid Example
```xml title="humanoid.urdf"
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base/Torso -->
  <link name="base_link">...</link>

  <!-- Head -->
  <link name="head">...</link>
  <joint name="neck" type="fixed">...</joint>

  <!-- Arms (simplified) -->
  <link name="left_arm">...</link>
  <joint name="left_shoulder" type="revolute">...</joint>

  <!-- Legs (simplified) -->
  <link name="left_leg">...</link>
  <joint name="left_hip" type="revolute">...</joint>
</robot>
```

## Visualizing in RViz
```bash
ros2 launch urdf_tutorial display.launch.py model:=humanoid.urdf
```

**Expected Output**:
[Screenshot placeholder showing RViz with humanoid model]

## Simulation Readiness
[Brief intro to Gazebo integration - collision, inertia]

## Key Takeaways
- URDF is XML describing robot structure
- Links are rigid bodies with visual/collision geometry
- Joints connect links with kinematic constraints
- RViz for visualization, Gazebo for simulation

## Next Steps
[Preview of Module 2 or related resources]
```

---

## Validation Checklist

For each chapter:

- [ ] Frontmatter includes all required fields
- [ ] "What You'll Learn" section present
- [ ] "Key Takeaways" section present
- [ ] All code blocks have `title` attribute
- [ ] All code blocks have expected output
- [ ] All tables are properly formatted
- [ ] Internal links use relative paths
- [ ] Technical terms defined on first use
- [ ] Reading time estimate is reasonable
