---
sidebar_position: 3
title: "URDF: Describing Robot Structure"
description: "Learn to define robot links, joints, and create a humanoid robot description for visualization and simulation"
keywords: [URDF, robot description, links, joints, RViz, humanoid robot]
---

# URDF: Describing Robot Structure

## What You'll Learn

By the end of this chapter, you will be able to:

- Explain what URDF is and why it's essential for robotics
- Define robot links with visual and collision geometry
- Connect links with different joint types
- Create a simple humanoid robot description
- Visualize your robot in RViz
- Understand how URDF connects to simulation

---

## What is URDF?

**URDF** (Unified Robot Description Format) is an XML format for describing a robot's physical structure. It defines:

- **Links**: The rigid bodies (parts) of the robot
- **Joints**: The connections between links that allow movement
- **Visual geometry**: What the robot looks like
- **Collision geometry**: Simplified shapes for physics simulation
- **Inertial properties**: Mass and inertia for dynamics

### Why URDF Matters

```text
URDF File → RViz Visualization
          → Gazebo Simulation
          → Motion Planning (MoveIt)
          → State Publisher (TF transforms)
```

A single URDF file enables your robot to work with the entire ROS 2 ecosystem.

---

## Links: The Rigid Bodies

A **link** represents a rigid body in your robot. Each link can have:

- **Visual**: How it appears in visualizations (meshes, colors)
- **Collision**: Simplified geometry for physics calculations
- **Inertial**: Mass and moment of inertia

### Basic Link Structure

```xml title="link_example.urdf"
<link name="torso">
  <!-- Visual: What you see in RViz -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
    <material name="blue">
      <color rgba="0.0 0.0 0.8 1.0"/>
    </material>
  </visual>

  <!-- Collision: Used for physics simulation -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </collision>

  <!-- Inertial: Mass properties for dynamics -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="10.0"/>
    <inertia ixx="0.1" ixy="0" ixz="0"
             iyy="0.1" iyz="0"
             izz="0.1"/>
  </inertial>
</link>
```

### Geometry Types

URDF supports several primitive shapes:

| Shape | XML Element | Parameters |
|-------|-------------|------------|
| Box | `<box size="x y z"/>` | Width, depth, height |
| Cylinder | `<cylinder radius="r" length="l"/>` | Radius, length |
| Sphere | `<sphere radius="r"/>` | Radius |
| Mesh | `<mesh filename="path.stl"/>` | External mesh file |

### Origin and Orientation

The `<origin>` tag positions geometry relative to the link frame:

- `xyz`: Translation in meters (x, y, z)
- `rpy`: Rotation in radians (roll, pitch, yaw)

```xml
<!-- Offset the visual 0.1m up and rotate 90° around Z -->
<origin xyz="0 0 0.1" rpy="0 0 1.5708"/>
```

---

## Joints: Connecting Links

A **joint** connects two links and defines how they can move relative to each other.

### Joint Structure

```xml title="joint_example.urdf"
<joint name="shoulder_pitch" type="revolute">
  <!-- Parent link (fixed reference) -->
  <parent link="torso"/>

  <!-- Child link (moves relative to parent) -->
  <child link="upper_arm"/>

  <!-- Where the joint is located (relative to parent) -->
  <origin xyz="0.15 0 0.2" rpy="0 0 0"/>

  <!-- Axis of rotation/translation -->
  <axis xyz="0 1 0"/>

  <!-- Joint limits (for revolute/prismatic) -->
  <limit lower="-1.57" upper="1.57"
         effort="100" velocity="1.0"/>
</joint>
```

### Joint Types

| Type | Motion | Use Case |
|------|--------|----------|
| **revolute** | Rotation with limits | Elbows, knees, fingers |
| **continuous** | Unlimited rotation | Wheels, spinning joints |
| **prismatic** | Linear translation | Telescoping, linear actuators |
| **fixed** | No motion | Rigidly attached parts |
| **floating** | 6-DOF (free) | Base link of mobile robots |
| **planar** | 2D translation + rotation | Rarely used |

### Joint Parameters

- **axis**: Direction of motion (unit vector)
- **limit**: Motion constraints
  - `lower/upper`: Position limits (radians or meters)
  - `effort`: Maximum force/torque (N or Nm)
  - `velocity`: Maximum speed (rad/s or m/s)

---

## Joint Types Comparison

| Joint Type | Degrees of Freedom | Typical Use |
|------------|-------------------|-------------|
| `revolute` | 1 (rotation, limited) | Arm joints, leg joints |
| `continuous` | 1 (rotation, unlimited) | Wheels, wrists |
| `prismatic` | 1 (translation) | Grippers, linear slides |
| `fixed` | 0 | Sensor mounts, rigid connections |
| `floating` | 6 | Mobile robot base |
| `planar` | 3 | Planar mechanisms |

---

## Simple Humanoid Example

Here's a complete URDF for a simple humanoid upper body:

```xml title="humanoid.urdf"
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- ========== MATERIALS ========== -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="skin">
    <color rgba="0.96 0.80 0.69 1.0"/>
  </material>

  <!-- ========== BASE LINK ========== -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.01 0.01 0.01"/>
      </geometry>
    </visual>
  </link>

  <!-- ========== TORSO ========== -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.30 0.15 0.40"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.30 0.15 0.40"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="15.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0"
               iyy="0.5" iyz="0" izz="0.3"/>
    </inertial>
  </link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
  </joint>

  <!-- ========== HEAD ========== -->
  <link name="head">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.10"/>
      </geometry>
      <material name="skin"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.10"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="4.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0"
               iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="50" velocity="2.0"/>
  </joint>

  <!-- ========== RIGHT ARM ========== -->
  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 -0.12" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.24"/>
      </geometry>
      <material name="skin"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.12" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.24"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="right_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.18 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="100" velocity="3.0"/>
  </joint>

  <link name="right_forearm">
    <visual>
      <origin xyz="0 0 -0.11" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.22"/>
      </geometry>
      <material name="skin"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.11" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.22"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.008" ixy="0" ixz="0"
               iyy="0.008" iyz="0" izz="0.003"/>
    </inertial>
  </link>

  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 -0.24" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="80" velocity="3.0"/>
  </joint>

  <!-- ========== LEFT ARM ========== -->
  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.12" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.24"/>
      </geometry>
      <material name="skin"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.12" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.24"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.18 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="100" velocity="3.0"/>
  </joint>

  <link name="left_forearm">
    <visual>
      <origin xyz="0 0 -0.11" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.22"/>
      </geometry>
      <material name="skin"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.11" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.22"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.008" ixy="0" ixz="0"
               iyy="0.008" iyz="0" izz="0.003"/>
    </inertial>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.24" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="80" velocity="3.0"/>
  </joint>

</robot>
```

### URDF Structure Summary

```text
simple_humanoid
├── base_link (fixed reference)
│   └── torso (fixed joint)
│       ├── head (revolute: neck_joint)
│       ├── right_upper_arm (revolute: right_shoulder_pitch)
│       │   └── right_forearm (revolute: right_elbow)
│       └── left_upper_arm (revolute: left_shoulder_pitch)
│           └── left_forearm (revolute: left_elbow)
```

---

## Visualizing in RViz

To visualize your URDF in RViz, you need a launch file that:

1. Loads the URDF as a robot description
2. Starts the robot state publisher
3. Launches RViz with appropriate settings

### Launch File

```python title="launch/display.launch.py"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('your_robot_description'),
        'urdf',
        'humanoid.urdf'
    )

    # Read URDF content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint state publisher GUI (for moving joints manually)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('your_robot_description'),
            'rviz',
            'display.rviz'
        )]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
```

### Running the Visualization

```bash
# Build your package
colcon build --packages-select your_robot_description

# Source the workspace
source install/setup.bash

# Launch visualization
ros2 launch your_robot_description display.launch.py
```

### What You'll See

1. **RViz window** showing your robot model
2. **Joint State Publisher GUI** with sliders to move each joint
3. **TF frames** showing the coordinate frames for each link

---

## Simulation Readiness

URDF files work directly with Gazebo for physics simulation. To make your robot simulation-ready:

### 1. Add Gazebo Plugins

```xml
<!-- Add to your URDF for Gazebo integration -->
<gazebo>
  <plugin filename="libgazebo_ros2_control.so"
          name="gazebo_ros2_control">
    <robot_param>robot_description</robot_param>
    <robot_param_node>robot_state_publisher</robot_param_node>
  </plugin>
</gazebo>
```

### 2. Define Transmissions

```xml
<transmission name="right_shoulder_transmission">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="right_shoulder_pitch">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="right_shoulder_motor">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### 3. Add Sensor Plugins

```xml
<!-- IMU sensor -->
<gazebo reference="torso">
  <sensor type="imu" name="torso_imu">
    <update_rate>100</update_rate>
    <plugin filename="libgazebo_ros_imu_sensor.so"
            name="torso_imu_plugin">
      <ros>
        <remapping>~/out:=/imu/data</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

---

## Key Takeaways

1. **URDF defines robot structure** using links (rigid bodies) and joints (connections)
2. **Links have visual, collision, and inertial properties** for rendering and physics
3. **Joint types** determine how links can move relative to each other
4. **revolute joints** (limited rotation) are most common for humanoid limbs
5. **RViz visualizes URDF** with the robot state publisher
6. **URDF extends to Gazebo** with plugins for controllers and sensors

---

## Next Steps

Congratulations! You've completed Module 1: The Robotic Nervous System.

You now understand:
- ROS 2 fundamentals and DDS communication
- Nodes, topics, and services in Python
- Robot description with URDF

**In Module 2**, you'll learn about:
- Motion planning with MoveIt 2
- Inverse kinematics for humanoid arms
- Trajectory execution and feedback

Continue your journey with Module 2: Motion Planning (coming soon).
