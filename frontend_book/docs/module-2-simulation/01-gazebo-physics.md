---
sidebar_position: 1
title: "Physics Simulation with Gazebo"
description: "Learn to set up Gazebo Fortress for physics-based humanoid robot simulation with ROS 2"
keywords: [Gazebo, physics simulation, ROS 2, SDF, robotics, Gazebo Fortress]
---

# Physics Simulation with Gazebo

## What You'll Learn

By the end of this chapter, you will be able to:

- Understand Gazebo Fortress architecture and its integration with ROS 2
- Explain the difference between SDF and URDF formats for simulation
- Create a simulation world file with a humanoid robot
- Configure physics engine parameters for stable simulation
- Control robot joints via ROS 2 topics and services
- Launch and interact with a Gazebo simulation

---

## Gazebo Architecture Overview

**Gazebo** is an open-source 3D robotics simulator that provides physics simulation, sensor simulation, and visualization. The modern version, **Gazebo Fortress** (part of the "Ignition" rebranding, now simply called "Gazebo"), is designed to work seamlessly with ROS 2.

### Core Components

```text
┌─────────────────────────────────────────────────────────────┐
│                     Gazebo Fortress                         │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │   Physics   │  │   Sensors   │  │    Rendering        │ │
│  │   Engine    │  │   Plugins   │  │    (Ogre2)          │ │
│  │ (DART/ODE)  │  │             │  │                     │ │
│  └─────────────┘  └─────────────┘  └─────────────────────┘ │
│                         │                                   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              Gazebo Transport (ign-transport)        │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                           │
                    ros_gz_bridge
                           │
┌─────────────────────────────────────────────────────────────┐
│                        ROS 2                                │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │   Your      │  │   RViz2     │  │   Control           │ │
│  │   Nodes     │  │             │  │   Nodes             │ │
│  └─────────────┘  └─────────────┘  └─────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### Gazebo Classic vs Gazebo Fortress

| Aspect | Gazebo Classic (gazebo11) | Gazebo Fortress |
|--------|---------------------------|-----------------|
| ROS Version | ROS 1 / ROS 2 (via wrapper) | Native ROS 2 support |
| Architecture | Monolithic | Modular (gz-* libraries) |
| Physics Engines | ODE, Bullet, DART, Simbody | DART, TPE, Bullet |
| Sensor Framework | Built-in | Plugin-based (gz-sensors) |
| Transport | Custom | ign-transport (DDS-like) |
| Rendering | Ogre 1.x | Ogre 2.x (modern) |
| Status | Legacy (maintenance only) | Active development |

**Recommendation**: Use Gazebo Fortress for all new ROS 2 Humble projects.

### When to Use Gazebo

| Use Case | Gazebo | Alternative |
|----------|--------|-------------|
| Physics validation | ✅ Best choice | - |
| Controller testing | ✅ Best choice | - |
| Multi-robot simulation | ✅ Excellent | - |
| High-fidelity visuals | ⚠️ Limited | Unity, Unreal |
| HRI with humans | ⚠️ Basic | Unity |
| Real-time performance | ✅ Good | - |

---

## SDF and URDF for Simulation

Gazebo uses **SDF (Simulation Description Format)** as its native format, but also supports **URDF** with extensions.

### SDF vs URDF

| Feature | URDF | SDF |
|---------|------|-----|
| Primary use | Robot description | World + robot description |
| Simulation-specific | Limited | Full support |
| Multiple robots | One per file | Multiple in world |
| Sensors | Via Gazebo tags | Native support |
| Physics properties | Via Gazebo tags | Native support |
| Worlds/environments | Not supported | Full support |

### Adding Gazebo Extensions to URDF

To use your URDF from Module 1 in Gazebo, add `<gazebo>` tags:

```xml title="humanoid_gazebo.urdf"
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Include your URDF links and joints from Module 1 -->

  <!-- ========== GAZEBO EXTENSIONS ========== -->

  <!-- Material colors for Gazebo (URDF materials don't transfer) -->
  <gazebo reference="torso">
    <material>Gazebo/Blue</material>
    <mu1>0.8</mu1>  <!-- Friction coefficient -->
    <mu2>0.8</mu2>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/Skin</material>
  </gazebo>

  <!-- Joint properties for simulation -->
  <gazebo reference="right_shoulder_pitch">
    <implicitSpringDamper>true</implicitSpringDamper>
    <provideFeedback>true</provideFeedback>
  </gazebo>

  <!-- Base link for fixed world attachment (optional) -->
  <gazebo reference="base_link">
    <static>false</static>
  </gazebo>

  <!-- Gazebo ROS 2 Control plugin -->
  <gazebo>
    <plugin filename="libgz_ros2_control-system.so"
            name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <robot_param>robot_description</robot_param>
      <robot_param_node>robot_state_publisher</robot_param_node>
      <parameters>$(find your_package)/config/controllers.yaml</parameters>
    </plugin>
  </gazebo>

</robot>
```

### Key Gazebo Tags

| Tag | Purpose | Example |
|-----|---------|---------|
| `<material>` | Visual appearance | `Gazebo/Blue` |
| `<mu1>`, `<mu2>` | Friction coefficients | `0.8` |
| `<kp>`, `<kd>` | Contact stiffness/damping | `1e6`, `1.0` |
| `<selfCollide>` | Enable self-collision | `true` |
| `<gravity>` | Enable/disable gravity | `true` |
| `<static>` | Fixed in world | `false` |

---

## Creating a Simulation World

A **world file** defines the complete simulation environment including ground, lighting, physics settings, and robot models.

```xml title="humanoid_world.sdf"
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="humanoid_world">

    <!-- ========== PHYSICS CONFIGURATION ========== -->
    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>  <!-- 1ms timestep -->
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <dart>
        <solver>
          <solver_type>pgs</solver_type>
          <collision_detector>bullet</collision_detector>
        </solver>
      </dart>
    </physics>

    <!-- ========== SCENE SETTINGS ========== -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- ========== LIGHTING ========== -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- ========== GROUND PLANE ========== -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.8</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- ========== ROBOT MODEL ========== -->
    <!-- Option 1: Include URDF via spawn_entity -->
    <!-- Robot will be spawned via ros_gz_sim create service -->

    <!-- Option 2: Include SDF model directly -->
    <!--
    <include>
      <uri>model://simple_humanoid</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>
    -->

    <!-- ========== ROS-GAZEBO BRIDGE PLUGIN ========== -->
    <plugin filename="libgz_sim_ros_init-system.so"
            name="gz::sim::systems::RosGzInit">
    </plugin>

  </world>
</sdf>
```

---

## Physics Configuration

Proper physics configuration is critical for stable humanoid simulation.

### Key Physics Parameters

| Parameter | Description | Typical Value | Impact |
|-----------|-------------|---------------|--------|
| `max_step_size` | Simulation timestep | 0.001s (1ms) | Lower = more stable, slower |
| `real_time_factor` | Speed relative to real time | 1.0 | Higher = faster, less accurate |
| `solver_iterations` | Constraint solver iterations | 50-100 | Higher = more accurate, slower |
| `sor` | Successive over-relaxation | 1.3 | Convergence speed |
| `contact_max_correcting_vel` | Max velocity correction | 100 | Stability vs accuracy |
| `contact_surface_layer` | Surface penetration allowance | 0.001 | Prevent jitter |

### Timestep Selection

```text
Timestep Trade-offs:

10ms (0.01s): Fast simulation, may be unstable for fast movements
 5ms (0.005s): Good balance for most robots
 1ms (0.001s): High accuracy, required for fast dynamics
0.1ms (0.0001s): Very accurate, very slow - use for validation only

Humanoid robots with walking: Use 1ms or smaller
Static poses and slow movements: 5ms is acceptable
```

### Physics Engine Comparison

| Engine | Accuracy | Speed | Best For |
|--------|----------|-------|----------|
| **DART** | High | Medium | Articulated robots, humanoids |
| **ODE** | Medium | Fast | General robotics |
| **Bullet** | Medium | Fast | Game-like physics, collision-heavy |
| **TPE** | Low | Very Fast | Large-scale, simplified physics |

**Recommendation**: Use DART for humanoid robots due to superior joint accuracy.

---

## Joint Control via ROS 2

Control your simulated robot using ROS 2 topics and services through the `ros_gz_bridge`.

### Launch File with Controllers

```python title="gazebo_launch.py"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Package paths
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_your_robot = get_package_share_directory('your_robot_description')

    # World file
    world_file = PathJoinSubstitution([
        pkg_your_robot, 'worlds', 'humanoid_world.sdf'
    ])

    # URDF file
    urdf_file = os.path.join(pkg_your_robot, 'urdf', 'humanoid_gazebo.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ]),
            launch_arguments={
                'gz_args': ['-r ', world_file],
            }.items(),
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'simple_humanoid',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.5',
            ],
            output='screen',
        ),

        # ROS-Gazebo Bridge for joint states
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/humanoid_world/model/simple_humanoid/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
                '/model/simple_humanoid/joint/right_shoulder_pitch/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/simple_humanoid/joint/left_shoulder_pitch/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/simple_humanoid/joint/right_elbow/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/simple_humanoid/joint/left_elbow/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double',
            ],
            output='screen',
        ),
    ])
```

### Joint Position Controller

```python title="joint_controller.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class JointController(Node):
    """
    Simple joint position controller for Gazebo simulation.
    Sends sinusoidal position commands to demonstrate joint control.
    """

    def __init__(self):
        super().__init__('joint_controller')

        # Publishers for joint position commands
        self.joint_publishers = {
            'right_shoulder': self.create_publisher(
                Float64,
                '/model/simple_humanoid/joint/right_shoulder_pitch/cmd_pos',
                10
            ),
            'left_shoulder': self.create_publisher(
                Float64,
                '/model/simple_humanoid/joint/left_shoulder_pitch/cmd_pos',
                10
            ),
            'right_elbow': self.create_publisher(
                Float64,
                '/model/simple_humanoid/joint/right_elbow/cmd_pos',
                10
            ),
            'left_elbow': self.create_publisher(
                Float64,
                '/model/simple_humanoid/joint/left_elbow/cmd_pos',
                10
            ),
        }

        # Timer for control loop at 50 Hz
        self.timer = self.create_timer(0.02, self.control_loop)
        self.time = 0.0

        self.get_logger().info('Joint controller started')

    def control_loop(self):
        """Send sinusoidal position commands to joints."""
        self.time += 0.02

        # Sinusoidal motion for arms (wave pattern)
        shoulder_angle = 0.5 * math.sin(self.time * 2.0)  # +/- 0.5 rad
        elbow_angle = 0.8 + 0.3 * math.sin(self.time * 2.0)  # 0.5 to 1.1 rad

        # Publish commands
        msg = Float64()

        # Right arm
        msg.data = shoulder_angle
        self.joint_publishers['right_shoulder'].publish(msg)
        msg.data = elbow_angle
        self.joint_publishers['right_elbow'].publish(msg)

        # Left arm (opposite phase)
        msg.data = -shoulder_angle
        self.joint_publishers['left_shoulder'].publish(msg)
        msg.data = elbow_angle
        self.joint_publishers['left_elbow'].publish(msg)

        # Log periodically
        if int(self.time * 10) % 20 == 0:
            self.get_logger().info(
                f'Shoulder: {shoulder_angle:.2f} rad, Elbow: {elbow_angle:.2f} rad'
            )

def main():
    rclpy.init()
    controller = JointController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output:**

```text
[INFO] [joint_controller]: Joint controller started
[INFO] [joint_controller]: Shoulder: 0.45 rad, Elbow: 0.95 rad
[INFO] [joint_controller]: Shoulder: -0.23 rad, Elbow: 0.65 rad
[INFO] [joint_controller]: Shoulder: -0.49 rad, Elbow: 0.52 rad
...
```

### Running the Simulation

```bash
# Terminal 1: Launch Gazebo with robot
ros2 launch your_robot_description gazebo_launch.py

# Terminal 2: Run joint controller
ros2 run your_robot_description joint_controller

# Terminal 3: Echo joint states
ros2 topic echo /joint_states
```

---

## Key Takeaways

1. **Gazebo Fortress** is the modern, ROS 2-native simulator — use it for all new projects
2. **SDF** is Gazebo's native format; extend URDF with `<gazebo>` tags for simulation
3. **Physics timestep** critically affects stability — use 1ms for humanoid robots
4. **DART physics engine** provides the best accuracy for articulated robots
5. **ros_gz_bridge** connects Gazebo topics to ROS 2 topics for control and sensing
6. **World files** define the complete environment: ground, lighting, physics, and models

---

## Next Steps

In the next chapter, [Digital Twins & HRI in Unity](./02-unity-digital-twins.md), you'll learn how to:

- Import your robot into Unity for high-fidelity visualization
- Connect Unity to ROS 2 for real-time control
- Create human-robot interaction scenarios
- Build realistic environments for training data generation
