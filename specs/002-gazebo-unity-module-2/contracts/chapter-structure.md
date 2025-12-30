# Chapter Structure Contract: Module 2

**Feature**: 002-gazebo-unity-module-2
**Date**: 2025-12-29

## Chapter 1: Physics Simulation with Gazebo

### Required Sections

1. **What You'll Learn** (bullet list, 4-6 items)
   - Gazebo Fortress architecture and ROS 2 integration
   - SDF format and URDF extensions for simulation
   - Physics engine configuration and tuning
   - Joint control via ROS 2 interfaces
   - World file creation and environment setup

2. **Gazebo Architecture Overview**
   - Component diagram: Gazebo + ROS 2 + ros_gz_bridge
   - Comparison with Gazebo Classic
   - When to use Gazebo vs other simulators

3. **SDF and URDF for Simulation**
   - SDF format introduction
   - Adding `<gazebo>` tags to URDF
   - Inertial, collision, and visual properties
   - Code example: URDF with Gazebo extensions

4. **Creating a Simulation World**
   - World file structure
   - Adding ground plane, lighting, physics
   - Including robot model
   - Code example: Complete world file

5. **Physics Configuration**
   - Timestep selection (1ms vs 10ms)
   - Solver iterations
   - Friction models
   - Table: Physics parameters and their effects

6. **Joint Control via ROS 2**
   - ros_gz_bridge configuration
   - Position, velocity, effort controllers
   - Code example: Launch file with controllers
   - Code example: Python joint command publisher

7. **Key Takeaways** (bullet list, 5-6 items)

8. **Next Steps** (link to Chapter 2)

### Code Examples Required

| Example | Language | Title | Has Output |
|---------|----------|-------|------------|
| URDF with Gazebo tags | xml | `humanoid_gazebo.urdf` | No |
| SDF world file | xml | `humanoid_world.sdf` | No |
| Launch file | python | `gazebo_launch.py` | No |
| Joint controller | python | `joint_controller.py` | Yes |

---

## Chapter 2: Digital Twins & HRI in Unity

### Required Sections

1. **What You'll Learn** (bullet list, 4-6 items)
   - Unity-ROS 2 integration via ROS-TCP-Connector
   - URDF import and robot visualization
   - Realistic environment creation
   - Human avatar integration for HRI
   - Real-time bidirectional communication

2. **Why Unity for Digital Twins?**
   - Comparison: Unity vs Gazebo capabilities
   - Use cases: visualization, HRI, synthetic data
   - Table: Feature comparison

3. **Setting Up Unity with ROS 2**
   - Installing Unity Hub and Editor
   - Adding ROS-TCP-Connector package
   - Configuring ROS endpoint
   - Code example: ROS settings configuration

4. **Importing Robot Models**
   - URDF Importer package
   - Step-by-step import workflow
   - ArticulationBody configuration
   - Screenshots/diagrams of import process

5. **Creating Realistic Environments**
   - Lighting setup (HDRP/URP considerations)
   - Material assignment
   - Physics settings for ArticulationBody
   - Code example: Environment setup script

6. **Human-Robot Interaction Setup**
   - Human avatar options (Unity built-in, Mixamo)
   - Interaction scenarios
   - Animation integration
   - Code example: HRI interaction script

7. **Bidirectional ROS Communication**
   - Publishing from Unity to ROS 2
   - Subscribing in Unity from ROS 2
   - Code example: C# ROS subscriber
   - Code example: C# ROS publisher

8. **Key Takeaways** (bullet list, 5-6 items)

9. **Next Steps** (link to Chapter 3)

### Code Examples Required

| Example | Language | Title | Has Output |
|---------|----------|-------|------------|
| ROS endpoint config | csharp | `ROSConnection.cs` | No |
| Joint subscriber | csharp | `JointStateSubscriber.cs` | Yes |
| Command publisher | csharp | `JointCommandPublisher.cs` | Yes |
| HRI interaction | csharp | `HRIInteraction.cs` | No |

---

## Chapter 3: Sensor Simulation & Validation

### Required Sections

1. **What You'll Learn** (bullet list, 4-6 items)
   - LiDAR sensor configuration and parameters
   - Depth camera setup and noise models
   - IMU simulation with realistic noise
   - Sensor data visualization in RViz
   - Validation against real sensor specifications

2. **Sensor Simulation Overview**
   - Why simulated sensors matter
   - Simulation-to-reality gap
   - Table: Sensor types and use cases

3. **LiDAR Simulation**
   - Parameters: range, resolution, scan rate
   - Noise models (Gaussian)
   - Code example: LiDAR SDF configuration
   - Visualization in RViz

4. **Depth Camera Simulation**
   - Parameters: resolution, FOV, depth range
   - RGB + Depth streams
   - Noise and artifacts
   - Code example: Depth camera SDF configuration

5. **IMU Simulation**
   - Parameters: update rate, axes
   - Noise and bias models
   - Code example: IMU SDF configuration
   - Integration with robot state estimation

6. **Visualizing Sensor Data**
   - RViz2 configuration
   - Point cloud display
   - Image topics
   - Code example: RViz config file

7. **Sensor Validation Approaches**
   - Comparing to real sensor specs
   - Domain randomization basics
   - Table: Common validation metrics

8. **Key Takeaways** (bullet list, 5-6 items)

9. **Module Summary** (link to Module 3 preview)

### Code Examples Required

| Example | Language | Title | Has Output |
|---------|----------|-------|------------|
| LiDAR sensor | xml | `lidar_sensor.sdf` | No |
| Depth camera | xml | `depth_camera.sdf` | No |
| IMU sensor | xml | `imu_sensor.sdf` | No |
| RViz config | yaml | `sensors.rviz` | No |
| Sensor bridge launch | python | `sensor_bridge.launch.py` | Yes |

---

## Cross-Chapter Requirements

### Navigation Links
- Chapter 1 ends with: "Next: [Digital Twins & HRI in Unity](./02-unity-digital-twins.md)"
- Chapter 2 ends with: "Next: [Sensor Simulation & Validation](./03-sensor-simulation.md)"
- Chapter 3 ends with: "Continue with Module 3: Motion Planning (coming soon)"

### Consistency Rules
- All SDF/XML examples use consistent robot name: `simple_humanoid`
- All Python examples use ROS 2 Humble conventions
- All C# examples use Unity 2022 LTS API
- Technical terms defined in first chapter they appear

### Module 1 References
- Link to URDF chapter when discussing URDF extensions
- Reference ROS 2 communication patterns when explaining ros_gz_bridge
- Build on joint concepts from Module 1 Chapter 3
