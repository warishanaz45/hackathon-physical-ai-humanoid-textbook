# Research: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature**: 002-gazebo-unity-module-2
**Date**: 2025-12-29
**Status**: Complete

## Research Questions

### RQ-001: Gazebo Version and ROS 2 Integration

**Question**: Which Gazebo version should be used with ROS 2 Humble?

**Decision**: Gazebo Fortress (Ignition Gazebo)

**Rationale**:
- Gazebo Fortress is the officially supported version for ROS 2 Humble
- Released as part of the "Ignition" project, now rebranded as "Gazebo"
- Provides modern plugin architecture compatible with ROS 2 design patterns
- Long-term support aligns with ROS 2 Humble EOL (2027)

**Alternatives Considered**:
- Gazebo Classic (gazebo11): Legacy version, not recommended for new projects
- Gazebo Garden: Newer but less stable, may have documentation gaps

**Source**: ROS 2 Humble Release Notes, Gazebo Documentation

---

### RQ-002: Unity-ROS 2 Integration Approach

**Question**: What is the recommended method for Unity-ROS 2 communication?

**Decision**: Unity Robotics Hub with ROS-TCP-Connector

**Rationale**:
- Official Unity package maintained by Unity Technologies
- Supports bidirectional communication with ROS 2 Humble
- Handles message serialization/deserialization automatically
- Well-documented with tutorials and sample projects
- Active community and ongoing development

**Alternatives Considered**:
- ROS2-Unity Bridge (community): Less maintained, compatibility issues
- WebSocket bridges: Higher latency, more complex setup
- Shared memory: Complex setup, platform-specific

**Source**: Unity Robotics Hub GitHub, ROS-TCP-Connector Documentation

---

### RQ-003: Sensor Plugin Architecture

**Question**: Which sensor plugins are available for Gazebo Fortress?

**Decision**: Use gz-sensors library with built-in plugins

**Rationale**:
- gz-sensors provides standardized sensor implementations
- LiDAR: `gz::sensors::Lidar` with configurable rays, range, noise
- Camera: `gz::sensors::Camera` and `gz::sensors::DepthCamera`
- IMU: `gz::sensors::Imu` with noise models
- All sensors publish to ROS 2 topics via ros_gz_bridge

**Key Plugins**:
| Sensor | Plugin | ROS 2 Message Type |
|--------|--------|-------------------|
| LiDAR | `gz::sensors::Lidar` | `sensor_msgs/msg/LaserScan` |
| Depth Camera | `gz::sensors::DepthCamera` | `sensor_msgs/msg/Image` |
| RGB Camera | `gz::sensors::Camera` | `sensor_msgs/msg/Image` |
| IMU | `gz::sensors::Imu` | `sensor_msgs/msg/Imu` |

**Source**: gz-sensors API Documentation, ros_gz Package Documentation

---

### RQ-004: URDF to Unity Import Workflow

**Question**: How to import URDF robot models into Unity?

**Decision**: Use URDF Importer package from Unity Robotics Hub

**Rationale**:
- Directly imports URDF files with mesh references
- Converts joint definitions to Unity ArticulationBody components
- Handles collision meshes and visual materials
- Integrates with ROS-TCP-Connector for joint state publishing

**Workflow**:
1. Install URDF Importer package via Package Manager
2. Place URDF and mesh files in Assets folder
3. Right-click URDF → Import Robot from URDF
4. Configure ArticulationBody properties
5. Add ROS connection scripts

**Source**: Unity URDF Importer Documentation

---

### RQ-005: Physics Engine Comparison

**Question**: How do Gazebo and Unity physics engines differ?

**Decision**: Document both for appropriate use cases

**Comparison**:
| Aspect | Gazebo (DART/Bullet/ODE) | Unity (PhysX) |
|--------|--------------------------|---------------|
| **Primary Use** | Robotics simulation | Game development |
| **Accuracy** | High (engineering-grade) | Medium (game-grade) |
| **Real-time** | Configurable timestep | Fixed 0.02s default |
| **Contact Model** | Compliant contact | Rigid contact |
| **Joint Accuracy** | Very high | Good |
| **Rendering** | Basic | High-fidelity |

**Recommendation**:
- Use Gazebo for physics validation and controller testing
- Use Unity for visualization, HRI, and training data generation

**Source**: Gazebo Physics Plugin Documentation, Unity PhysX Documentation

---

### RQ-006: Sensor Noise Models

**Question**: What noise models are available for simulated sensors?

**Decision**: Document Gaussian noise models with configurable parameters

**LiDAR Noise Model**:
```xml
<noise type="gaussian">
  <mean>0.0</mean>
  <stddev>0.01</stddev>  <!-- 1cm standard deviation -->
</noise>
```

**Camera Noise Model**:
- Gaussian noise on pixel values
- Depth quantization noise
- Missing depth values (holes)

**IMU Noise Model**:
```xml
<angular_velocity>
  <noise type="gaussian">
    <mean>0</mean>
    <stddev>0.0001</stddev>
  </noise>
</angular_velocity>
<linear_acceleration>
  <noise type="gaussian">
    <mean>0</mean>
    <stddev>0.001</stddev>
  </noise>
</linear_acceleration>
```

**Source**: gz-sensors Noise Models, Real Sensor Datasheets (Intel RealSense, Velodyne)

---

### RQ-007: Chapter Organization Best Practices

**Question**: How should simulation content be organized for progressive learning?

**Decision**: Build from physics foundation to visualization to sensing

**Chapter Flow**:
1. **Gazebo First**: Establishes physics simulation fundamentals
2. **Unity Second**: Adds visualization on top of physics understanding
3. **Sensors Third**: Applies sensing to both platforms

**Rationale**:
- Physics is prerequisite for understanding sensor behavior
- Unity builds on URDF knowledge from Module 1
- Sensors tie everything together with practical perception

**Source**: Educational design principles, Module 1 structure precedent

---

## Technology Stack Summary

| Component | Technology | Version |
|-----------|------------|---------|
| Physics Simulation | Gazebo Fortress | 6.x |
| ROS Integration | ros_gz | Latest for Humble |
| Unity | Unity Editor | 2022.3 LTS |
| Unity-ROS Bridge | ROS-TCP-Connector | 0.7.0+ |
| URDF Import | Unity URDF Importer | Latest |
| Sensor Visualization | RViz2 | Humble |

## Resolved Clarifications

All technical context items resolved:

- **Language/Version**: N/A (documentation project)
- **Primary Dependencies**: Gazebo Fortress, Unity 2022 LTS, ROS-TCP-Connector
- **Storage**: N/A (no database for documentation)
- **Testing**: Docusaurus build verification
- **Target Platform**: Ubuntu 22.04 + Windows/macOS for Unity
- **Project Type**: Documentation (Docusaurus)
- **Performance Goals**: Build time < 60s, all examples runnable
- **Constraints**: Content must work with ROS 2 Humble EOL timeline
- **Scale/Scope**: 3 chapters, ~50 pages total
