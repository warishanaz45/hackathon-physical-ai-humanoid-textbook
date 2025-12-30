---
sidebar_position: 3
title: "Sensor Simulation & Validation"
description: "Configure and validate LiDAR, depth cameras, and IMU sensors in simulation"
keywords: [LiDAR, depth camera, IMU, sensor simulation, noise models, RViz, Gazebo sensors]
---

# Sensor Simulation & Validation

## What You'll Learn

By the end of this chapter, you will be able to:

- Understand why sensor simulation is critical for robotics development
- Configure LiDAR sensors with realistic parameters and noise models
- Set up depth cameras with appropriate resolution and noise
- Simulate IMU sensors with bias and noise characteristics
- Visualize sensor data in RViz
- Validate simulated sensors against real-world specifications

---

## Sensor Simulation Overview

Simulated sensors enable **perception algorithm development** without physical hardware. This accelerates development, enables exhaustive testing, and reduces costs.

### Why Simulated Sensors Matter

| Benefit | Description |
|---------|-------------|
| **Cost reduction** | No physical sensors needed during development |
| **Safe testing** | Test edge cases that would be dangerous in reality |
| **Repeatability** | Same scenario, same sensor data every time |
| **Ground truth** | Perfect knowledge of true state for validation |
| **Domain randomization** | Train robust perception with varied conditions |

### The Simulation-Reality Gap

Simulated sensors differ from real sensors in several ways:

```text
Reality                     Simulation
   │                            │
   ├─ Physical noise            ├─ Modeled noise (Gaussian)
   ├─ Environmental effects     ├─ Simplified effects
   ├─ Sensor-specific quirks    ├─ Idealized behavior
   ├─ Manufacturing variance    ├─ Perfect specs
   └─ Degradation over time     └─ Consistent performance
```

**Mitigation strategies**:
- Use realistic noise parameters from datasheets
- Apply domain randomization during training
- Validate with real sensor data periodically

### Sensor Types and Use Cases

| Sensor | Primary Use | Data Type | Typical Rate |
|--------|-------------|-----------|--------------|
| **LiDAR** | 3D mapping, obstacle detection | Point cloud | 10-20 Hz |
| **Depth Camera** | Close-range 3D, object recognition | Depth image | 30-90 Hz |
| **RGB Camera** | Visual perception, tracking | Color image | 30-60 Hz |
| **IMU** | Orientation, motion estimation | Accel + Gyro | 100-400 Hz |
| **Force/Torque** | Manipulation, contact detection | Force vector | 100-1000 Hz |

---

## LiDAR Simulation

**LiDAR (Light Detection and Ranging)** provides 3D point clouds for mapping and obstacle detection.

### Key Parameters

| Parameter | Description | Typical Range |
|-----------|-------------|---------------|
| `horizontal_samples` | Points per scan line | 360-2048 |
| `vertical_samples` | Number of scan lines | 1-128 |
| `horizontal_fov` | Horizontal field of view | 360° (spinning) |
| `vertical_fov` | Vertical field of view | 30°-40° |
| `min_range` | Minimum detection distance | 0.1-0.5 m |
| `max_range` | Maximum detection distance | 10-200 m |
| `update_rate` | Scans per second | 10-20 Hz |
| `noise_stddev` | Range measurement noise | 0.01-0.05 m |

### LiDAR SDF Configuration

```xml title="lidar_sensor.sdf"
<!-- Add this inside your robot model's link -->
<sensor name="lidar" type="gpu_lidar">
  <pose>0 0 0.2 0 0 0</pose>  <!-- Position on robot -->
  <topic>/scan</topic>
  <update_rate>10</update_rate>
  <always_on>true</always_on>
  <visualize>true</visualize>

  <lidar>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle>  <!-- -15 degrees -->
        <max_angle>0.261799</max_angle>   <!-- +15 degrees -->
      </vertical>
    </scan>

    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>

    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.02</stddev>  <!-- 2cm standard deviation -->
    </noise>
  </lidar>
</sensor>
```

### Realistic LiDAR Noise

Real LiDAR sensors have multiple noise sources:

| Noise Type | Simulation Approach | Typical Value |
|------------|---------------------|---------------|
| **Range noise** | Gaussian on distance | 1-3 cm stddev |
| **Angular noise** | Gaussian on ray direction | 0.01-0.05° |
| **Dropout** | Random missing points | 1-5% rate |
| **Multipath** | Ghost points | Model-specific |
| **Edge effects** | Points at object edges | Increased noise |

For a VLP-16 (Velodyne Puck) style sensor:
```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.03</stddev>  <!-- 3cm, typical for VLP-16 -->
</noise>
```

---

## Depth Camera Simulation

**Depth cameras** (e.g., Intel RealSense, Microsoft Kinect) provide dense depth maps at close range.

### Key Parameters

| Parameter | Description | Typical Range |
|-----------|-------------|---------------|
| `width` | Image width (pixels) | 640-1280 |
| `height` | Image height (pixels) | 480-720 |
| `horizontal_fov` | Field of view | 60°-90° |
| `min_depth` | Minimum depth | 0.1-0.3 m |
| `max_depth` | Maximum depth | 3-10 m |
| `update_rate` | Frames per second | 30-90 Hz |
| `noise` | Depth measurement noise | Distance-dependent |

### Depth Camera SDF Configuration

```xml title="depth_camera.sdf"
<!-- Add this inside your robot model's link -->
<sensor name="depth_camera" type="depth_camera">
  <pose>0.1 0 0.15 0 0 0</pose>  <!-- Position on robot head -->
  <topic>/camera/depth</topic>
  <update_rate>30</update_rate>
  <always_on>true</always_on>
  <visualize>true</visualize>

  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R_FLOAT32</format>
    </image>

    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>

    <depth_camera>
      <output>depths</output>
      <clip>
        <near>0.2</near>
        <far>5.0</far>
      </clip>
    </depth_camera>

    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.005</stddev>  <!-- 5mm baseline noise -->
    </noise>
  </camera>
</sensor>

<!-- RGB camera (paired with depth) -->
<sensor name="rgb_camera" type="camera">
  <pose>0.1 0 0.15 0 0 0</pose>
  <topic>/camera/rgb</topic>
  <update_rate>30</update_rate>
  <always_on>true</always_on>

  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100.0</far>
    </clip>
  </camera>
</sensor>
```

### Depth Camera Noise Model

Real depth cameras (like Intel RealSense D435) have **distance-dependent noise**:

```text
Noise ∝ distance²

At 1m: ~1-2mm
At 2m: ~4-8mm
At 3m: ~9-18mm
```

To simulate this in Gazebo, you may need a custom plugin or post-processing.

---

## IMU Simulation

**IMU (Inertial Measurement Unit)** provides acceleration and angular velocity measurements for state estimation.

### Key Parameters

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `update_rate` | Measurements per second | 100-400 Hz |
| `accel_noise` | Accelerometer noise density | 0.0001-0.001 m/s²/√Hz |
| `accel_bias` | Accelerometer bias instability | 0.00001-0.0001 m/s² |
| `gyro_noise` | Gyroscope noise density | 0.0001-0.001 rad/s/√Hz |
| `gyro_bias` | Gyroscope bias instability | 0.00001-0.0001 rad/s |

### IMU SDF Configuration

```xml title="imu_sensor.sdf"
<!-- Add this inside your robot model's link (typically torso) -->
<sensor name="imu" type="imu">
  <pose>0 0 0 0 0 0</pose>  <!-- At link origin -->
  <topic>/imu/data</topic>
  <update_rate>200</update_rate>
  <always_on>true</always_on>
  <visualize>false</visualize>

  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.0001</stddev>  <!-- rad/s -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.0001</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.0001</stddev>
        </noise>
      </z>
    </angular_velocity>

    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.001</stddev>  <!-- m/s² -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.001</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.001</stddev>
        </noise>
      </z>
    </linear_acceleration>

    <!-- Enable orientation output -->
    <enable_orientation>true</enable_orientation>
  </imu>
</sensor>
```

### IMU Bias Modeling

Real IMUs have bias that drifts over time. For high-fidelity simulation:

```text
True value = Measured value + Bias + Noise

Bias characteristics:
- Initial bias: Random offset at startup
- Bias instability: Slow random walk
- Temperature dependence: (often ignored in simulation)
```

---

## Visualizing Sensor Data

RViz2 is the standard tool for visualizing ROS 2 sensor data.

### RViz Configuration

```yaml title="sensors.rviz"
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Displays:
    # Robot Model
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description Topic: /robot_description

    # LiDAR Point Cloud
    - Class: rviz_default_plugins/PointCloud2
      Name: LiDAR
      Enabled: true
      Topic: /scan
      Style: Points
      Size (Pixels): 3
      Color Transformer: Intensity
      Decay Time: 0

    # Depth Camera
    - Class: rviz_default_plugins/PointCloud2
      Name: DepthCloud
      Enabled: true
      Topic: /camera/depth/points
      Style: Points
      Size (Pixels): 2
      Color Transformer: RGB8

    # RGB Camera Image
    - Class: rviz_default_plugins/Image
      Name: CameraImage
      Enabled: true
      Topic: /camera/rgb
      Transport Hint: raw

    # TF Frames
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Show Names: true
      Show Axes: true

  Global Options:
    Fixed Frame: base_link
    Frame Rate: 30
```

### Launch File for Sensor Bridge

```python title="sensor_bridge.launch.py"
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Bridge LiDAR data
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            ],
            output='screen',
        ),

        # Bridge depth camera data
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera/depth@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/depth/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                '/camera/rgb@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            ],
            output='screen',
        ),

        # Bridge IMU data
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            ],
            output='screen',
        ),

        # Launch RViz with config
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                os.path.dirname(__file__), 'sensors.rviz'
            )],
            output='screen',
        ),
    ])
```

**Expected output when running:**

```text
[INFO] [ros_gz_bridge]: Creating bridge for topic [/scan]
[INFO] [ros_gz_bridge]: Creating bridge for topic [/camera/depth]
[INFO] [ros_gz_bridge]: Creating bridge for topic [/imu/data]
[INFO] [rviz2]: RViz2 started
```

---

## Sensor Validation Approaches

Validating that simulated sensors match real-world behavior is critical for sim-to-real transfer.

### Validation Metrics

| Metric | Description | Target |
|--------|-------------|--------|
| **Noise distribution** | Compare histogram of noise | Match datasheet |
| **Range accuracy** | Error at known distances | < 2-3x datasheet |
| **Detection rate** | Objects detected vs missed | > 95% |
| **False positive rate** | Ghost detections | < 1% |
| **Latency** | Timestamp accuracy | < 1 frame |
| **Resolution** | Angular/spatial resolution | Match spec |

### Validation Process

```text
1. Collect Real Data
   └─ Record sensor data in controlled environment
   └─ Known object positions (ground truth)

2. Replicate in Simulation
   └─ Create matching environment in Gazebo
   └─ Same sensor configuration

3. Compare Metrics
   └─ Noise characteristics
   └─ Detection performance
   └─ Edge cases

4. Tune Parameters
   └─ Adjust noise models
   └─ Add missing effects
   └─ Iterate until acceptable match
```

### Domain Randomization

For robust perception, **randomize simulation parameters** during training:

| Parameter | Randomization Range |
|-----------|---------------------|
| Sensor noise | 0.5x - 2x nominal |
| Lighting | 0.5x - 2x intensity |
| Object textures | Multiple variations |
| Object positions | +/- 10cm noise |
| Sensor pose | +/- 1° orientation |
| Missing points | 0-10% dropout |

This trains perception models that generalize better to real-world conditions.

---

## Key Takeaways

1. **Sensor simulation accelerates development** — test perception without physical hardware
2. **LiDAR** provides 3D point clouds for mapping — configure range, resolution, and Gaussian noise
3. **Depth cameras** give dense depth at close range — model distance-dependent noise
4. **IMU** enables state estimation — include both noise and bias characteristics
5. **RViz2** visualizes all sensor data — use ros_gz_bridge to connect Gazebo topics
6. **Validation is essential** — compare simulated vs real sensor characteristics
7. **Domain randomization** bridges the sim-to-real gap for robust perception

---

## Module Summary

Congratulations! You've completed **Module 2: The Digital Twin**.

You now understand:
- **Gazebo Fortress** for physics-based simulation
- **Unity** for high-fidelity visualization and HRI
- **Sensor simulation** with LiDAR, depth cameras, and IMU

### What's Next

In **Module 3: Motion Planning** (coming soon), you'll learn:
- Motion planning with MoveIt 2
- Inverse kinematics for humanoid arms
- Trajectory generation and execution
- Collision avoidance in dynamic environments

Continue your journey with humanoid robotics!
