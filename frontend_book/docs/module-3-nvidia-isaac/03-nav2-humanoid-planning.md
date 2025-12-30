---
sidebar_position: 3
title: "Nav2 for Humanoid Path Planning"
description: "Configure Nav2 for autonomous humanoid robot navigation with behavior trees and dynamic obstacle avoidance"
keywords: [nav2, navigation, path planning, humanoid, behavior tree, costmap, ros2, obstacle avoidance]
---

# Nav2 for Humanoid Path Planning

:::info Hardware Requirements
- **GPU**: NVIDIA Jetson Orin OR x86 with RTX 3060+ (same as Chapter 2)
- **RAM**: 16GB minimum
- **OS**: Ubuntu 22.04 LTS with ROS 2 Humble
- **Nav2**: Navigation2 stack installed
:::

:::tip Prerequisites
Before starting this chapter, ensure you have completed:
- **Chapters 1-2**: Isaac Sim, Isaac ROS, and VSLAM setup
- **Nav2 basics**: Familiarity with ROS 2 navigation concepts (from Module 2)
:::

## What You'll Learn

In this chapter, you will:

- **Understand** Nav2 architecture including planners, controllers, costmaps, and behavior trees
- **Identify** humanoid-specific navigation challenges: stability, footprint, and velocity constraints
- **Configure** costmaps with appropriate inflation layers for bipedal robot safety
- **Design** behavior trees for navigation with humanoid-specific recovery behaviors
- **Compare** path planning algorithms and select appropriate ones for humanoid movement
- **Implement** dynamic obstacle avoidance with real-time replanning

---

## Nav2 Architecture Overview

### What is Nav2?

**Nav2** (Navigation2) is the ROS 2 navigation stack providing autonomous navigation capabilities:

```
┌─────────────────────────────────────────────────────────────────┐
│                        Nav2 Architecture                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐         │
│  │ BT Navigator │───▶│   Planner   │───▶│ Controller  │         │
│  │  (Behavior  │    │   Server    │    │   Server    │         │
│  │    Tree)    │    │(Global Path)│    │(Local Ctrl) │         │
│  └─────────────┘    └─────────────┘    └─────────────┘         │
│         │                  │                  │                 │
│         ▼                  ▼                  ▼                 │
│  ┌─────────────────────────────────────────────────────┐       │
│  │              Costmap 2D (Obstacle Layer)            │       │
│  │  ┌─────────┐  ┌──────────┐  ┌──────────────────┐   │       │
│  │  │ Static  │  │ Obstacle │  │ Inflation Layer  │   │       │
│  │  │  Layer  │  │  Layer   │  │  (Safety Margin) │   │       │
│  │  └─────────┘  └──────────┘  └──────────────────┘   │       │
│  └─────────────────────────────────────────────────────┘       │
│                           │                                     │
│                           ▼                                     │
│  ┌─────────────────────────────────────────────────────┐       │
│  │     Localization (AMCL / VSLAM from Chapter 2)      │       │
│  └─────────────────────────────────────────────────────┘       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Key Nav2 Components

| Component | Role | Humanoid Considerations |
|-----------|------|------------------------|
| **BT Navigator** | Orchestrates navigation behaviors | Custom recovery for balance |
| **Planner Server** | Computes global path to goal | Smooth paths for bipedal gait |
| **Controller Server** | Follows path, avoids obstacles | Velocity limits for stability |
| **Costmap 2D** | Represents obstacles and free space | Larger inflation for arm swing |
| **Recovery Server** | Handles failures | Balance-aware recovery |

---

## Humanoid-Specific Challenges

### Why Humanoids are Different

Humanoid robots have unique navigation requirements compared to wheeled robots:

| Challenge | Wheeled Robot | Humanoid Robot |
|-----------|---------------|----------------|
| **Stability** | Always stable | Must maintain balance during motion |
| **Footprint** | Fixed circular/rectangular | Dynamic (arm swing, stepping) |
| **Velocity** | Can stop instantly | Needs deceleration for stability |
| **Terrain** | Requires flat surface | Can step over small obstacles |
| **Recovery** | Spin in place | Must stabilize before moving |

### Footprint Considerations

```yaml title="Humanoid Footprint vs. Safety Zone"
# The robot footprint is the physical body outline
# But for safety, we need a larger zone accounting for:
# - Arm swing during walking
# - Stepping motion width
# - Balance recovery space

footprint: [[-0.15, -0.25], [-0.15, 0.25], [0.15, 0.25], [0.15, -0.25]]
#           ↑ Back-left    ↑ Back-right  ↑ Front-right ↑ Front-left
#           Standing footprint: 0.3m x 0.5m

# Safety inflation accounts for dynamic movement
inflation_radius: 0.5  # Extra 0.35m for arm swing and stepping
```

### Velocity Limitations

```yaml title="Humanoid Velocity Constraints"
# Maximum velocities must ensure stability
max_vel_x: 0.5      # Forward: 0.5 m/s (walking speed)
max_vel_y: 0.0      # Lateral: 0 (humanoids walk forward, turn in place)
max_vel_theta: 0.5  # Rotation: 0.5 rad/s (slow turning)

# Acceleration limits for smooth motion
acc_lim_x: 0.3      # Gentle acceleration
acc_lim_y: 0.0
acc_lim_theta: 0.3

# Deceleration (critical for balance)
decel_lim_x: -0.5   # Can decelerate faster than accelerate
```

---

## Costmap Configuration

### Understanding Costmap Layers

Nav2 costmaps combine multiple layers to represent the environment:

```
┌────────────────────────────────────────┐
│           Combined Costmap             │
├────────────────────────────────────────┤
│  ┌──────────────────────────────────┐  │
│  │      Inflation Layer             │  │  Cost = 253 → 0
│  │  (Gradual decay from obstacles)  │  │  (Lethal → Free)
│  └──────────────────────────────────┘  │
│              ▲                         │
│  ┌──────────────────────────────────┐  │
│  │      Obstacle Layer              │  │  From sensors:
│  │  (Dynamic obstacles from VSLAM)  │  │  LiDAR, depth camera
│  └──────────────────────────────────┘  │
│              ▲                         │
│  ┌──────────────────────────────────┐  │
│  │       Static Layer               │  │  From map file
│  │  (Walls, permanent obstacles)    │  │  (SLAM output)
│  └──────────────────────────────────┘  │
└────────────────────────────────────────┘
```

### Humanoid Costmap Configuration

```yaml title="humanoid_costmap.yaml"
# Global costmap (for path planning)
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link

      # Resolution: smaller = more precise but slower
      resolution: 0.05  # 5cm cells for humanoid precision

      # Track unknown space (important for exploration)
      track_unknown_space: true

      # Robot footprint for collision checking
      # Rectangular footprint typical for humanoids
      footprint: "[[0.15, 0.25], [0.15, -0.25], [-0.15, -0.25], [-0.15, 0.25]]"

      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transitioned_to_inactive: true

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: vslam_pointcloud
        vslam_pointcloud:
          topic: /visual_slam/vis/observations_cloud
          data_type: "PointCloud2"
          marking: true
          clearing: true
          max_obstacle_height: 2.0  # Humanoid is tall
          min_obstacle_height: 0.1  # Ignore ground noise

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        # CRITICAL: Large inflation for humanoid safety
        inflation_radius: 0.55  # Robot radius + arm swing + margin

# Local costmap (for obstacle avoidance)
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link

      # Smaller area but higher resolution for local planning
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.03  # 3cm for fine obstacle detection

      footprint: "[[0.15, 0.25], [0.15, -0.25], [-0.15, -0.25], [-0.15, 0.25]]"

      plugins: ["obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: depth_camera
        depth_camera:
          topic: /stereo/points2
          data_type: "PointCloud2"
          marking: true
          clearing: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 4.0
        inflation_radius: 0.55
```

---

## Behavior Trees for Navigation

### What are Behavior Trees?

**Behavior Trees (BTs)** are hierarchical structures that control robot behavior:

```
               ┌─────────────┐
               │   Sequence  │  (Execute children in order)
               └──────┬──────┘
                      │
        ┌─────────────┼─────────────┐
        ▼             ▼             ▼
   ┌─────────┐  ┌──────────┐  ┌──────────┐
   │ Compute │  │  Follow  │  │  Goal    │
   │  Path   │  │   Path   │  │ Reached  │
   └─────────┘  └──────────┘  └──────────┘
```

### NavigateToPose Behavior Tree

```xml title="navigation_bt.xml"
<?xml version="1.0"?>
<!--
  Humanoid Navigation Behavior Tree
  Custom recovery behaviors for balance maintenance
-->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">

    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <!-- Main navigation sequence -->
      <PipelineSequence name="NavigateWithReplanning">

        <!-- Rate-limited replanning for smooth paths -->
        <RateController hz="0.5">
          <ComputePathToPose goal="{goal}" path="{path}"
                             planner_id="GridBased"/>
        </RateController>

        <!-- Path following with stability check -->
        <ReactiveSequence name="FollowPathWithStability">
          <!-- Custom: Check humanoid balance before each step -->
          <StabilityCheck threshold="0.1" topic="/imu/data"/>

          <FollowPath path="{path}" controller_id="FollowPath"/>
        </ReactiveSequence>

      </PipelineSequence>

      <!-- Recovery behaviors (humanoid-specific) -->
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>

        <RoundRobin name="RecoveryActions">
          <!-- First: Clear costmap (standard) -->
          <ClearEntireCostmap name="ClearGlobalCostmap"
                              service_name="/global_costmap/clear_entirely_global_costmap"/>

          <!-- Second: Wait for balance recovery (humanoid-specific) -->
          <Wait wait_duration="3" name="WaitForStability"/>

          <!-- Third: Small backup motion -->
          <BackUp backup_dist="0.15" backup_speed="0.1"
                  name="BackUpSmall"/>

          <!-- Fourth: Rotate slowly in place -->
          <Spin spin_dist="0.5" name="SpinSlow"/>

          <!-- Fifth: Longer wait (last resort) -->
          <Wait wait_duration="5" name="WaitLong"/>
        </RoundRobin>

      </ReactiveFallback>
    </RecoveryNode>

  </BehaviorTree>
</root>
```

### Custom Stability Check Node

For humanoids, add a custom BT node to verify balance:

```cpp title="StabilityCheck BT Node (Concept)"
// Custom behavior tree node that checks IMU data
// before allowing the robot to continue walking

class StabilityCheck : public BT::ConditionNode {
public:
  BT::NodeStatus tick() override {
    // Get latest IMU data
    auto imu_msg = get_latest_imu();

    // Check if robot is stable (pitch/roll within threshold)
    double pitch = imu_msg.orientation.pitch;
    double roll = imu_msg.orientation.roll;

    if (abs(pitch) < threshold_ && abs(roll) < threshold_) {
      return BT::NodeStatus::SUCCESS;  // Continue navigation
    } else {
      return BT::NodeStatus::FAILURE;  // Trigger recovery
    }
  }
};
```

---

## Path Planning Algorithms

### Comparing Planners

Nav2 provides multiple path planners:

| Planner | Algorithm | Best For | Humanoid Suitability |
|---------|-----------|----------|---------------------|
| **NavFn** | Dijkstra/A* | Fast, simple paths | ⭐⭐ (sharp turns) |
| **Smac 2D** | A* variant | Memory efficient | ⭐⭐⭐ (smoother) |
| **Smac Hybrid** | Hybrid A* | Ackermann vehicles | ⭐ (not suitable) |
| **Smac Lattice** | State lattice | Smooth, feasible | ⭐⭐⭐⭐ (best) |
| **Theta*** | Any-angle A* | Smooth paths | ⭐⭐⭐⭐ (good) |

### Recommended: Smac Lattice Planner

For humanoids, the **Smac Lattice Planner** produces smooth, dynamically feasible paths:

```yaml title="Smac Lattice Configuration"
planner_server:
  ros__parameters:
    expected_planner_frequency: 10.0
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerLattice"

      # Lattice resolution
      lattice_filepath: "/path/to/humanoid_lattice.json"

      # Search parameters
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 5.0

      # Humanoid motion constraints
      motion_model: "MOORE"  # 8-connected grid

      # Smooth turning (no sharp turns)
      minimum_turning_radius: 0.3  # Humanoids need gradual turns

      # Cost penalties
      cost_travel_multiplier: 2.0
      cost_penalty: 2.0  # Penalize near-obstacle paths
```

---

## Dynamic Obstacle Avoidance

### Local Controller Tuning

The controller handles real-time obstacle avoidance:

```yaml title="humanoid_nav_params.yaml"
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001

    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.1
      movement_time_allowance: 20.0  # More time for slow humanoid

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.15    # 15cm position tolerance
      yaw_goal_tolerance: 0.25   # ~15 degree heading tolerance
      stateful: true

    # DWB Controller (good for humanoids)
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"

      # Velocity limits (humanoid-safe)
      min_vel_x: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0    # No sideways movement
      max_vel_theta: 0.5

      min_speed_xy: 0.1
      max_speed_xy: 0.5
      min_speed_theta: 0.0

      # Acceleration (smooth for balance)
      acc_lim_x: 0.3
      acc_lim_y: 0.0
      acc_lim_theta: 0.3
      decel_lim_x: -0.5
      decel_lim_y: 0.0
      decel_lim_theta: -0.5

      # Trajectory simulation
      vx_samples: 20
      vy_samples: 1
      vtheta_samples: 40
      sim_time: 2.0      # Look ahead 2 seconds

      # Critics (cost functions)
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle",
                "GoalAlign", "PathAlign", "PathDist", "GoalDist"]

      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.2
      GoalAlign.scale: 24.0
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0

# Recovery behaviors
recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]

    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"
```

### Replanning Strategies

```yaml title="Replanning Configuration"
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /visual_slam/tracking/odometry  # From Isaac ROS VSLAM

    # Behavior tree file
    default_bt_xml_filename: "/path/to/navigation_bt.xml"

    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
```

---

## Putting It Together

### Complete Nav2 Launch

```python title="nav2_launch.py"
#!/usr/bin/env python3
"""Launch complete Nav2 stack for humanoid robot navigation."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Configuration files
    nav2_params = LaunchConfiguration('params_file')
    bt_xml = LaunchConfiguration('bt_xml_file')

    # Declare arguments
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/path/to/humanoid_nav_params.yaml',
        description='Full path to Nav2 parameters file'
    )

    bt_arg = DeclareLaunchArgument(
        'bt_xml_file',
        default_value='/path/to/navigation_bt.xml',
        description='Full path to behavior tree XML'
    )

    # Nav2 nodes
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    recoveries_server = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[nav2_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'bt_xml_filename': bt_xml}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'recoveries_server',
                'bt_navigator'
            ]
        }]
    )

    return LaunchDescription([
        params_arg,
        bt_arg,
        controller_server,
        planner_server,
        recoveries_server,
        bt_navigator,
        lifecycle_manager,
    ])
```

**Expected Output**:
```
[lifecycle_manager]: Starting lifecycle manager
[controller_server]: Configuring controller server
[planner_server]: Created SmacPlannerLattice
[recoveries_server]: Created spin, backup, wait recoveries
[bt_navigator]: Loading behavior tree from /path/to/navigation_bt.xml
[lifecycle_manager]: All nodes active, navigation stack ready!
```

### Sending Navigation Goals

```bash title="Send Navigation Goal"
# Using CLI
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Using RViz
# 1. Click "2D Goal Pose" tool
# 2. Click and drag on map to set goal position and heading
```

### Simulation to Deployment Path

| Stage | Environment | Purpose |
|-------|-------------|---------|
| 1. Isaac Sim | Synthetic | Verify Nav2 config with perfect sensors |
| 2. Gazebo | Physics | Test with realistic physics, no RTX needed |
| 3. Hardware-in-Loop | Mixed | Real sensors → simulated actuators |
| 4. Real Robot | Physical | Final deployment with safety constraints |

---

## Key Takeaways

1. **Nav2 architecture** consists of planners (global path), controllers (local avoidance), and behavior trees (orchestration)—each requires humanoid-specific tuning.

2. **Humanoid footprint** must account for arm swing and stepping motion—use larger inflation radius (0.5m+) than the physical body size.

3. **Velocity constraints** are critical for balance—limit max velocity to 0.5 m/s and ensure smooth acceleration/deceleration profiles.

4. **Behavior trees** enable custom recovery behaviors—add stability checks and longer wait times for humanoid balance recovery.

5. **Smac Lattice Planner** produces the smoothest paths for humanoid robots—avoid sharp turns that could destabilize bipedal walking.

**Module Complete**: You now have the knowledge to build AI-driven perception and navigation for humanoid robots using the NVIDIA Isaac ecosystem!
