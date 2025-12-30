# Chapter Contracts: Module 3 - NVIDIA Isaac

**Feature**: 003-nvidia-isaac-module-3
**Date**: 2025-12-29
**Purpose**: Define contracts for each chapter's content and structure

---

## Contract 1: Chapter 1 - Isaac Sim & Synthetic Data

### Input Requirements
- Reader has completed Modules 1-2
- Reader has NVIDIA GPU (RTX 3060+) with Ubuntu 22.04
- Reader has 32GB RAM and 100GB storage

### Output Guarantees
1. Reader can install Isaac Sim via Omniverse Launcher
2. Reader can load a humanoid robot (USD format) into Isaac Sim
3. Reader can configure domain randomization (lighting, textures)
4. Reader can generate labeled synthetic data in COCO format
5. Reader understands when to use Isaac Sim vs Gazebo

### Content Contract

| Section | Must Include | Validation |
|---------|--------------|------------|
| Installation | Step-by-step commands | Screenshot of Isaac Sim launched |
| USD Loading | Python script | Robot visible in viewport |
| Replicator | Pipeline code | Generated images with annotations |
| Domain Randomization | Configuration examples | Varied output images |
| Export | Writer configuration | Valid COCO JSON output |

### Code Examples Required

```python
# Example 1: Launch Isaac Sim headless
# File: isaac_sim_launch.py
# Expected output: Isaac Sim running, viewport accessible via localhost:8211
```

```python
# Example 2: Load humanoid robot
# File: load_humanoid.py
# Expected output: Robot visible in Isaac Sim scene
```

```python
# Example 3: Synthetic data pipeline
# File: replicator_pipeline.py
# Expected output: 100 images saved with COCO annotations
```

```python
# Example 4: Domain randomization
# File: domain_randomization.py
# Expected output: Scene with randomized lighting/textures
```

### Success Criteria
- [ ] SC-001: Launch Isaac Sim within 15 minutes
- [ ] SC-002: Generate 1000 labeled images
- [ ] SC-007: Articulate domain randomization purpose

---

## Contract 2: Chapter 2 - Isaac ROS Perception & VSLAM

### Input Requirements
- Reader has completed Chapter 1
- Reader has Docker installed
- Reader has Jetson Orin OR x86 with RTX 3060+
- Reader understands ROS 2 topics/nodes

### Output Guarantees
1. Reader can install Isaac ROS via Docker
2. Reader can run GPU-accelerated stereo depth
3. Reader can run AprilTag detection
4. Reader can deploy VSLAM with odometry output
5. Reader can measure performance improvements

### Content Contract

| Section | Must Include | Validation |
|---------|--------------|------------|
| Installation | Docker commands | Container running |
| Stereo Depth | Launch file | Depth image in RViz |
| AprilTag | Launch file | Detected tags in RViz |
| VSLAM | Configuration | Odometry topic publishing |
| Performance | Benchmark | FPS comparison table |

### Code Examples Required

```bash
# Example 1: Docker setup
# File: isaac_ros_setup.sh
# Expected output: Isaac ROS container running
```

```python
# Example 2: Stereo depth launch
# File: stereo_depth_launch.py
# Expected output: /depth_image topic publishing at >30 FPS
```

```python
# Example 3: VSLAM launch
# File: vslam_humanoid.launch.py
# Expected output: /odom and /map topics publishing
```

```python
# Example 4: Full perception pipeline
# File: perception_pipeline.launch.py
# Expected output: Integrated perception stack running
```

### Success Criteria
- [ ] SC-003: Install and run Isaac ROS within 20 minutes
- [ ] SC-004: Observe 5x performance improvement
- [ ] SC-005: VSLAM odometry visible in RViz
- [ ] SC-008: Explain NITROS, TensorRT, zero-copy

---

## Contract 3: Chapter 3 - Nav2 for Humanoid Path Planning

### Input Requirements
- Reader has completed Chapters 1-2
- Reader has VSLAM running
- Reader understands Nav2 basics

### Output Guarantees
1. Reader can configure Nav2 for humanoid footprint
2. Reader can configure costmaps with appropriate inflation
3. Reader can send navigation goals
4. Reader can observe dynamic replanning
5. Reader understands deployment considerations

### Content Contract

| Section | Must Include | Validation |
|---------|--------------|------------|
| Nav2 Setup | Parameter files | Nav2 nodes running |
| Costmaps | YAML configuration | Costmap visible in RViz |
| Behavior Trees | XML definition | BT executing |
| Path Planning | Planner comparison | Path generated |
| Obstacle Avoidance | Demo scenario | Replanning observed |

### Code Examples Required

```yaml
# Example 1: Nav2 parameters
# File: humanoid_nav_params.yaml
# Expected output: Nav2 configured for humanoid
```

```yaml
# Example 2: Costmap configuration
# File: humanoid_costmap.yaml
# Expected output: Appropriate inflation for humanoid
```

```xml
# Example 3: Behavior tree
# File: navigation_bt.xml
# Expected output: NavigateToPose with humanoid recovery
```

```python
# Example 4: Nav2 launch
# File: nav2_launch.py
# Expected output: Full navigation stack running
```

### Success Criteria
- [ ] SC-006: Configure Nav2 within 15 minutes
- [ ] SC-009: All examples run on specified hardware

---

## Cross-Chapter Contracts

### Consistency Requirements
| Requirement | Validation |
|-------------|------------|
| Same ROS 2 Humble version | All launch files compatible |
| Same Isaac ROS version | 3.0 throughout |
| Same Isaac Sim version | 2023.1.1 throughout |
| Same robot model | Consistent USD/URDF |

### Writing Standards
| Standard | Validation |
|----------|------------|
| "What You'll Learn" present | Each chapter starts with list |
| "Key Takeaways" present | Each chapter ends with list |
| Technical terms defined | First use includes definition |
| Expected outputs shown | All code blocks have output |
| Hardware requirements stated | Each chapter intro |

### Link Requirements
| Link Type | Requirement |
|-----------|-------------|
| Internal | Use relative Docusaurus paths |
| External | Official NVIDIA documentation only |
| Code | GitHub-style permalinks |
