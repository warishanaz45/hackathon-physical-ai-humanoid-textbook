---
sidebar_position: 1
title: "Isaac Sim & Synthetic Data"
description: "Generate photorealistic synthetic training data for AI perception models using NVIDIA Isaac Sim and Replicator"
keywords: [nvidia, isaac sim, synthetic data, replicator, domain randomization, omniverse, humanoid robotics]
---

# Isaac Sim & Synthetic Data

:::info Hardware Requirements
- **GPU**: NVIDIA RTX 3060 or higher (RTX 4080/A5000 recommended)
- **RAM**: 32GB minimum (64GB recommended)
- **Storage**: 100GB SSD (500GB NVMe recommended)
- **OS**: Ubuntu 22.04 LTS
:::

:::tip Prerequisites
Before starting this chapter, ensure you have completed:
- **Module 1**: ROS 2 fundamentals and URDF basics
- **Module 2**: Simulation concepts with Gazebo and Unity
:::

## What You'll Learn

In this chapter, you will:

- **Understand** NVIDIA Omniverse and Isaac Sim architecture and their role in robotics simulation
- **Install** Isaac Sim via Omniverse Launcher on Ubuntu 22.04
- **Load** humanoid robot models in USD format and convert existing URDFs
- **Configure** Replicator for automated synthetic data generation with annotations
- **Apply** domain randomization techniques to bridge the simulation-to-reality gap
- **Export** labeled datasets in COCO and KITTI formats for model training

---

## Introduction to NVIDIA Omniverse & Isaac Sim

### What is NVIDIA Omniverse?

**NVIDIA Omniverse** is a platform for building and operating metaverse applications, designed for 3D content creation and simulation. It provides:

- **Universal Scene Description (USD)**: An open framework for 3D scene interchange
- **RTX-powered rendering**: Physically accurate ray tracing for photorealistic visuals
- **Nucleus**: Collaborative database for sharing 3D assets across teams
- **Connectors**: Integration with tools like Blender, Maya, and ROS 2

### What is Isaac Sim?

**Isaac Sim** is NVIDIA's robotics simulation platform built on Omniverse. It offers:

| Feature | Description |
|---------|-------------|
| **Physics Simulation** | PhysX 5 for accurate rigid body, articulation, and soft body dynamics |
| **Photorealistic Rendering** | RTX ray tracing for camera sensors and domain randomization |
| **Sensor Simulation** | LiDAR, depth cameras, IMU, and contact sensors |
| **ROS 2 Integration** | Native OmniGraph-based bridge for ROS 2 Humble |
| **Synthetic Data** | Replicator for automated labeling and dataset generation |

### Isaac Sim vs Gazebo: When to Use Each

| Aspect | Isaac Sim | Gazebo |
|--------|-----------|--------|
| **Rendering Quality** | RTX ray tracing (photorealistic) | OpenGL (functional) |
| **Synthetic Data** | Built-in Replicator with auto-labeling | Requires custom plugins |
| **GPU Requirement** | NVIDIA RTX required | CPU-only possible |
| **ROS Integration** | Native via OmniGraph | Native via ros_gz |
| **Best For** | AI training data, perception R&D | Control algorithm testing, multi-robot |

**Rule of thumb**: Use Isaac Sim when you need photorealistic synthetic data for training AI models. Use Gazebo for physics-focused testing and when NVIDIA GPU is unavailable.

---

## Installation & Setup

### System Requirements

Before installation, verify your system meets these requirements:

```bash title="Check NVIDIA GPU"
nvidia-smi
```

**Expected Output**:
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.xx.xx    Driver Version: 535.xx.xx    CUDA Version: 12.x    |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  NVIDIA RTX 3060 ...  On  | 00000000:01:00.0  On |                  N/A |
|  0%   45C    P8    15W / 170W |   1234MiB / 12288MiB |      2%      Default |
+-------------------------------+----------------------+----------------------+
```

### Installing Omniverse Launcher

1. **Download** Omniverse Launcher from [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/):

```bash title="Download Launcher"
# Navigate to your Downloads directory
cd ~/Downloads

# Make the AppImage executable
chmod +x omniverse-launcher-linux.AppImage

# Run the launcher
./omniverse-launcher-linux.AppImage
```

2. **Sign in** with your NVIDIA Developer account (free registration required)

3. **Install Isaac Sim** from the Exchange tab:
   - Search for "Isaac Sim"
   - Select version **2023.1.1** or later
   - Click "Install" and wait for download (~15GB)

### Launching Isaac Sim

```python title="isaac_sim_launch.py"
#!/usr/bin/env python3
"""Launch Isaac Sim in headless mode for synthetic data generation."""

from omni.isaac.kit import SimulationApp

# Configure headless mode (no GUI, faster for data generation)
CONFIG = {
    "headless": True,
    "width": 1280,
    "height": 720,
    "renderer": "RayTracedLighting",  # Use RTX for photorealism
}

# Initialize the simulation application
simulation_app = SimulationApp(CONFIG)

# Now we can import Isaac Sim modules
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage

# Create simulation world
world = World(stage_units_in_meters=1.0)

print("Isaac Sim initialized successfully!")
print(f"Stage units: {world.stage.GetMetadata()}")

# Keep simulation running
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

**Expected Output**:
```
[Info] [omni.isaac.kit] Isaac Sim Version: 2023.1.1
[Info] [omni.isaac.kit] Rendering: RayTracedLighting
Isaac Sim initialized successfully!
Stage units: {'metersPerUnit': 1.0, 'upAxis': 'Z'}
```

---

## Loading Humanoid Robots (USD Format)

### USD vs URDF

| Format | Description | Strengths |
|--------|-------------|-----------|
| **URDF** | Unified Robot Description Format (XML) | ROS standard, widely supported |
| **USD** | Universal Scene Description | Rich materials, physics, collaboration |

Isaac Sim uses USD natively but can import URDF files with automatic conversion.

### Importing URDF to Isaac Sim

```python title="load_humanoid.py"
#!/usr/bin/env python3
"""Load a humanoid robot from URDF into Isaac Sim."""

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.urdf import _urdf

# Enable URDF extension
enable_extension("omni.isaac.urdf")

# Create world
world = World()

# Configure URDF import settings
urdf_interface = _urdf.acquire_urdf_interface()

import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = False  # Allow robot to move
import_config.make_default_prim = True
import_config.create_physics_scene = True

# Path to your URDF file (use the humanoid from Module 1)
urdf_path = "/path/to/humanoid_robot.urdf"

# Import URDF and convert to USD
result = urdf_interface.parse_urdf(urdf_path, import_config)
robot_prim_path = urdf_interface.import_robot(
    urdf_path,
    result,
    import_config,
    "/World/HumanoidRobot"
)

print(f"Robot loaded at: {robot_prim_path}")

# Initialize the world
world.reset()

# Run simulation
for i in range(1000):
    world.step(render=True)

simulation_app.close()
```

**Expected Output**:
```
[Info] [omni.isaac.urdf] Parsing URDF: /path/to/humanoid_robot.urdf
[Info] [omni.isaac.urdf] Found 22 links and 21 joints
[Info] [omni.isaac.urdf] Creating articulation at /World/HumanoidRobot
Robot loaded at: /World/HumanoidRobot
```

### Using Pre-built Robot Assets from NGC

NVIDIA NGC provides ready-to-use robot assets:

```python title="Load from NGC"
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Get path to Isaac Sim assets on NGC
assets_root = get_assets_root_path()
robot_usd = f"{assets_root}/Isaac/Robots/Humanoid/humanoid.usd"

# Add robot to stage
add_reference_to_stage(usd_path=robot_usd, prim_path="/World/Humanoid")
```

---

## Synthetic Data with Replicator

### What is Replicator?

**Replicator** is Isaac Sim's synthetic data generation (SDG) framework. It automates:

- **Scene randomization**: Lighting, textures, object positions
- **Annotation generation**: Bounding boxes, segmentation masks, depth
- **Data export**: COCO, KITTI, and custom formats

### Replicator Architecture

```
┌─────────────────────────────────────────────────────┐
│                  Replicator Pipeline                │
├─────────────────────────────────────────────────────┤
│  Randomizers          │  Annotators    │  Writers  │
│  ├── Lighting         │  ├── BBox 2D   │  ├── COCO │
│  ├── Texture          │  ├── BBox 3D   │  ├── KITTI│
│  ├── Camera Pose      │  ├── Semantic  │  └── PNG  │
│  ├── Object Position  │  ├── Instance  │           │
│  └── Physics          │  └── Depth     │           │
└─────────────────────────────────────────────────────┘
```

### Creating a Synthetic Data Pipeline

```python title="replicator_pipeline.py"
#!/usr/bin/env python3
"""Complete synthetic data generation pipeline using Replicator."""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.replicator.core as rep
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage

# Create world and load robot
world = World()
add_reference_to_stage(
    usd_path="/Isaac/Robots/Humanoid/humanoid.usd",
    prim_path="/World/Robot"
)

# Setup camera for data capture
camera = rep.create.camera(
    position=(3.0, 3.0, 2.0),
    look_at=(0, 0, 1.0),
    focal_length=24.0
)

# Create render product (what the camera sees)
render_product = rep.create.render_product(camera, (1280, 720))

# Configure annotators
with rep.AnnotatorRegistry.register():
    # 2D bounding boxes for object detection
    rep.annotators.get("bounding_box_2d_tight").attach(render_product)

    # Semantic segmentation masks
    rep.annotators.get("semantic_segmentation").attach(render_product)

    # Depth images for 3D perception
    rep.annotators.get("distance_to_camera").attach(render_product)

    # Instance segmentation for multi-object scenes
    rep.annotators.get("instance_segmentation").attach(render_product)

# Configure output writer (COCO format)
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="./synthetic_data",
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=True,
    distance_to_camera=True
)
writer.attach(render_product)

# Generate 100 frames of data
with rep.trigger.on_frame(num_frames=100):
    rep.randomizer.rotation(rep.get.prims(path_pattern="/World/Robot"))

# Run the generation
rep.orchestrator.run()

print("Synthetic data generation complete!")
print("Output saved to: ./synthetic_data/")

simulation_app.close()
```

**Expected Output**:
```
[Info] [Replicator] Starting data generation...
[Info] [Replicator] Frame 1/100 - Annotations: bbox, semantic, depth
[Info] [Replicator] Frame 50/100 - Annotations: bbox, semantic, depth
[Info] [Replicator] Frame 100/100 - Annotations: bbox, semantic, depth
Synthetic data generation complete!
Output saved to: ./synthetic_data/
```

**Generated Files**:
```
synthetic_data/
├── rgb/
│   ├── 000000.png
│   ├── 000001.png
│   └── ...
├── bounding_box_2d_tight/
│   └── 000000.json
├── semantic_segmentation/
│   └── 000000.png
├── distance_to_camera/
│   └── 000000.npy
└── annotations.json  # COCO format
```

---

## Domain Randomization

### Why Domain Randomization?

**Domain randomization** varies visual and physical properties during data generation to:

- Bridge the **sim-to-real gap** (models trained on synthetic data work on real sensors)
- Increase **dataset diversity** without manual labeling
- Improve model **generalization** to unseen environments

### Key Randomization Strategies

| Randomizer | Purpose | Real-World Variation Simulated |
|------------|---------|-------------------------------|
| **Lighting** | HDR intensity, color temperature | Indoor/outdoor, time of day |
| **Texture** | Surface materials, colors | Different environments |
| **Camera** | Position, FOV, noise | Sensor mounting variations |
| **Physics** | Friction, mass | Manufacturing tolerances |
| **Distractors** | Background objects | Cluttered environments |

### Configuring Randomizers

```python title="domain_randomization.py"
#!/usr/bin/env python3
"""Configure domain randomization for robust model training."""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.replicator.core as rep

# Get references to scene elements
robot = rep.get.prims(path_pattern="/World/Robot")
lights = rep.get.prims(path_pattern="/World/Lights/*")
floor = rep.get.prims(path_pattern="/World/Floor")

# Define randomization graph
with rep.trigger.on_frame(num_frames=1000):

    # Randomize lighting intensity and color
    with lights:
        rep.modify.attribute(
            "intensity",
            rep.distribution.uniform(500, 5000)  # Lux range
        )
        rep.modify.attribute(
            "color",
            rep.distribution.uniform((0.8, 0.8, 0.8), (1.0, 1.0, 1.0))
        )

    # Randomize robot pose
    with robot:
        rep.modify.pose(
            position=rep.distribution.uniform(
                (-0.5, -0.5, 0),
                (0.5, 0.5, 0)
            ),
            rotation=rep.distribution.uniform(
                (0, 0, -180),
                (0, 0, 180)
            )
        )

    # Randomize floor texture
    with floor:
        rep.randomizer.texture(
            textures=[
                "/Isaac/Materials/Textures/Patterns/concrete.jpg",
                "/Isaac/Materials/Textures/Patterns/wood.jpg",
                "/Isaac/Materials/Textures/Patterns/tile.jpg",
            ]
        )

    # Add random distractor objects (clutter)
    rep.randomizer.scatter_2d(
        rep.get.prims(path_pattern="/World/Distractors/*"),
        surface=floor,
        num_objects=rep.distribution.uniform(5, 15)
    )

# Camera noise (simulates real sensor imperfections)
camera = rep.get.prims(path_pattern="/World/Camera")
with camera:
    rep.modify.attribute(
        "motion_blur",
        rep.distribution.uniform(0, 0.2)
    )

print("Domain randomization configured!")
print("Randomizers: lighting, pose, texture, distractors, camera noise")

simulation_app.close()
```

**Expected Output**:
```
[Info] [Replicator] Registered randomizers:
  - Lighting: intensity uniform(500, 5000), color uniform
  - Robot pose: position uniform, rotation uniform
  - Floor texture: 3 materials
  - Distractors: scatter_2d, 5-15 objects
  - Camera: motion_blur uniform(0, 0.2)
Domain randomization configured!
Randomizers: lighting, pose, texture, distractors, camera noise
```

---

## Exporting Training Datasets

### COCO Format Export

The **COCO format** is standard for object detection and segmentation tasks:

```json title="annotations.json (COCO format)"
{
  "images": [
    {"id": 1, "file_name": "000000.png", "width": 1280, "height": 720}
  ],
  "annotations": [
    {
      "id": 1,
      "image_id": 1,
      "category_id": 1,
      "bbox": [420, 180, 200, 400],
      "segmentation": [[420, 180, 620, 180, 620, 580, 420, 580]],
      "area": 80000,
      "iscrowd": 0
    }
  ],
  "categories": [
    {"id": 1, "name": "humanoid_robot", "supercategory": "robot"}
  ]
}
```

### KITTI Format Export

The **KITTI format** is standard for autonomous driving and 3D perception:

```python title="KITTI Writer Configuration"
writer = rep.WriterRegistry.get("KittiWriter")
writer.initialize(
    output_dir="./kitti_data",
    semantic_types=["class", "instance"],
    image_output_format="png",
    colorize_instance_segmentation=True
)
```

**KITTI Output Structure**:
```
kitti_data/
├── image_2/           # RGB images
├── label_2/           # 2D labels (txt)
├── velodyne/          # Point clouds (if LiDAR simulated)
└── calib/             # Camera calibration
```

### Custom Writers

For specialized formats, create custom writers:

```python title="Custom Writer Example"
import omni.replicator.core as rep

class CustomWriter(rep.Writer):
    def __init__(self, output_dir):
        self.output_dir = output_dir
        self.frame_id = 0

    def write(self, data):
        # Access annotation data
        rgb = data["rgb"]
        bbox = data["bounding_box_2d_tight"]
        semantic = data["semantic_segmentation"]

        # Custom processing and export
        # ... your format here ...

        self.frame_id += 1

# Register and use
rep.WriterRegistry.register(CustomWriter)
```

---

## Key Takeaways

1. **Isaac Sim** is NVIDIA's robotics simulation platform built on Omniverse, providing photorealistic rendering and integrated synthetic data generation.

2. **USD format** is native to Isaac Sim but URDFs can be imported with automatic conversion, allowing you to reuse robot models from ROS.

3. **Replicator** automates synthetic data generation with randomizers (scene variation) and annotators (auto-labeling), eliminating manual labeling effort.

4. **Domain randomization** is essential for bridging the sim-to-real gap—vary lighting, textures, poses, and add distractors to improve model generalization.

5. **COCO and KITTI formats** are industry standards for object detection and 3D perception training data export.

**Next Steps**: In Chapter 2, you'll learn to use Isaac ROS for GPU-accelerated perception on the synthetic data generated here.
