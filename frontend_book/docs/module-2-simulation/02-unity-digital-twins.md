---
sidebar_position: 2
title: "Digital Twins & HRI in Unity"
description: "Build high-fidelity digital twins and human-robot interaction scenarios in Unity with ROS 2"
keywords: [Unity, digital twin, HRI, ROS-TCP-Connector, visualization, human-robot interaction]
---

# Digital Twins & HRI in Unity

## What You'll Learn

By the end of this chapter, you will be able to:

- Understand why Unity is valuable for robotics visualization and HRI
- Set up Unity with ROS-TCP-Connector for ROS 2 communication
- Import URDF robot models into Unity using the URDF Importer
- Create realistic environments with lighting and materials
- Build human-robot interaction (HRI) scenarios with avatars
- Implement bidirectional communication between Unity and ROS 2

---

## Why Unity for Digital Twins?

While Gazebo excels at physics simulation, **Unity** provides capabilities that make it essential for certain robotics applications.

### Digital Twin Definition

A **digital twin** is a virtual replica of a physical system that:
- Mirrors the real-world counterpart in real-time
- Enables visualization, monitoring, and prediction
- Supports "what-if" scenario testing
- Provides high-fidelity rendering for human perception studies

### Unity vs Gazebo: When to Use Each

| Capability | Gazebo | Unity | Winner |
|------------|--------|-------|--------|
| **Physics accuracy** | Engineering-grade | Game-grade | Gazebo |
| **Visual fidelity** | Basic | Photorealistic | Unity |
| **Human avatars** | Limited | Excellent | Unity |
| **VR/AR support** | None | Native | Unity |
| **Synthetic data generation** | Basic | Advanced | Unity |
| **Real-time performance** | Good | Excellent | Unity |
| **Learning curve** | Moderate | Steeper | Gazebo |
| **ROS integration** | Native | Via bridge | Gazebo |

### Use Cases for Unity in Robotics

| Use Case | Why Unity |
|----------|-----------|
| **Training data generation** | Photorealistic rendering, domain randomization |
| **HRI studies** | Realistic human avatars, emotion display |
| **Visualization dashboards** | Beautiful UI, real-time monitoring |
| **VR teleoperation** | Native VR headset support |
| **Marketing/demos** | High-quality renders for presentations |
| **Perception testing** | Realistic lighting, materials, and occlusion |

---

## Setting Up Unity with ROS 2

Unity connects to ROS 2 through the **ROS-TCP-Connector** package from Unity Robotics Hub.

### Prerequisites

- Unity Hub installed
- Unity Editor 2022.3 LTS (or newer)
- ROS 2 Humble running on Ubuntu (can be on same machine or networked)

### Installation Steps

1. **Create a new Unity project** (3D Core template)

2. **Add the Unity Robotics packages** via Package Manager:
   - Open Window → Package Manager
   - Click "+" → "Add package from git URL"
   - Add these packages:
     ```text
     https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
     https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
     ```

3. **Configure ROS connection**:
   - Go to Robotics → ROS Settings
   - Set ROS IP Address (Ubuntu machine IP)
   - Set ROS Port (default: 10000)
   - Select Protocol: ROS2

### ROS 2 Side Setup

On your ROS 2 machine, install and run the TCP endpoint:

```bash
# Install ros_tcp_endpoint
sudo apt install ros-humble-ros-tcp-endpoint

# Run the endpoint (replace IP with Unity machine's IP)
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

### Connection Verification

```csharp title="ROSConnectionTest.cs"
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class ROSConnectionTest : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        // Get ROS connection instance
        ros = ROSConnection.GetOrCreateInstance();

        // Register a publisher
        ros.RegisterPublisher<StringMsg>("/unity/status");

        // Send a test message
        var msg = new StringMsg("Unity connected!");
        ros.Publish("/unity/status", msg);

        Debug.Log("ROS connection initialized");
    }
}
```

Verify on ROS 2 side:
```bash
ros2 topic echo /unity/status
```

---

## Importing Robot Models

The **URDF Importer** converts URDF files into Unity GameObjects with proper joint configurations.

### Import Workflow

1. **Copy URDF and meshes** to your Unity project's `Assets/` folder:
   ```text
   Assets/
   └── Robot/
       ├── urdf/
       │   └── humanoid.urdf
       └── meshes/
           ├── torso.stl
           ├── head.stl
           └── ...
   ```

2. **Import the URDF**:
   - Right-click the URDF file in Project window
   - Select "Import Robot from URDF"
   - Configure import settings:
     - **Axis Type**: Y-Up (Unity default)
     - **Mesh Decomposer**: VHACD (for convex collision)
     - **Convex Hulls**: Enable for physics

3. **Verify the import**:
   - Robot appears in Hierarchy
   - Joints use `ArticulationBody` components
   - Colliders are properly configured

### ArticulationBody Configuration

Unity uses **ArticulationBody** instead of Rigidbody for robotic joints, providing more accurate joint physics.

Key properties to configure:

| Property | Description | Typical Value |
|----------|-------------|---------------|
| `Joint Type` | Revolute, Prismatic, etc. | From URDF |
| `Anchor Position` | Joint origin | From URDF |
| `Stiffness` | Position control gain | 10000-100000 |
| `Damping` | Velocity damping | 100-1000 |
| `Force Limit` | Max joint torque | From URDF limits |

---

## Creating Realistic Environments

High-fidelity environments enhance visualization and enable synthetic data generation.

### Lighting Setup

```text
Recommended Lighting Configuration:

1. Directional Light (Sun)
   - Intensity: 1.0-1.5
   - Color: Warm white (#FFF4E5)
   - Shadows: Soft shadows enabled

2. Environment Lighting
   - Source: Skybox or Gradient
   - Ambient Intensity: 0.5-1.0

3. Reflection Probes
   - Place in key areas
   - Update mode: Via scripting (for performance)
```

### Material Setup for Robots

For realistic robot rendering:

```text
Robot Materials:
├── Metal (torso, frame)
│   - Shader: Standard (Metallic)
│   - Metallic: 0.8-1.0
│   - Smoothness: 0.6-0.8
│
├── Plastic (covers, buttons)
│   - Shader: Standard
│   - Metallic: 0.0
│   - Smoothness: 0.3-0.6
│
└── Rubber (grippers, wheels)
    - Shader: Standard
    - Metallic: 0.0
    - Smoothness: 0.1-0.3
```

### Environment Script

```csharp title="EnvironmentSetup.cs"
using UnityEngine;

public class EnvironmentSetup : MonoBehaviour
{
    [Header("Lighting")]
    public Light directionalLight;
    public float lightIntensity = 1.2f;

    [Header("Ground")]
    public GameObject groundPlane;
    public Material groundMaterial;

    void Start()
    {
        SetupLighting();
        SetupGround();
        Debug.Log("Environment setup complete");
    }

    void SetupLighting()
    {
        if (directionalLight != null)
        {
            directionalLight.intensity = lightIntensity;
            directionalLight.shadows = LightShadows.Soft;
            directionalLight.shadowStrength = 0.8f;
        }

        // Ambient lighting
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
        RenderSettings.ambientSkyColor = new Color(0.8f, 0.9f, 1.0f);
        RenderSettings.ambientEquatorColor = new Color(0.6f, 0.6f, 0.6f);
        RenderSettings.ambientGroundColor = new Color(0.4f, 0.4f, 0.4f);
    }

    void SetupGround()
    {
        if (groundPlane != null && groundMaterial != null)
        {
            groundPlane.GetComponent<Renderer>().material = groundMaterial;
        }
    }
}
```

---

## Human-Robot Interaction Setup

Unity excels at creating HRI scenarios with realistic human avatars.

### Human Avatar Options

| Source | Pros | Cons |
|--------|------|------|
| **Unity Asset Store** | Ready to use | Cost, licensing |
| **Mixamo** | Free, animated | Adobe account required |
| **MakeHuman** | Free, customizable | Lower quality |
| **MetaHuman** | Photorealistic | Unreal-focused |

### Basic HRI Scenario

```csharp title="HRIInteraction.cs"
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class HRIInteraction : MonoBehaviour
{
    [Header("References")]
    public Transform humanAvatar;
    public Transform robotEndEffector;

    [Header("Interaction Settings")]
    public float interactionDistance = 1.0f;
    public float handoverHeight = 1.0f;

    private ROSConnection ros;
    private bool isHandoverActive = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointMsg>("/hri/target_position");
    }

    void Update()
    {
        // Check proximity for handover
        float distance = Vector3.Distance(
            humanAvatar.position,
            robotEndEffector.position
        );

        if (distance < interactionDistance && !isHandoverActive)
        {
            InitiateHandover();
        }

        // Publish human position for robot tracking
        PublishHumanPosition();
    }

    void InitiateHandover()
    {
        isHandoverActive = true;
        Debug.Log("Handover initiated - human within range");

        // Calculate handover point (between human and robot)
        Vector3 handoverPoint = Vector3.Lerp(
            humanAvatar.position,
            robotEndEffector.position,
            0.5f
        );
        handoverPoint.y = handoverHeight;

        // Send target to robot
        var targetMsg = new PointMsg
        {
            x = handoverPoint.x,
            y = handoverPoint.z,  // Unity Y -> ROS Z
            z = handoverPoint.y   // Unity Z -> ROS Y
        };
        ros.Publish("/hri/target_position", targetMsg);
    }

    void PublishHumanPosition()
    {
        // Convert Unity coordinates to ROS coordinates
        var posMsg = new PointMsg
        {
            x = humanAvatar.position.x,
            y = humanAvatar.position.z,
            z = humanAvatar.position.y
        };
        ros.Publish("/hri/human_position", posMsg);
    }

    public void CompleteHandover()
    {
        isHandoverActive = false;
        Debug.Log("Handover completed");
    }
}
```

---

## Bidirectional ROS Communication

Real-time communication between Unity and ROS 2 enables live control and monitoring.

### Subscribing to Joint States

```csharp title="JointStateSubscriber.cs"
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;

public class JointStateSubscriber : MonoBehaviour
{
    [Header("Robot Reference")]
    public GameObject robotRoot;

    private ROSConnection ros;
    private Dictionary<string, ArticulationBody> joints;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>("/joint_states", OnJointStateReceived);

        // Build joint dictionary
        joints = new Dictionary<string, ArticulationBody>();
        foreach (var ab in robotRoot.GetComponentsInChildren<ArticulationBody>())
        {
            if (ab.jointType != ArticulationJointType.FixedJoint)
            {
                joints[ab.name] = ab;
            }
        }

        Debug.Log($"Subscribed to /joint_states, tracking {joints.Count} joints");
    }

    void OnJointStateReceived(JointStateMsg msg)
    {
        for (int i = 0; i < msg.name.Length; i++)
        {
            string jointName = msg.name[i];
            if (joints.ContainsKey(jointName))
            {
                // Set joint position
                var drive = joints[jointName].xDrive;
                drive.target = (float)msg.position[i] * Mathf.Rad2Deg;
                joints[jointName].xDrive = drive;
            }
        }
    }
}
```

**Expected behavior:**
```text
# Console output when joint states are received:
Subscribed to /joint_states, tracking 6 joints
```

### Publishing Joint Commands

```csharp title="JointCommandPublisher.cs"
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using System.Collections.Generic;

public class JointCommandPublisher : MonoBehaviour
{
    [Header("Robot Reference")]
    public GameObject robotRoot;

    [Header("Publishing Settings")]
    public float publishRate = 50f;  // Hz

    private ROSConnection ros;
    private Dictionary<string, ArticulationBody> joints;
    private float publishInterval;
    private float lastPublishTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        publishInterval = 1f / publishRate;

        // Build joint dictionary and register publishers
        joints = new Dictionary<string, ArticulationBody>();
        foreach (var ab in robotRoot.GetComponentsInChildren<ArticulationBody>())
        {
            if (ab.jointType != ArticulationJointType.FixedJoint)
            {
                joints[ab.name] = ab;
                string topic = $"/joint_commands/{ab.name}";
                ros.RegisterPublisher<Float64Msg>(topic);
            }
        }

        Debug.Log($"Publishing joint commands for {joints.Count} joints at {publishRate} Hz");
    }

    void Update()
    {
        if (Time.time - lastPublishTime >= publishInterval)
        {
            PublishJointCommands();
            lastPublishTime = Time.time;
        }
    }

    void PublishJointCommands()
    {
        foreach (var kvp in joints)
        {
            string jointName = kvp.Key;
            ArticulationBody joint = kvp.Value;

            // Get current joint position in radians
            float position = joint.jointPosition[0];

            var msg = new Float64Msg { data = position };
            ros.Publish($"/joint_commands/{jointName}", msg);
        }
    }

    // Public method to set joint target from UI or script
    public void SetJointTarget(string jointName, float targetRadians)
    {
        if (joints.ContainsKey(jointName))
        {
            var drive = joints[jointName].xDrive;
            drive.target = targetRadians * Mathf.Rad2Deg;
            joints[jointName].xDrive = drive;

            Debug.Log($"Set {jointName} target to {targetRadians:F2} rad");
        }
    }
}
```

**Expected behavior:**
```text
# Console output when publishing starts:
Publishing joint commands for 6 joints at 50 Hz

# When setting joint target via script:
Set right_shoulder_pitch target to 0.50 rad
```

### Coordinate System Conversion

Unity and ROS use different coordinate systems:

| Axis | Unity | ROS |
|------|-------|-----|
| Forward | Z+ | X+ |
| Left | X- | Y+ |
| Up | Y+ | Z+ |

```csharp title="CoordinateConverter.cs"
public static class CoordinateConverter
{
    // Unity position to ROS position
    public static Vector3 UnityToROS(Vector3 unityPos)
    {
        return new Vector3(unityPos.z, -unityPos.x, unityPos.y);
    }

    // ROS position to Unity position
    public static Vector3 ROSToUnity(Vector3 rosPos)
    {
        return new Vector3(-rosPos.y, rosPos.z, rosPos.x);
    }

    // Unity rotation to ROS rotation (quaternion)
    public static Quaternion UnityToROS(Quaternion unityRot)
    {
        return new Quaternion(-unityRot.z, unityRot.x, -unityRot.y, unityRot.w);
    }
}
```

---

## Key Takeaways

1. **Unity excels at visualization** — use it for HRI, training data, and demonstrations
2. **ROS-TCP-Connector** bridges Unity and ROS 2 with minimal latency
3. **URDF Importer** converts robot descriptions to Unity GameObjects with ArticulationBody joints
4. **Coordinate conversion** is essential — Unity uses Y-up, ROS uses Z-up
5. **Bidirectional communication** enables live control and monitoring
6. **Combine Gazebo + Unity** — Gazebo for physics, Unity for visualization

---

## Next Steps

In the next chapter, [Sensor Simulation & Validation](./03-sensor-simulation.md), you'll learn how to:

- Configure LiDAR sensors with realistic noise models
- Simulate depth cameras and RGB cameras
- Add IMU sensors to your robot
- Visualize sensor data in RViz
- Validate simulated sensors against real specifications
