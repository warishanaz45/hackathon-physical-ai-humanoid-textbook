# Data Model: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature**: 002-gazebo-unity-module-2
**Date**: 2025-12-29
**Type**: Documentation Content Structure

## Overview

This data model defines the structure for Docusaurus documentation content, not database entities. It specifies the schema for chapter files, frontmatter, and content organization.

## Content Entities

### Chapter Document

Represents a single chapter markdown file in the Docusaurus docs folder.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `sidebar_position` | integer | Yes | Order in sidebar (1, 2, 3) |
| `title` | string | Yes | Chapter title displayed in sidebar |
| `description` | string | Yes | Meta description for SEO |
| `keywords` | string[] | Yes | Keywords for search indexing |

**Frontmatter Schema**:
```yaml
---
sidebar_position: 1
title: "Chapter Title"
description: "Brief description for SEO"
keywords: [keyword1, keyword2, keyword3]
---
```

**Content Sections** (in order):
1. H1 Title (matches frontmatter title)
2. "What You'll Learn" section with bullet list
3. Main content sections (H2 headings)
4. "Key Takeaways" section with bullet list
5. "Next Steps" section with navigation link

---

### Module Category

Represents the _category_.json file for the module folder.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `label` | string | Yes | Display name in sidebar |
| `position` | integer | Yes | Order among modules (2 for this module) |
| `collapsed` | boolean | Yes | Whether collapsed by default |
| `collapsible` | boolean | Yes | Whether can be collapsed |
| `description` | string | No | Module description tooltip |

**Schema**:
```json
{
  "label": "Module 2: The Digital Twin",
  "position": 2,
  "collapsed": false,
  "collapsible": true,
  "description": "Physics simulation with Gazebo, digital twins with Unity, and sensor simulation"
}
```

---

### Code Example Block

Represents a code block within chapter content.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `language` | string | Yes | Syntax highlighting language |
| `title` | string | Yes | Filename or description |
| `content` | string | Yes | The code content |
| `expected_output` | string | No | What running this produces |

**Markdown Format**:
````markdown
```python title="example.py"
# Code content here
```

**Expected output:**
```text
Output here
```
````

---

### Comparison Table

Represents a comparison table for concepts.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `headers` | string[] | Yes | Column headers |
| `rows` | object[] | Yes | Data rows |

**Markdown Format**:
```markdown
| Aspect | Gazebo | Unity |
|--------|--------|-------|
| Physics | Engineering-grade | Game-grade |
| Rendering | Basic | High-fidelity |
```

---

## Chapter-Specific Entities

### Chapter 1: Physics Simulation with Gazebo

**Key Concepts to Define**:
- SDF (Simulation Description Format)
- World file
- Physics engine (DART, Bullet, ODE)
- Gazebo plugin
- ros_gz_bridge
- Joint controller

**Code Examples Required**:
- SDF world file with robot
- URDF with Gazebo extensions
- Launch file for Gazebo + ROS 2
- Joint position command example

---

### Chapter 2: Digital Twins & HRI in Unity

**Key Concepts to Define**:
- Digital twin
- ROS-TCP-Connector
- ArticulationBody
- Human-Robot Interaction (HRI)
- Unity scene
- ROS message subscriber/publisher

**Code Examples Required**:
- Unity project setup steps
- ROS-TCP endpoint configuration
- URDF import workflow
- C# ROS subscriber script
- C# ROS publisher script

---

### Chapter 3: Sensor Simulation & Validation

**Key Concepts to Define**:
- LiDAR sensor model
- Depth camera
- IMU (Inertial Measurement Unit)
- Sensor noise model
- Point cloud
- Domain randomization

**Code Examples Required**:
- LiDAR sensor SDF configuration
- Depth camera SDF configuration
- IMU SDF configuration
- RViz visualization commands
- Noise model configuration

---

## File Structure

```text
frontend_book/docs/module-2-simulation/
├── _category_.json              # Module metadata
├── 01-gazebo-physics.md         # Chapter 1: Physics Simulation
├── 02-unity-digital-twins.md    # Chapter 2: Digital Twins & HRI
└── 03-sensor-simulation.md      # Chapter 3: Sensor Simulation
```

## Validation Rules

### Frontmatter Validation
- `sidebar_position` must be unique within module
- `title` must not exceed 60 characters
- `description` must be 50-160 characters for SEO
- `keywords` must have 3-7 items

### Content Validation
- Every code block must have `title` attribute
- Every code block must have language identifier
- "What You'll Learn" must have 3-6 bullet points
- "Key Takeaways" must have 4-6 bullet points
- All technical terms defined on first use

### Cross-Reference Validation
- Internal links must use relative paths (`./02-chapter.md`)
- External links must be to official documentation
- Module 1 concepts referenced must link back

## State Transitions

Not applicable - documentation content is static (no state machine).

## Relationships

```text
Module 2
├── Chapter 1 (Gazebo) → builds on Module 1 URDF
├── Chapter 2 (Unity) → imports URDF from Module 1
└── Chapter 3 (Sensors) → applies to both Gazebo and Unity
```
