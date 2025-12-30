# Quickstart: Module 2 Implementation

**Feature**: 002-gazebo-unity-module-2
**Date**: 2025-12-29

## Prerequisites

Before implementing Module 2, ensure:

1. **Module 1 Complete**: `frontend_book/docs/module-1-ros2/` exists with 3 chapters
2. **Docusaurus Running**: `npm start` works in `frontend_book/`
3. **Branch Created**: On branch `002-gazebo-unity-module-2`

## Quick Setup Commands

```bash
# Navigate to project
cd "C:\Users\HP\OneDrive\Desktop\Hackathon-book creation-2025"

# Verify branch
git branch --show-current  # Should show: 002-gazebo-unity-module-2

# Navigate to Docusaurus
cd frontend_book

# Start dev server (optional, for live preview)
npm start
```

## Implementation Steps

### Step 1: Create Module Directory

```bash
# Create module-2 directory
mkdir -p docs/module-2-simulation
```

### Step 2: Create Category Metadata

Create `docs/module-2-simulation/_category_.json`:

```json
{
  "label": "Module 2: The Digital Twin",
  "position": 2,
  "collapsed": false,
  "collapsible": true,
  "description": "Physics simulation with Gazebo, digital twins with Unity, and sensor simulation"
}
```

### Step 3: Create Chapter Files

Create three chapter files with frontmatter:

**01-gazebo-physics.md**:
```yaml
---
sidebar_position: 1
title: "Physics Simulation with Gazebo"
description: "Learn to set up Gazebo Fortress for physics-based humanoid robot simulation with ROS 2"
keywords: [Gazebo, physics simulation, ROS 2, SDF, robotics]
---
```

**02-unity-digital-twins.md**:
```yaml
---
sidebar_position: 2
title: "Digital Twins & HRI in Unity"
description: "Build high-fidelity digital twins and human-robot interaction scenarios in Unity with ROS 2"
keywords: [Unity, digital twin, HRI, ROS-TCP-Connector, visualization]
---
```

**03-sensor-simulation.md**:
```yaml
---
sidebar_position: 3
title: "Sensor Simulation & Validation"
description: "Configure and validate LiDAR, depth cameras, and IMU sensors in simulation"
keywords: [LiDAR, depth camera, IMU, sensor simulation, noise models]
---
```

### Step 4: Write Content

Follow the chapter contracts in `specs/002-gazebo-unity-module-2/contracts/chapter-structure.md`.

For each chapter:
1. Add "What You'll Learn" section
2. Write main content sections
3. Include all required code examples
4. Add "Key Takeaways" section
5. Add navigation link to next chapter

### Step 5: Verify Build

```bash
# Build to check for errors
npm run build

# Preview locally
npm run serve
```

## File Checklist

After implementation, verify these files exist:

```text
frontend_book/docs/module-2-simulation/
├── _category_.json              ✓
├── 01-gazebo-physics.md         ✓
├── 02-unity-digital-twins.md    ✓
└── 03-sensor-simulation.md      ✓
```

## Validation Checklist

- [ ] All chapters have correct frontmatter
- [ ] All code blocks have language and title
- [ ] All code examples have expected outputs where applicable
- [ ] All technical terms defined on first use
- [ ] Navigation links work between chapters
- [ ] `npm run build` passes without errors
- [ ] Module appears in sidebar with correct position

## Common Issues

### Issue: Module not appearing in sidebar
**Fix**: Verify `_category_.json` has `"position": 2` and is valid JSON

### Issue: Code blocks not highlighting
**Fix**: Ensure language identifier is specified (e.g., ```python, ```xml, ```csharp)

### Issue: Broken links
**Fix**: Use relative paths (e.g., `./02-chapter.md`) not absolute paths

## Next Steps After Implementation

1. Run `/sp.tasks` to generate task list
2. Run `/sp.implement` to execute tasks
3. Create PR when complete
