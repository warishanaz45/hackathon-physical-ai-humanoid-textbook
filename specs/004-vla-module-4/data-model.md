# Data Model: Module 4 - Vision-Language-Action (VLA)

**Feature**: 004-vla-module-4
**Date**: 2025-12-29
**Purpose**: Define content structure and entities for VLA module documentation

## Content Entities

### 1. Module Container

| Field | Value |
|-------|-------|
| Name | module-4-vla |
| Label | Module 4: Vision-Language-Action |
| Position | 4 (after module-3-nvidia-isaac) |
| Description | Convergence of LLMs and robotics for voice-controlled humanoid actions |
| Chapters | 3 |
| Word Count | 3000-5000 (constraint from spec) |

### 2. Chapter Entities

#### Chapter 1: Voice-to-Action with Speech Recognition

| Field | Value |
|-------|-------|
| File | `01-voice-to-action.md` |
| Slug | voice-to-action |
| Position | 1 |
| Prerequisites | Modules 1-3 (ROS 2, simulation, NVIDIA Isaac) |
| Hardware | USB microphone, internet connection |
| Target Words | 1000-1500 |

**Section Structure**:
```
1. What You'll Learn
2. Introduction to Voice-Controlled Robotics
   - Why voice interfaces for humanoids
   - The VLA pipeline overview
   - Module prerequisites recap
3. Speech Recognition Fundamentals
   - How speech-to-text works
   - Available approaches (cloud vs local)
   - OpenAI Whisper introduction
4. Setting Up Audio Capture
   - Hardware requirements
   - PyAudio installation
   - Voice activity detection (VAD)
5. Implementing Whisper Transcription
   - Model selection (tiny to large)
   - Real-time transcription
   - Handling errors and unclear speech
6. Publishing to ROS 2
   - Creating the voice_command topic
   - Message types and publishing
   - Integration with ROS 2 ecosystem
7. Key Takeaways
```

**Code Examples**:
- `audio_capture.py` - PyAudio microphone capture with VAD
- `whisper_node.py` - ROS 2 node wrapping Whisper transcription
- `voice_command_publisher.py` - Complete voice capture + publish node

---

#### Chapter 2: LLM-Based Cognitive Planning

| Field | Value |
|-------|-------|
| File | `02-llm-cognitive-planning.md` |
| Slug | llm-cognitive-planning |
| Position | 2 |
| Prerequisites | Chapter 1 (voice commands available) |
| Hardware | Internet connection (API access) |
| Target Words | 1200-1800 |

**Section Structure**:
```
1. What You'll Learn
2. LLMs as Robot Planners
   - From language understanding to action planning
   - Why LLMs for robotics
   - Capabilities and limitations
3. Prompt Engineering for Robotics
   - System prompt design
   - Role and constraint definition
   - Output format specification
4. Generating Action Plans
   - JSON schema for action plans
   - Action primitives (navigate, gesture, pick, place)
   - Handling complex multi-step commands
5. Parsing and Validating LLM Output
   - JSON parsing in Python
   - Schema validation
   - Error handling and retry logic
6. Mapping Actions to ROS 2
   - Action primitives to ROS 2 actions
   - Nav2 integration (NavigateToPose)
   - Custom action servers
7. Safety Considerations
   - Capability checking
   - Invalid command detection
   - Action validation filters
8. Key Takeaways
```

**Code Examples**:
- `llm_planner_node.py` - ROS 2 node that calls LLM API
- `action_parser.py` - Parse LLM JSON output to action primitives
- `system_prompt.txt` - Example system prompt for robot planning
- `action_executor.py` - Execute parsed actions via ROS 2 action clients

---

#### Chapter 3: Capstone - Voice-Controlled Humanoid

| Field | Value |
|-------|-------|
| File | `03-capstone-voice-humanoid.md` |
| Slug | capstone-voice-humanoid |
| Position | 3 |
| Prerequisites | Chapters 1-2, Module 3 (Nav2) |
| Hardware | Full setup from Modules 1-4 |
| Target Words | 1000-1500 |

**Section Structure**:
```
1. What You'll Learn
2. Capstone Overview
   - Project goals and scope
   - End-to-end pipeline recap
   - What you'll build
3. Integrating the VLA Pipeline
   - Connecting voice → LLM → actions
   - Launch file configuration
   - System startup sequence
4. Navigation via Voice Commands
   - Mapping voice to Nav2 goals
   - Integration with Module 3 Nav2 setup
   - Testing navigation commands
5. Gesture Execution
   - Simple gesture action server
   - Joint controller commands
   - Wave, point, nod gestures
6. Multi-Step Command Demo
   - Example: "Go to the table and wave hello"
   - LLM planning the sequence
   - Execution and monitoring
7. Testing and Validation
   - End-to-end testing approach
   - Common issues and fixes
   - Performance considerations
8. Key Takeaways
9. Where to Go from Here
   - Advanced topics (vision integration)
   - Real hardware deployment
   - Further reading
```

**Code Examples**:
- `vla_pipeline.launch.py` - Complete VLA launch file
- `gesture_action_server.py` - Simple gesture execution
- `capstone_demo.py` - End-to-end demonstration script

---

## Content Relationships

```
Module 1 (ROS 2)
     ↓ (provides)
     ├── Topics/Services → Voice command publishing
     ├── Actions → Gesture action servers
     └── Node lifecycle → VLA node management

Module 2 (Simulation)
     ↓ (provides)
     ├── Gazebo → Testing voice commands in simulation
     └── RViz → Visualizing robot state

Module 3 (NVIDIA Isaac)
     ↓ (provides)
     ├── Nav2 → Navigation action server
     ├── VSLAM → Robot localization
     └── Perception → Environment awareness

Module 4 (VLA)
     ├── Chapter 1: Voice Input → Provides text commands
     ├── Chapter 2: LLM Planning → Generates action plans
     └── Chapter 3: Capstone → Integrates all modules
```

## Docusaurus Metadata

### _category_.json
```json
{
  "label": "Module 4: Vision-Language-Action",
  "position": 4,
  "link": {
    "type": "generated-index",
    "description": "Convergence of LLMs and robotics for voice-controlled humanoid actions"
  }
}
```

### Chapter Frontmatter Template
```yaml
---
sidebar_position: [1|2|3]
title: [Chapter Title]
description: [Brief description for SEO]
keywords: [vla, voice, llm, robotics, whisper, openai, ...]
---
```

## Key Entities (Domain Model)

### Voice Command
| Field | Type | Description |
|-------|------|-------------|
| text | string | Transcribed speech |
| confidence | float | Recognition confidence (0-1) |
| timestamp | time | When captured |
| language | string | Detected language |

### Action Plan
| Field | Type | Description |
|-------|------|-------------|
| plan_id | string | Unique identifier |
| command | string | Original voice command |
| actions | Action[] | Sequence of action primitives |
| safety_check | bool | Safety validation passed |

### Action Primitive
| Field | Type | Description |
|-------|------|-------------|
| action_type | enum | navigate, gesture, pick, place, wait |
| parameters | object | Action-specific parameters |
| timeout_sec | float | Maximum execution time |
| status | enum | pending, executing, completed, failed |

### VLA Pipeline State
| State | Description |
|-------|-------------|
| IDLE | Waiting for voice command |
| LISTENING | Capturing audio |
| TRANSCRIBING | Running Whisper |
| PLANNING | Calling LLM |
| EXECUTING | Running action sequence |
| COMPLETED | Action sequence finished |
| ERROR | Error occurred |

## Validation Rules

| Rule | Applies To | Check |
|------|-----------|-------|
| Prerequisites listed | Each chapter intro | Must reference prior chapters |
| Hardware requirements | Chapter 1 intro | Must include microphone |
| Code has expected output | All code blocks | Must show terminal output |
| Technical terms defined | First occurrence | Must include inline definition |
| Word count | All chapters | Total 3000-5000 words |
| Sources cited | Technical claims | Official documentation links |

## Success Criteria Mapping

| Criterion | Chapter | Validation |
|-----------|---------|------------|
| SC-001: Voice capture in 5 min | Chapter 1 | Audio capture example works |
| SC-002: >90% accuracy | Chapter 1 | Clear speech transcription |
| SC-003: Action plan in 10 min | Chapter 2 | LLM planner example works |
| SC-004: Valid ROS 2 actions | Chapter 2 | Action parser output |
| SC-005: Capstone demo in 20 min | Chapter 3 | End-to-end demo works |
| SC-006: Articulate VLA architecture | All | Clear explanations |
| SC-007: All examples run | All | Code tested |
| SC-008: Explain 3 challenges | Chapter 2, 3 | Challenges documented |
