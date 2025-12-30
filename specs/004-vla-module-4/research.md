# Research: Module 4 - Vision-Language-Action (VLA)

**Feature**: 004-vla-module-4
**Date**: 2025-12-29
**Purpose**: Resolve technical unknowns and document best practices for VLA module documentation

## Research Summary

This module covers the convergence of Large Language Models (LLMs) and robotics for voice-controlled autonomous humanoid actions. The research resolves technology choices, integration patterns, and best practices for speech recognition, LLM-based planning, and end-to-end voice-to-action pipelines.

---

## 1. Speech Recognition Approach

### Decision
Use **OpenAI Whisper** (open-source) for speech-to-text transcription.

### Rationale
- OpenAI Whisper is state-of-the-art for multilingual speech recognition
- Available as open-source with multiple model sizes (tiny to large)
- Can run locally without API costs for development
- Achieves >90% accuracy for clear English speech
- Well-documented with Python integration

### Model Size Recommendations
| Model | Parameters | Speed | Accuracy | Use Case |
|-------|------------|-------|----------|----------|
| tiny | 39M | Real-time | ~80% | Quick testing |
| base | 74M | Fast | ~85% | Development |
| small | 244M | Moderate | ~90% | Production (local) |
| medium | 769M | Slow | ~93% | High accuracy |
| large-v3 | 1.5B | Very slow | ~95% | Maximum accuracy |

### Alternatives Considered
| Option | Pros | Cons |
|--------|------|------|
| OpenAI Whisper (local) | Free, offline, accurate | Requires GPU for large models |
| OpenAI Whisper API | Easy setup, hosted | API costs, requires internet |
| Google Speech-to-Text | Real-time streaming | API costs, Google dependency |
| Azure Speech | Enterprise features | Microsoft dependency, costs |
| Vosk | Lightweight, offline | Lower accuracy than Whisper |

### References
- [OpenAI Whisper GitHub](https://github.com/openai/whisper)
- [Whisper Documentation](https://github.com/openai/whisper/blob/main/README.md)

---

## 2. LLM for Cognitive Planning

### Decision
Use **OpenAI GPT-4** (or GPT-4o) via API for cognitive planning, with local LLM fallback option.

### Rationale
- GPT-4 provides best-in-class reasoning for complex planning tasks
- Strong instruction following for structured output (JSON action plans)
- Well-documented API with Python SDK
- Can be replaced with local models (Llama, Mistral) for offline scenarios
- Prompt engineering can achieve consistent structured outputs

### LLM Integration Pattern
```
User Voice Command
    ↓
Whisper (Speech-to-Text)
    ↓
Command Text
    ↓
LLM (GPT-4 with system prompt)
    ↓
Structured Action Plan (JSON)
    ↓
Action Parser
    ↓
ROS 2 Action Clients
```

### System Prompt Strategy
1. **Role Definition**: Define LLM as a robot task planner
2. **Capability Constraints**: List available robot actions (navigate, pick, place, wave)
3. **Output Format**: Enforce JSON schema for action sequences
4. **Safety Rules**: Include guardrails against unsafe commands

### Alternatives Considered
| Option | Pros | Cons |
|--------|------|------|
| GPT-4 / GPT-4o | Best reasoning, structured output | API costs, internet required |
| Claude 3.5 | Strong reasoning | API costs |
| Llama 3.1 70B | Local, no API costs | High hardware requirements |
| Mistral Large | Good balance | Less tested for robotics |
| GPT-3.5-turbo | Cheaper, faster | Less reliable for complex planning |

### References
- [OpenAI API Documentation](https://platform.openai.com/docs/api-reference)
- [OpenAI Cookbook - Function Calling](https://cookbook.openai.com/examples/how_to_call_functions_with_chat_models)

---

## 3. ROS 2 Integration Architecture

### Decision
Use **ROS 2 Humble** with action interfaces for robot command execution.

### Rationale
- Consistent with Modules 1-3 (ROS 2 Humble throughout)
- Action interfaces provide feedback and preemption for long-running tasks
- Nav2 action servers already available from Module 3
- Standard pattern for humanoid robot control

### VLA Pipeline Architecture
```
┌─────────────────────────────────────────────────────────┐
│                     VLA Pipeline                         │
├─────────────────────────────────────────────────────────┤
│  ┌──────────┐    ┌──────────┐    ┌──────────────────┐  │
│  │ Audio    │───▶│ Whisper  │───▶│ /voice_command   │  │
│  │ Capture  │    │ Node     │    │ (topic)          │  │
│  └──────────┘    └──────────┘    └────────┬─────────┘  │
│                                           │            │
│                                           ▼            │
│                              ┌──────────────────┐      │
│                              │ LLM Planner      │      │
│                              │ Node             │      │
│                              └────────┬─────────┘      │
│                                       │                │
│                                       ▼                │
│                              ┌──────────────────┐      │
│                              │ /action_plan     │      │
│                              │ (topic)          │      │
│                              └────────┬─────────┘      │
│                                       │                │
│                                       ▼                │
│  ┌──────────────┐    ┌────────────────────────────┐   │
│  │ Action       │◀───│ Action Executor Node        │   │
│  │ Servers      │    │ (parses plan, calls actions)│   │
│  │ - Nav2       │    └────────────────────────────┘   │
│  │ - Gesture    │                                      │
│  │ - Manipulate │                                      │
│  └──────────────┘                                      │
└─────────────────────────────────────────────────────────┘
```

### ROS 2 Interfaces
| Interface | Type | Purpose |
|-----------|------|---------|
| `/voice_command` | `std_msgs/String` | Transcribed voice command |
| `/action_plan` | Custom msg | Structured action sequence |
| `/navigate_to_pose` | Nav2 Action | Navigation from Module 3 |
| `/execute_gesture` | Custom Action | Humanoid gestures |

### References
- [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions.html)
- [Nav2 Action Servers](https://navigation.ros.org/concepts/index.html)

---

## 4. Audio Capture for Voice Commands

### Decision
Use **PyAudio** with voice activity detection (VAD) for audio capture.

### Rationale
- PyAudio is the standard Python audio I/O library
- Works across Linux, Windows, macOS
- Can integrate with webrtcvad for voice activity detection
- Simple integration with Whisper for transcription

### Audio Pipeline
```
Microphone Input (16kHz, mono)
    ↓
Voice Activity Detection (VAD)
    ↓
Audio Buffer (when speech detected)
    ↓
Whisper Transcription
    ↓
Text Output
```

### Configuration Parameters
| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Sample rate | 16000 Hz | Optimal for Whisper |
| Channels | 1 (mono) | Sufficient for speech |
| Chunk size | 1024 samples | Balance latency/efficiency |
| VAD aggressiveness | 2 (of 0-3) | Filter noise, keep speech |

### Alternatives Considered
| Option | Pros | Cons |
|--------|------|------|
| PyAudio + VAD | Standard, cross-platform | Requires PortAudio |
| sounddevice | Pure Python | Less common |
| ROS 2 audio_common | ROS-native | More complex setup |
| Web browser + WebRTC | Works anywhere | Requires browser |

### References
- [PyAudio Documentation](https://people.csail.mit.edu/hubert/pyaudio/)
- [webrtcvad GitHub](https://github.com/wiseman/py-webrtcvad)

---

## 5. Action Plan Format and Parsing

### Decision
Use **JSON schema** for action plans with predefined action primitives.

### Rationale
- JSON is easy to parse in Python
- LLMs can be prompted to output valid JSON
- Schema validation ensures safety
- Extensible for new action types

### Action Plan Schema
```json
{
  "plan_id": "string",
  "command": "original voice command",
  "confidence": 0.95,
  "actions": [
    {
      "action_type": "navigate",
      "parameters": {
        "target": "kitchen",
        "coordinates": [2.5, 3.0, 0.0]
      },
      "timeout_sec": 60
    },
    {
      "action_type": "gesture",
      "parameters": {
        "gesture_name": "wave",
        "duration_sec": 3.0
      },
      "timeout_sec": 10
    }
  ],
  "safety_check": true
}
```

### Supported Action Primitives
| Primitive | Parameters | Maps To |
|-----------|------------|---------|
| `navigate` | target, coordinates | Nav2 NavigateToPose |
| `gesture` | gesture_name, duration | Custom gesture action |
| `pick` | object_name, location | Manipulation action |
| `place` | object_name, location | Manipulation action |
| `speak` | text | Text-to-speech |
| `wait` | duration_sec | Timer |

### References
- [JSON Schema](https://json-schema.org/)
- [OpenAI Function Calling](https://platform.openai.com/docs/guides/function-calling)

---

## 6. Safety and Error Handling

### Decision
Implement **multi-layer safety checks** at pipeline stages.

### Rationale
- LLMs can generate unsafe or impossible commands
- Robot actions have physical consequences
- Network failures can interrupt API calls
- Speech recognition errors need graceful handling

### Safety Layers
| Layer | Check | Response |
|-------|-------|----------|
| Speech | Confidence threshold | Request repeat if <80% |
| LLM Input | Command validation | Reject unsafe keywords |
| LLM Output | Schema validation | Retry or reject malformed |
| Action Plan | Capability check | Filter unsupported actions |
| Execution | Timeout and preemption | Cancel stuck actions |

### Error Handling Strategies
| Error | Strategy | Fallback |
|-------|----------|----------|
| Speech unclear | Request repeat | Visual feedback |
| LLM API timeout | Retry with backoff | Local LLM fallback |
| Invalid action plan | Re-prompt LLM | Manual confirmation |
| Action execution failure | Recovery behavior | Stop and report |

### References
- [ROS 2 Error Handling](https://design.ros2.org/articles/node_lifecycle.html)
- [LLM Safety Best Practices](https://platform.openai.com/docs/guides/safety-best-practices)

---

## 7. Hardware and Software Requirements

### Decision
Document minimum requirements compatible with Modules 1-3 setup.

### Hardware Requirements
| Component | Minimum | Recommended | Notes |
|-----------|---------|-------------|-------|
| Microphone | USB mic | Directional mic | For voice capture |
| GPU | None (Whisper tiny) | RTX 3060+ | For larger Whisper models |
| RAM | 8GB | 16GB+ | For Whisper + LLM client |
| Internet | Required (LLM API) | High bandwidth | For OpenAI API calls |

### Software Requirements
| Package | Version | Purpose |
|---------|---------|---------|
| Python | 3.10+ | LLM integration |
| ROS 2 | Humble | Robot middleware |
| openai-whisper | 20231117+ | Speech recognition |
| openai | 1.0+ | LLM API client |
| PyAudio | 0.2.14+ | Audio capture |
| pyaudio-webrtcvad | 2.0.10+ | Voice activity detection |

### References
- [Whisper Requirements](https://github.com/openai/whisper#setup)
- [OpenAI Python SDK](https://github.com/openai/openai-python)

---

## 8. Content Delivery Format

### Decision
Follow existing Docusaurus pattern from Modules 1-3.

### File Structure
```
frontend_book/docs/module-4-vla/
├── _category_.json
├── 01-voice-to-action.md
├── 02-llm-cognitive-planning.md
└── 03-capstone-voice-humanoid.md
```

### Word Count Distribution
| Chapter | Target Words | Content Focus |
|---------|--------------|---------------|
| Chapter 1 | 1000-1500 | Voice input, Whisper, ROS 2 publishing |
| Chapter 2 | 1200-1800 | LLM planning, prompts, action parsing |
| Chapter 3 | 1000-1500 | Integration, capstone demo |
| **Total** | **3200-4800** | Within 3000-5000 constraint |

---

## Unknowns Resolved

| Unknown | Resolution |
|---------|------------|
| Speech recognition | OpenAI Whisper (local, open-source) |
| LLM for planning | OpenAI GPT-4 via API |
| Audio capture | PyAudio with webrtcvad |
| Action plan format | JSON schema with primitives |
| ROS 2 integration | Action interfaces, Nav2 from Module 3 |
| Safety approach | Multi-layer validation |
| Hardware requirements | USB mic + internet (API) |
| Content format | Docusaurus Markdown, 3000-5000 words |

## Next Steps

1. Generate `data-model.md` with content structure
2. Create chapter contracts in `contracts/`
3. Generate `quickstart.md` for implementation
4. Create `plan.md` main planning document
5. Proceed to `/sp.tasks` for task generation
