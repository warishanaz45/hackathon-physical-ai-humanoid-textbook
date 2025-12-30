# Chapter Contracts: Module 4 - Vision-Language-Action (VLA)

**Feature**: 004-vla-module-4
**Date**: 2025-12-29
**Purpose**: Define contracts for each chapter's content and structure

---

## Contract 1: Chapter 1 - Voice-to-Action with Speech Recognition

### Input Requirements
- Reader has completed Modules 1-3 (ROS 2, simulation, NVIDIA Isaac)
- Reader has a USB microphone connected
- Reader has internet access (for initial setup)
- Reader has Python 3.10+ installed

### Output Guarantees
1. Reader can capture audio from a microphone
2. Reader can transcribe speech to text using Whisper
3. Reader can publish transcribed commands to a ROS 2 topic
4. Reader achieves >90% accuracy for clear English commands
5. Reader understands the VLA pipeline architecture

### Content Contract

| Section | Must Include | Validation |
|---------|--------------|------------|
| Introduction | VLA pipeline overview diagram | Clear architecture visual |
| Audio Capture | PyAudio setup with VAD | Audio captured successfully |
| Whisper Setup | Model selection guidance | Whisper running |
| Transcription | Real-time transcription demo | Text output from speech |
| ROS 2 Integration | Topic publishing | `/voice_command` topic active |

### Code Examples Required

```python
# Example 1: Audio capture with VAD
# File: audio_capture.py
# Expected output: "Listening... Speech detected... Audio captured (2.3 seconds)"
```

```python
# Example 2: Whisper ROS 2 node
# File: whisper_node.py
# Expected output: "Transcribed: 'go to the kitchen'"
```

```python
# Example 3: Complete voice command publisher
# File: voice_command_publisher.py
# Expected output: "Published to /voice_command: 'pick up the red block'"
```

### Success Criteria
- [ ] SC-001: Reader captures and transcribes voice within 5 minutes
- [ ] SC-002: Speech recognition achieves >90% accuracy for simple commands
- [ ] SC-007: All code examples run successfully

---

## Contract 2: Chapter 2 - LLM-Based Cognitive Planning

### Input Requirements
- Reader has completed Chapter 1
- Reader has OpenAI API key (or alternative LLM access)
- Reader understands ROS 2 actions concept
- Reader has internet access for API calls

### Output Guarantees
1. Reader can send commands to an LLM for planning
2. Reader can receive structured action plans in JSON
3. Reader can parse action plans into executable primitives
4. Reader can map primitives to ROS 2 action interfaces
5. Reader understands safety considerations for LLM-robotics

### Content Contract

| Section | Must Include | Validation |
|---------|--------------|------------|
| LLM as Planner | Conceptual explanation | Clear understanding |
| Prompt Engineering | System prompt example | Prompt file |
| Action Generation | JSON schema | Valid JSON output |
| Parsing | Python parser | Primitives extracted |
| ROS 2 Mapping | Action client code | Actions callable |
| Safety | Validation examples | Invalid commands rejected |

### Code Examples Required

```python
# Example 1: LLM planner node
# File: llm_planner_node.py
# Expected output: "Generated plan with 3 actions for: 'go to kitchen and wave'"
```

```python
# Example 2: Action parser
# File: action_parser.py
# Expected output: "Parsed actions: [navigate(kitchen), gesture(wave)]"
```

```text
# Example 3: System prompt
# File: system_prompt.txt
# Expected output: N/A (configuration file)
```

```python
# Example 4: Action executor
# File: action_executor.py
# Expected output: "Executing action 1/2: navigate to kitchen"
```

### Success Criteria
- [ ] SC-003: Reader generates action plans within 10 minutes of completing chapter
- [ ] SC-004: LLM-generated plans are parseable and map to valid ROS 2 actions
- [ ] SC-006: Reader can articulate the VLA architecture
- [ ] SC-008: Reader can explain 3 key challenges in LLM-robotics integration

---

## Contract 3: Chapter 3 - Capstone: Voice-Controlled Humanoid

### Input Requirements
- Reader has completed Chapters 1-2
- Reader has Nav2 configured from Module 3
- Reader has simulation environment running
- Reader has full VLA pipeline components ready

### Output Guarantees
1. Reader can integrate voice input, LLM planning, and robot execution
2. Reader can execute navigation commands via voice
3. Reader can execute gesture commands via voice
4. Reader can execute multi-step commands autonomously
5. Reader completes a working capstone demonstration

### Content Contract

| Section | Must Include | Validation |
|---------|--------------|------------|
| Integration | Launch file | All nodes running |
| Navigation | Voice-to-Nav2 demo | Robot navigates |
| Gestures | Gesture action server | Robot gestures |
| Multi-step | Complex command demo | Sequence completes |
| Testing | Validation approach | Tests passing |

### Code Examples Required

```python
# Example 1: VLA pipeline launch
# File: vla_pipeline.launch.py
# Expected output: "VLA pipeline running: whisper_node, llm_planner, action_executor"
```

```python
# Example 2: Gesture action server
# File: gesture_action_server.py
# Expected output: "Gesture server ready. Executing: wave"
```

```python
# Example 3: Capstone demo script
# File: capstone_demo.py
# Expected output: "Demo complete: Robot navigated to table and waved hello"
```

### Success Criteria
- [ ] SC-005: Reader executes capstone demo within 20 minutes of completing chapter
- [ ] SC-007: All code examples run successfully
- [ ] SC-006: Reader can articulate the VLA architecture and data flow

---

## Cross-Chapter Contracts

### Consistency Requirements
| Requirement | Validation |
|-------------|------------|
| ROS 2 Humble throughout | All launch files compatible |
| Python 3.10+ throughout | All scripts compatible |
| Whisper model consistent | Same model in examples |
| JSON schema consistent | Same action format |

### Writing Standards
| Standard | Validation |
|----------|------------|
| "What You'll Learn" present | Each chapter starts with list |
| "Key Takeaways" present | Each chapter ends with list |
| Technical terms defined | First use includes definition |
| Expected outputs shown | All code blocks have output |
| Prerequisites stated | Each chapter intro |
| Word count | 3000-5000 total |

### Link Requirements
| Link Type | Requirement |
|-----------|-------------|
| Internal | Use relative Docusaurus paths |
| External | Official OpenAI/ROS 2 docs only |
| Module refs | Link to Module 1-3 chapters |

### Source Citation Requirements
| Citation Type | Format |
|---------------|--------|
| OpenAI docs | Official API documentation |
| Whisper docs | GitHub README |
| ROS 2 docs | docs.ros.org |
| Nav2 docs | navigation.ros.org |

---

## Word Count Budget

| Chapter | Min Words | Max Words | Primary Focus |
|---------|-----------|-----------|---------------|
| Chapter 1 | 1000 | 1500 | Voice capture + Whisper |
| Chapter 2 | 1200 | 1800 | LLM planning + parsing |
| Chapter 3 | 1000 | 1500 | Integration + capstone |
| **Total** | **3200** | **4800** | Within 3000-5000 |

---

## Edge Case Coverage

### Chapter 1 Edge Cases
| Edge Case | Must Cover |
|-----------|------------|
| Background noise | VAD filtering approach |
| Unclear speech | Confidence threshold, retry |
| Microphone disconnection | Error handling |
| Long commands | Buffer management |

### Chapter 2 Edge Cases
| Edge Case | Must Cover |
|-----------|------------|
| Ambiguous commands | LLM clarification prompts |
| Unsupported actions | Capability checking |
| Invalid JSON | Retry with error feedback |
| API timeout | Fallback strategies |

### Chapter 3 Edge Cases
| Edge Case | Must Cover |
|-----------|------------|
| Navigation failure | Recovery behaviors |
| Gesture timeout | Action preemption |
| Pipeline interruption | Graceful shutdown |
| Multi-step failure | Partial completion handling |
