# Tasks: Module 4 - Vision-Language-Action (VLA)

**Input**: Design documents from `/specs/004-vla-module-4/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/chapter-structure.md

**Tests**: No tests required - this is a documentation project. Validation via Docusaurus build.

**Organization**: Tasks are grouped by user story (chapter) to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=Chapter 1, US2=Chapter 2, US3=Chapter 3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus content**: `frontend_book/docs/module-4-vla/`
- **Spec artifacts**: `specs/004-vla-module-4/`

---

## Phase 1: Setup (Module Infrastructure)

**Purpose**: Create module directory structure and metadata

- [x] T001 Create module directory at `frontend_book/docs/module-4-vla/`
- [x] T002 Create category metadata in `frontend_book/docs/module-4-vla/_category_.json`
- [x] T003 [P] Create chapter stub with frontmatter in `frontend_book/docs/module-4-vla/01-voice-to-action.md`
- [x] T004 [P] Create chapter stub with frontmatter in `frontend_book/docs/module-4-vla/02-llm-cognitive-planning.md`
- [x] T005 [P] Create chapter stub with frontmatter in `frontend_book/docs/module-4-vla/03-capstone-voice-humanoid.md`
- [x] T006 Verify Docusaurus build passes with `npm run build` in `frontend_book/`

**Checkpoint**: Module structure ready, all stubs render in Docusaurus

---

## Phase 2: User Story 1 - Voice-to-Action with Speech Recognition (Priority: P1) 🎯 MVP

**Goal**: Reader can implement voice-to-action functionality using speech recognition to capture spoken commands and convert them to text for ROS 2 publishing.

**Independent Test**: Reader can set up a microphone, run speech recognition, speak a command like "pick up the red block", and see the transcribed text output correctly and published to a ROS 2 topic.

### Implementation for User Story 1

- [x] T007 [US1] Write "What You'll Learn" section in `frontend_book/docs/module-4-vla/01-voice-to-action.md`
- [x] T008 [US1] Write "Introduction to Voice-Controlled Robotics" section covering VLA pipeline overview and why voice for humanoids
- [x] T009 [US1] Write "Speech Recognition Fundamentals" section covering how STT works and available approaches (cloud vs local)
- [x] T010 [US1] Write "Setting Up Audio Capture" section covering hardware requirements, PyAudio installation, and VAD
- [x] T011 [US1] Add code example `audio_capture.py` with expected output for microphone capture with VAD
- [x] T012 [US1] Write "Implementing Whisper Transcription" section covering model selection and real-time transcription
- [x] T013 [US1] Add code example `whisper_node.py` with expected output for ROS 2 node wrapping Whisper
- [x] T014 [US1] Write "Publishing to ROS 2" section covering topic creation and message publishing
- [x] T015 [US1] Add code example `voice_command_publisher.py` with expected output for complete voice capture + publish node
- [x] T016 [US1] Write "Key Takeaways" section summarizing Chapter 1 concepts
- [x] T017 [US1] Add prerequisites callout referencing Modules 1-3
- [x] T018 [US1] Add hardware requirements callout (USB microphone, internet for setup)

**Checkpoint**: Chapter 1 complete - reader can capture and transcribe voice commands (SC-001, SC-002, SC-007 validated)

---

## Phase 3: User Story 2 - LLM-Based Cognitive Planning (Priority: P2)

**Goal**: Reader can use LLMs for cognitive planning to convert natural language commands into structured action sequences that a robot can execute.

**Independent Test**: Reader can send a natural language command like "Go to the kitchen and fetch a glass of water" to an LLM and receive a structured action plan (JSON format) with executable primitives.

### Implementation for User Story 2

- [x] T019 [US2] Write "What You'll Learn" section in `frontend_book/docs/module-4-vla/02-llm-cognitive-planning.md`
- [x] T020 [US2] Write "LLMs as Robot Planners" section covering language understanding to action planning and capabilities/limitations
- [x] T021 [US2] Write "Prompt Engineering for Robotics" section covering system prompt design and output format specification
- [x] T022 [US2] Add code example `system_prompt.txt` showing complete robot planner system prompt
- [x] T023 [US2] Write "Generating Action Plans" section covering JSON schema and action primitives (navigate, gesture, pick, place)
- [x] T024 [US2] Add code example `llm_planner_node.py` with expected output for ROS 2 node calling LLM API
- [x] T025 [US2] Write "Parsing and Validating LLM Output" section covering JSON parsing and schema validation
- [x] T026 [US2] Add code example `action_parser.py` with expected output for parsing LLM JSON to action primitives
- [x] T027 [US2] Write "Mapping Actions to ROS 2" section covering action primitives to ROS 2 actions and Nav2 integration
- [x] T028 [US2] Add code example `action_executor.py` with expected output for executing parsed actions via ROS 2 action clients
- [x] T029 [US2] Write "Safety Considerations" section covering capability checking, invalid command detection, and action validation
- [x] T030 [US2] Write "Key Takeaways" section summarizing Chapter 2 concepts
- [x] T031 [US2] Add prerequisites callout referencing Chapter 1 and OpenAI API setup
- [x] T032 [US2] Add performance and error handling callout (API timeouts, fallbacks)

**Checkpoint**: Chapter 2 complete - reader can generate and parse action plans from natural language (SC-003, SC-004, SC-006, SC-008 validated)

---

## Phase 4: User Story 3 - Capstone: Voice-Controlled Humanoid (Priority: P3)

**Goal**: Reader can build a capstone project demonstrating an autonomous humanoid executing tasks via voice commands by integrating all learned concepts into a complete working system.

**Independent Test**: Reader can speak a multi-step command like "Go to the table and wave hello", and the simulated humanoid robot navigates to the target location and performs the gesture.

### Implementation for User Story 3

- [x] T033 [US3] Write "What You'll Learn" section in `frontend_book/docs/module-4-vla/03-capstone-voice-humanoid.md`
- [x] T034 [US3] Write "Capstone Overview" section covering project goals, end-to-end pipeline recap, and what reader will build
- [x] T035 [US3] Write "Integrating the VLA Pipeline" section covering voice → LLM → actions connection and launch file configuration
- [x] T036 [US3] Add code example `vla_pipeline.launch.py` with expected output for complete VLA launch file
- [x] T037 [US3] Write "Navigation via Voice Commands" section covering voice to Nav2 goals and integration with Module 3 setup
- [x] T038 [US3] Write "Gesture Execution" section covering simple gesture action server and joint controller commands
- [x] T039 [US3] Add code example `gesture_action_server.py` with expected output for gesture execution node
- [x] T040 [US3] Write "Multi-Step Command Demo" section with example "Go to the table and wave hello" showing LLM planning and execution
- [x] T041 [US3] Add code example `capstone_demo.py` with expected output for end-to-end demonstration script
- [x] T042 [US3] Write "Testing and Validation" section covering end-to-end testing approach and common issues
- [x] T043 [US3] Write "Key Takeaways" section summarizing Chapter 3 concepts
- [x] T044 [US3] Write "Where to Go from Here" section covering advanced topics, real hardware deployment, and further reading
- [x] T045 [US3] Add prerequisites callout referencing Chapters 1-2 and Module 3 Nav2

**Checkpoint**: Chapter 3 complete - reader can execute complete VLA demo (SC-005 validated)

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Final validation and quality assurance

- [x] T046 Verify word count is within 3000-5000 words across all chapters (FR-025)
- [x] T047 Verify all code examples have expected outputs or visualization descriptions (FR-020)
- [x] T048 Verify all chapters have "What You'll Learn" and "Key Takeaways" sections (FR-021)
- [x] T049 Verify all technical terms are defined on first use (FR-022)
- [x] T050 Verify content builds progressively from Modules 1-3 concepts (FR-023)
- [x] T051 Verify all claims are supported by official documentation references (FR-024)
- [x] T052 Run Docusaurus build verification with `npm run build` in `frontend_book/`
- [x] T053 Check all internal links resolve correctly
- [x] T054 Validate code examples against official OpenAI and ROS 2 documentation
- [x] T055 [P] Update specs checklist in `specs/004-vla-module-4/checklists/requirements.md`

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **User Story 1 (Phase 2)**: Depends on Setup (T006 must pass)
- **User Story 2 (Phase 3)**: Depends on Setup, independent of US1 content
- **User Story 3 (Phase 4)**: Depends on Setup, independent of US1/US2 content
- **Polish (Phase 5)**: Depends on all User Stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Setup - foundation for voice input
- **User Story 2 (P2)**: Can start after Setup - content references Chapter 1 concepts but file is independent
- **User Story 3 (P3)**: Can start after Setup - content references Chapters 1-2 but file is independent

### Within Each User Story

- Write sections in order (top to bottom of chapter)
- Add code examples after their explanatory sections
- "Key Takeaways" always last
- Hardware/prerequisites callouts after main content

### Parallel Opportunities

Within Setup:
- T003, T004, T005 can run in parallel (different chapter stubs)

Across User Stories (if multiple writers):
- Once Setup complete, US1/US2/US3 can proceed in parallel
- Each chapter is a separate file with no conflicts

---

## Parallel Example: Setup Phase

```bash
# Launch all chapter stub creations together:
Task: "Create chapter stub in frontend_book/docs/module-4-vla/01-voice-to-action.md"
Task: "Create chapter stub in frontend_book/docs/module-4-vla/02-llm-cognitive-planning.md"
Task: "Create chapter stub in frontend_book/docs/module-4-vla/03-capstone-voice-humanoid.md"
```

## Parallel Example: Multiple Writers

```bash
# Writer A: User Story 1 (Chapter 1)
# Writer B: User Story 2 (Chapter 2)
# Writer C: User Story 3 (Chapter 3)
# All can proceed after Setup is complete
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T006)
2. Complete Phase 2: User Story 1 (T007-T018)
3. **STOP and VALIDATE**: Run `npm run build`, verify Chapter 1 renders
4. Deploy/demo if Chapter 1 content is sufficient for initial review

### Incremental Delivery

1. Complete Setup → Module structure ready
2. Add User Story 1 → Chapter 1 complete → Deploy/Demo (MVP!)
3. Add User Story 2 → Chapter 2 complete → Deploy/Demo
4. Add User Story 3 → Chapter 3 complete → Deploy/Demo
5. Polish phase → Final validation

### Success Criteria Mapping

| Task Range | Success Criteria Validated |
|------------|---------------------------|
| T007-T018 | SC-001, SC-002, SC-007 |
| T019-T032 | SC-003, SC-004, SC-006, SC-008 |
| T033-T045 | SC-005 |
| T046-T055 | All FR requirements verified |

---

## Summary

| Phase | Tasks | Parallel | Description |
|-------|-------|----------|-------------|
| Setup | 6 | 3 | Module infrastructure |
| US1 (P1) | 12 | 0 | Voice-to-Action |
| US2 (P2) | 14 | 0 | LLM Cognitive Planning |
| US3 (P3) | 13 | 0 | Capstone Voice Humanoid |
| Polish | 10 | 1 | Validation & QA |
| **Total** | **55** | **4** | |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story (chapter)
- Each chapter is independently completable
- Verify Docusaurus build after each phase
- Commit after each task or logical group
- Reference official OpenAI Whisper and ROS 2 documentation for all code examples
- Word count constraint: 3000-5000 words total across all 3 chapters
