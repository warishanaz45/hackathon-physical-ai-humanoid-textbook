# Tasks: Module 2 - The Digital Twin (Gazebo & Unity)

**Input**: Design documents from `/specs/002-gazebo-unity-module-2/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are NOT explicitly requested in the feature specification. This is a documentation/content project.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story. Each user story corresponds to a chapter in the Docusaurus book.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus project**: `frontend_book/` at repository root
- **Documentation content**: `frontend_book/docs/`
- **Module 2 content**: `frontend_book/docs/module-2-simulation/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create Module 2 directory structure and category metadata

- [x] T001 Create module directory at frontend_book/docs/module-2-simulation/
- [x] T002 Create module category metadata in frontend_book/docs/module-2-simulation/_category_.json
- [x] T003 [P] Create chapter file stub with frontmatter in frontend_book/docs/module-2-simulation/01-gazebo-physics.md
- [x] T004 [P] Create chapter file stub with frontmatter in frontend_book/docs/module-2-simulation/02-unity-digital-twins.md
- [x] T005 [P] Create chapter file stub with frontmatter in frontend_book/docs/module-2-simulation/03-sensor-simulation.md

**Checkpoint**: Module 2 appears in sidebar with 3 empty chapters

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Verify Module 1 exists and Docusaurus is configured correctly

**⚠️ CRITICAL**: No chapter content can be written until this phase is complete

- [x] T006 Verify Module 1 exists at frontend_book/docs/module-1-ros2/ with 3 chapters
- [x] T007 Verify Docusaurus build passes with `npm run build` in frontend_book/
- [x] T008 Verify sidebar autogeneration works with new module structure

**Checkpoint**: Foundation ready - Module 2 structure validated, can proceed with content

---

## Phase 3: User Story 1 - Learn Physics Simulation with Gazebo (Priority: P1) 🎯 MVP

**Goal**: Reader understands Gazebo Fortress architecture, can launch a humanoid robot in simulation, and configure physics parameters

**Independent Test**: Reader can launch Gazebo with a robot, modify physics parameters, and observe behavior changes within 10 minutes

### Implementation for User Story 1

- [x] T009 [US1] Write "What You'll Learn" section in frontend_book/docs/module-2-simulation/01-gazebo-physics.md
- [x] T010 [US1] Write "Gazebo Architecture Overview" section with component diagram and comparison in frontend_book/docs/module-2-simulation/01-gazebo-physics.md
- [x] T011 [US1] Write "SDF and URDF for Simulation" section with humanoid_gazebo.urdf example in frontend_book/docs/module-2-simulation/01-gazebo-physics.md
- [x] T012 [US1] Write "Creating a Simulation World" section with humanoid_world.sdf example in frontend_book/docs/module-2-simulation/01-gazebo-physics.md
- [x] T013 [US1] Write "Physics Configuration" section with timestep, solver, friction table in frontend_book/docs/module-2-simulation/01-gazebo-physics.md
- [x] T014 [US1] Write "Joint Control via ROS 2" section with gazebo_launch.py example in frontend_book/docs/module-2-simulation/01-gazebo-physics.md
- [x] T015 [US1] Write joint_controller.py example with expected output in frontend_book/docs/module-2-simulation/01-gazebo-physics.md
- [x] T016 [US1] Write "Key Takeaways" section in frontend_book/docs/module-2-simulation/01-gazebo-physics.md
- [x] T017 [US1] Add navigation link to Chapter 2 in frontend_book/docs/module-2-simulation/01-gazebo-physics.md

**Checkpoint**: Chapter 1 complete and readable; reader can articulate Gazebo basics and launch simulation

---

## Phase 4: User Story 2 - Create Digital Twins & HRI in Unity (Priority: P2)

**Goal**: Reader can import robot into Unity, connect to ROS 2, and create basic HRI scenarios

**Independent Test**: Reader can import URDF into Unity and establish ROS 2 communication within 15 minutes

### Implementation for User Story 2

- [x] T018 [US2] Write "What You'll Learn" section in frontend_book/docs/module-2-simulation/02-unity-digital-twins.md
- [x] T019 [US2] Write "Why Unity for Digital Twins?" section with Gazebo vs Unity comparison table in frontend_book/docs/module-2-simulation/02-unity-digital-twins.md
- [x] T020 [US2] Write "Setting Up Unity with ROS 2" section with ROS-TCP-Connector setup in frontend_book/docs/module-2-simulation/02-unity-digital-twins.md
- [x] T021 [US2] Write "Importing Robot Models" section with URDF Importer workflow in frontend_book/docs/module-2-simulation/02-unity-digital-twins.md
- [x] T022 [US2] Write "Creating Realistic Environments" section with lighting and materials in frontend_book/docs/module-2-simulation/02-unity-digital-twins.md
- [x] T023 [US2] Write "Human-Robot Interaction Setup" section with avatar integration in frontend_book/docs/module-2-simulation/02-unity-digital-twins.md
- [x] T024 [US2] Write "Bidirectional ROS Communication" section with JointStateSubscriber.cs example in frontend_book/docs/module-2-simulation/02-unity-digital-twins.md
- [x] T025 [US2] Write JointCommandPublisher.cs example with expected output in frontend_book/docs/module-2-simulation/02-unity-digital-twins.md
- [x] T026 [US2] Write HRIInteraction.cs example in frontend_book/docs/module-2-simulation/02-unity-digital-twins.md
- [x] T027 [US2] Write "Key Takeaways" section in frontend_book/docs/module-2-simulation/02-unity-digital-twins.md
- [x] T028 [US2] Add navigation link to Chapter 3 in frontend_book/docs/module-2-simulation/02-unity-digital-twins.md

**Checkpoint**: Chapter 2 complete; all C# examples have expected outputs where applicable

---

## Phase 5: User Story 3 - Simulate and Validate Sensors (Priority: P3)

**Goal**: Reader can configure LiDAR, depth cameras, and IMU sensors with realistic noise models

**Independent Test**: Reader can configure and visualize all three sensor types in RViz after completing chapter

### Implementation for User Story 3

- [x] T029 [US3] Write "What You'll Learn" section in frontend_book/docs/module-2-simulation/03-sensor-simulation.md
- [x] T030 [US3] Write "Sensor Simulation Overview" section with sensor types table in frontend_book/docs/module-2-simulation/03-sensor-simulation.md
- [x] T031 [US3] Write "LiDAR Simulation" section with lidar_sensor.sdf example in frontend_book/docs/module-2-simulation/03-sensor-simulation.md
- [x] T032 [US3] Write "Depth Camera Simulation" section with depth_camera.sdf example in frontend_book/docs/module-2-simulation/03-sensor-simulation.md
- [x] T033 [US3] Write "IMU Simulation" section with imu_sensor.sdf example in frontend_book/docs/module-2-simulation/03-sensor-simulation.md
- [x] T034 [US3] Write "Visualizing Sensor Data" section with sensors.rviz configuration in frontend_book/docs/module-2-simulation/03-sensor-simulation.md
- [x] T035 [US3] Write sensor_bridge.launch.py example with expected output in frontend_book/docs/module-2-simulation/03-sensor-simulation.md
- [x] T036 [US3] Write "Sensor Validation Approaches" section with validation metrics table in frontend_book/docs/module-2-simulation/03-sensor-simulation.md
- [x] T037 [US3] Write "Key Takeaways" section in frontend_book/docs/module-2-simulation/03-sensor-simulation.md
- [x] T038 [US3] Add "Module Summary" section with Module 3 preview in frontend_book/docs/module-2-simulation/03-sensor-simulation.md

**Checkpoint**: Chapter 3 complete; all sensor configurations are syntactically valid SDF

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Validation and quality assurance

- [x] T039 Run Docusaurus build to verify no errors with `npm run build` in frontend_book/
- [x] T040 Verify all internal links work (chapter navigation, Module 1 references)
- [x] T041 Verify all code blocks have title attributes and language identifiers
- [x] T042 Verify all technical terms are defined on first use
- [x] T043 [P] Preview site locally and verify sidebar navigation with `npm run serve` in frontend_book/
- [x] T044 [P] Validate chapter frontmatter against data-model.md schema
- [x] T045 Update footer links in frontend_book/docusaurus.config.js to include Module 2

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Phase 1 (Module structure must exist)
- **User Stories (Phase 3-5)**: All depend on Phase 2 (chapter files must exist)
  - US1, US2, US3 can proceed in parallel OR sequentially
  - For this documentation project, sequential P1 → P2 → P3 is recommended
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Phase 2 - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Phase 2 - Builds on US1 concepts but independently testable
- **User Story 3 (P3)**: Can start after Phase 2 - Builds on US1/US2 concepts but independently testable

### Within Each User Story

- "What You'll Learn" → Main content sections → "Key Takeaways" → Navigation links
- Each chapter is independently completable
- Commit after each section or logical group

### Parallel Opportunities

- T003, T004, T005 (chapter stubs) can run in parallel
- T043 and T044 (preview + validation) can run in parallel
- Within each user story, content sections are sequential (build on each other)

---

## Parallel Example: Phase 1 Setup

```bash
# These can run in parallel:
Task: "Create chapter file stub with frontmatter in frontend_book/docs/module-2-simulation/01-gazebo-physics.md"
Task: "Create chapter file stub with frontmatter in frontend_book/docs/module-2-simulation/02-unity-digital-twins.md"
Task: "Create chapter file stub with frontmatter in frontend_book/docs/module-2-simulation/03-sensor-simulation.md"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (Module structure created)
2. Complete Phase 2: Foundational (Verified existing setup)
3. Complete Phase 3: User Story 1 (Chapter 1 written)
4. **STOP and VALIDATE**: Verify Chapter 1 is readable and accurate
5. Build and preview locally

### Incremental Delivery

1. Setup + Foundational → Module 2 structure ready
2. Add User Story 1 (Chapter 1: Gazebo) → Test independently → MVP!
3. Add User Story 2 (Chapter 2: Unity) → Test independently
4. Add User Story 3 (Chapter 3: Sensors) → Test independently
5. Each chapter adds value without breaking previous chapters

### Recommended Execution Order

1. T001 → T002 → (T003 ∥ T004 ∥ T005)
2. T006 → T007 → T008
3. T009 → T010 → T011 → T012 → T013 → T014 → T015 → T016 → T017
4. T018 → T019 → T020 → T021 → T022 → T023 → T024 → T025 → T026 → T027 → T028
5. T029 → T030 → T031 → T032 → T033 → T034 → T035 → T036 → T037 → T038
6. T039 → T040 → T041 → T042 → (T043 ∥ T044) → T045

---

## Summary

| Phase | Tasks | Parallel Tasks | Description |
|-------|-------|----------------|-------------|
| 1: Setup | 5 | 3 | Module structure creation |
| 2: Foundational | 3 | 0 | Verification |
| 3: US1 (P1) | 9 | 0 | Chapter 1: Gazebo Physics |
| 4: US2 (P2) | 11 | 0 | Chapter 2: Unity Digital Twins |
| 5: US3 (P3) | 10 | 0 | Chapter 3: Sensor Simulation |
| 6: Polish | 7 | 2 | Validation |
| **Total** | **45** | **5** | |

**MVP Scope**: Phases 1-3 (17 tasks) → Chapter 1 complete and deployable

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story/chapter
- Each chapter is independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate chapter independently
- All code examples must be verified against official documentation:
  - Gazebo Fortress: https://gazebosim.org/docs/fortress
  - Unity Robotics Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub
  - ROS 2 Humble: https://docs.ros.org/en/humble/
