# AI/Spec-Driven Book with RAG Chatbot Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-29

## Active Technologies

| Component | Technology | Version |
|-----------|------------|---------|
| Book Framework | Docusaurus | 3.x (latest) |
| Book Hosting | GitHub Pages | N/A |
| Runtime | Node.js | 18+ |
| Package Manager | npm | 9+ |
| Content Format | Markdown (.md) | N/A |

## Project Structure

```text
Hackathon-book-creation-2025/
├── frontend_book/                 # Docusaurus project
│   ├── docs/                      # Documentation content
│   │   ├── module-1-ros2/         # Module 1: ROS 2
│   │   │   ├── _category_.json
│   │   │   ├── 01-introduction.md
│   │   │   ├── 02-communication.md
│   │   │   └── 03-urdf.md
│   │   ├── module-2-simulation/   # Module 2: Digital Twin
│   │   │   ├── _category_.json
│   │   │   ├── 01-gazebo-physics.md
│   │   │   ├── 02-unity-digital-twins.md
│   │   │   └── 03-sensor-simulation.md
│   │   ├── module-3-nvidia-isaac/ # Module 3: NVIDIA Isaac
│   │   │   ├── _category_.json
│   │   │   ├── 01-isaac-sim-synthetic-data.md
│   │   │   ├── 02-isaac-ros-perception.md
│   │   │   └── 03-nav2-humanoid-planning.md
│   │   └── module-4-vla/          # Module 4: Vision-Language-Action
│   │       ├── _category_.json
│   │       ├── 01-voice-to-action.md
│   │       ├── 02-llm-cognitive-planning.md
│   │       └── 03-capstone-voice-humanoid.md
│   ├── src/                       # Custom React components
│   ├── static/                    # Static assets
│   ├── docusaurus.config.js       # Site configuration
│   ├── sidebars.js                # Sidebar configuration
│   └── package.json               # Node dependencies
├── specs/                         # Feature specifications
│   ├── 001-ros2-module-1/         # Module 1 spec
│   ├── 002-gazebo-unity-module-2/ # Module 2 spec
│   ├── 003-nvidia-isaac-module-3/ # Module 3 spec
│   │   ├── spec.md
│   │   ├── plan.md
│   │   ├── research.md
│   │   ├── data-model.md
│   │   ├── quickstart.md
│   │   └── contracts/
│   └── 004-vla-module-4/          # Module 4 spec
│       ├── spec.md
│       ├── plan.md
│       ├── research.md
│       ├── data-model.md
│       ├── quickstart.md
│       └── contracts/
├── history/                       # Prompt history records
│   └── prompts/
├── .specify/                      # SpecKit Plus configuration
│   ├── memory/                    # Constitution and agent context
│   ├── templates/                 # Document templates
│   └── scripts/                   # Automation scripts
└── .github/                       # GitHub configuration
    └── workflows/                 # CI/CD workflows
```

## Commands

### Docusaurus Development

```bash
# Navigate to book directory
cd book

# Install dependencies
npm install

# Start development server
npm start

# Build for production
npm run build

# Preview production build
npm run serve

# Clear cache
npm run clear

# Deploy to GitHub Pages
GIT_USER=<username> npm run deploy
```

### SpecKit Plus Workflow

```bash
# Create specification
/sp.specify <feature description>

# Create implementation plan
/sp.plan

# Generate tasks
/sp.tasks

# Implement tasks
/sp.implement
```

## Code Style

### Markdown (Book Content)

- Use ATX-style headers (`#`, `##`, `###`)
- Code blocks MUST have language identifier and title
- Tables MUST use pipe syntax with header separator
- Lists use `-` for unordered, `1.` for ordered
- One blank line between sections

### JavaScript (Docusaurus Config)

- Use `const` for immutable values
- Use template literals for string interpolation
- Single quotes for strings
- Trailing commas in multi-line objects/arrays

## Recent Changes

| Feature | Branch | What Was Added |
|---------|--------|----------------|
| Module 1: ROS 2 | 001-ros2-module-1 | Docusaurus setup, 3 chapters |
| Module 2: Digital Twin | 002-gazebo-unity-module-2 | Gazebo, Unity, sensor simulation chapters |
| Module 3: NVIDIA Isaac | 003-nvidia-isaac-module-3 | Isaac Sim, Isaac ROS, Nav2 chapters |
| Module 4: VLA | 004-vla-module-4 | Voice-to-action, LLM planning, capstone chapters |

## Module 3 Technologies

| Component | Technology | Version |
|-----------|------------|---------|
| Simulation | Isaac Sim | 2023.1.1+ |
| Perception | Isaac ROS | 3.0 |
| Navigation | Nav2 | ROS 2 Humble |
| Containers | Docker | Required for Isaac ROS |
| Hardware | NVIDIA GPU | RTX 3060+ or Jetson Orin |

## Module 4 Technologies

| Component | Technology | Version |
|-----------|------------|---------|
| Speech Recognition | OpenAI Whisper | 20231117+ |
| LLM | OpenAI GPT-4 | API v1.0+ |
| Audio Capture | PyAudio | 0.2.14+ |
| Voice Activity Detection | webrtcvad | 2.0.10+ |
| LLM Client | openai Python | 1.0+ |
| Robot Framework | ROS 2 Humble | Actions, Topics |
| Navigation | Nav2 | From Module 3 |

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->
