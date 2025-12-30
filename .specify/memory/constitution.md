<!--
=============================================================================
SYNC IMPACT REPORT
=============================================================================
Version change: 0.0.0 → 1.0.0 (MAJOR - initial ratification)

Modified principles: N/A (initial creation)

Added sections:
  - Core Principles (4 principles)
  - Technology Stack & Standards
  - Development Workflow
  - Governance

Removed sections: N/A (initial creation)

Templates requiring updates:
  ✅ plan-template.md - Constitution Check section references generic gates
  ✅ spec-template.md - No constitution-specific updates needed
  ✅ tasks-template.md - No constitution-specific updates needed

Follow-up TODOs: None
=============================================================================
-->

# AI/Spec-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Spec-First Workflow

All features MUST begin with a formal specification before any implementation.
The Spec-Kit Plus workflow is mandatory for every deliverable:

- `/sp.specify` produces the feature specification
- `/sp.plan` produces the implementation plan
- `/sp.tasks` produces actionable, dependency-ordered tasks
- No code is written until spec and plan are approved

**Rationale**: Specifications prevent scope creep, ensure stakeholder alignment, and create traceable artifacts for every decision.

### II. Technical Accuracy

All technical content MUST be verified against official documentation and authoritative sources.

- Book content MUST cite official docs (ROS 2, OpenAI, FastAPI, Qdrant, Neon, Docusaurus)
- Code examples MUST be tested and runnable
- Claims about API behavior MUST be verified—never assumed from memory
- Deprecated methods or breaking changes MUST be flagged

**Rationale**: Developer trust depends on accuracy. Incorrect tutorials cause hours of debugging and erode credibility.

### III. Clear Developer-Focused Writing

Content MUST prioritize clarity, brevity, and practical utility for developers.

- Use active voice and imperative instructions ("Run this command" not "The command can be run")
- Explain the "why" before the "how"
- Keep paragraphs short; prefer bullet lists and code blocks
- Include expected outputs for all commands
- Avoid jargon without definition; define acronyms on first use

**Rationale**: Developers skim; dense prose is skipped. Clear writing respects the reader's time and reduces support burden.

### IV. Reproducible Setup & Deployment

Every setup, configuration, and deployment MUST be reproducible from documentation alone.

- Environment requirements MUST be explicit (OS, versions, dependencies)
- All commands MUST be copy-paste ready
- Configuration MUST use environment variables (`.env` patterns) with documented defaults
- Docker/container setups MUST be provided where applicable
- CI/CD pipelines MUST be documented and runnable

**Rationale**: "Works on my machine" is unacceptable. Reproducibility is the foundation of developer experience.

## Technology Stack & Standards

### Platform Requirements

| Component | Technology | Constraint |
|-----------|------------|------------|
| Book Framework | Docusaurus | Latest stable version |
| Book Hosting | GitHub Pages | Public repository |
| RAG Backend | FastAPI | Python 3.11+ |
| Database | Neon Postgres | Serverless tier acceptable |
| Vector Store | Qdrant Cloud | Managed cloud instance |
| AI/Agent Layer | OpenAI Agents SDK / ChatKit | Latest stable SDK |

### RAG Chatbot Constraints

- The chatbot MUST be grounded exclusively in book content or user-selected text
- No hallucinated responses; if context is insufficient, chatbot MUST acknowledge uncertainty
- Vector embeddings MUST be regenerated when book content changes
- Source citations MUST be provided with every response

### Code Quality Standards

- All code MUST include type hints (Python) or TypeScript types
- All public functions MUST have docstrings
- Linting (ruff/eslint) and formatting (black/prettier) MUST pass before commit
- Test coverage MUST be meaningful (critical paths, edge cases)

## Development Workflow

### Branch Strategy

- Feature branches follow pattern: `###-feature-name` (e.g., `001-ros2-module-1`)
- Main branch is protected; requires passing CI before merge
- PRs MUST reference the spec they implement

### Documentation-Code Parity

- Book chapters and code examples MUST stay in sync
- Breaking changes to code MUST update corresponding documentation immediately
- Quickstart guides MUST be validated with fresh environment after each change

### Review Process

- All specs require stakeholder review before implementation
- All PRs require code review
- Documentation changes require accuracy review

## Governance

This constitution is the authoritative source for project standards. It supersedes conflicting guidance in other documents.

### Amendment Process

1. Propose amendment with rationale
2. Document impact on existing artifacts
3. Obtain stakeholder approval
4. Update constitution with version increment
5. Propagate changes to affected templates

### Versioning Policy

- **MAJOR**: Backward-incompatible principle changes or removals
- **MINOR**: New principles or significant expansions
- **PATCH**: Clarifications, typo fixes, non-semantic refinements

### Compliance

- All PRs and reviews MUST verify compliance with this constitution
- Violations MUST be documented and resolved before merge
- Exceptions require explicit justification in the PR description

**Version**: 1.0.0 | **Ratified**: 2025-12-29 | **Last Amended**: 2025-12-29
