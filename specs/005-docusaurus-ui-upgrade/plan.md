# Implementation Plan: Docusaurus UI Upgrade

**Branch**: `005-docusaurus-ui-upgrade` | **Date**: 2025-12-30 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-docusaurus-ui-upgrade/spec.md`

## Summary

Upgrade the visual design of the frontend_book Docusaurus site to provide a modern, clean, and user-friendly UI/UX for developers and readers. The implementation uses CSS-only customization through Infima CSS variables, avoiding component swizzling to ensure maintainability and clean upgrades.

## Technical Context

**Language/Version**: CSS3, JavaScript (Docusaurus 3.x, Node.js 18+)
**Primary Dependencies**: Docusaurus 3.x, Infima CSS Framework (built-in)
**Storage**: N/A (static site)
**Testing**: Visual inspection, browser DevTools, responsive testing
**Target Platform**: Web (all modern browsers)
**Project Type**: Web/Frontend (Docusaurus static site)
**Performance Goals**: Site loads in <3s, smooth theme transitions
**Constraints**: No swizzling, CSS-only changes, maintain Docusaurus upgrade path
**Scale/Scope**: Single documentation site with 4 modules, homepage, and navigation

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Spec-First Workflow | ✅ PASS | Spec created before implementation |
| II. Technical Accuracy | ✅ PASS | CSS patterns verified against Docusaurus docs |
| III. Clear Developer-Focused Writing | ✅ PASS | Quickstart provides copy-paste commands |
| IV. Reproducible Setup | ✅ PASS | All steps documented with expected outputs |

**Code Quality Standards**:
- CSS linting will be manual (no separate linter configured)
- Changes validated visually in both themes
- Responsive testing required at standard breakpoints

## Project Structure

### Documentation (this feature)

```text
specs/005-docusaurus-ui-upgrade/
├── spec.md                        # Feature specification
├── plan.md                        # This file
├── research.md                    # Phase 0 output
├── data-model.md                  # Design tokens definition
├── quickstart.md                  # Developer setup guide
├── contracts/
│   └── css-architecture.md        # CSS patterns and rules
├── checklists/
│   └── requirements.md            # Quality checklist
└── tasks.md                       # Phase 2 output (from /sp.tasks)
```

### Source Code (repository root)

```text
frontend_book/
├── src/
│   ├── css/
│   │   └── custom.css             # PRIMARY: All CSS variable overrides
│   ├── pages/
│   │   ├── index.js               # Homepage component
│   │   └── index.module.css       # Homepage-specific styles
│   └── components/
│       └── HomepageFeatures/
│           ├── index.js           # Feature cards component
│           └── styles.module.css  # Feature card styles
├── static/
│   └── img/                       # Static images (may add new assets)
├── docusaurus.config.js           # Site configuration
└── package.json                   # Dependencies
```

**Structure Decision**: Frontend-only project. All changes occur in `frontend_book/src/` directory. Primary file is `custom.css` for CSS variable overrides. Component CSS modules (`*.module.css`) for component-specific styling.

## Complexity Tracking

No violations requiring justification. Implementation follows minimal complexity approach:
- CSS-only changes (no component swizzling)
- Single CSS file for variable overrides
- Standard Docusaurus patterns

## Implementation Approach

### Phase 1: Color Palette (P1)

Define new color scheme meeting accessibility requirements:
1. Select primary color with WCAG AA contrast
2. Generate 7-shade scale using ColorBox or similar
3. Define dark mode complementary palette
4. Override Infima color variables in `custom.css`

### Phase 2: Typography & Spacing (P1)

Improve readability with refined typography:
1. Adjust line-heights for body text (1.65-1.7)
2. Define consistent heading scale
3. Set up spacing scale using CSS custom properties
4. Ensure code font sizing is comfortable

### Phase 3: Component Styling (P1-P2)

Enhance individual component appearance:
1. Navbar: Improved hover states, visual refinement
2. Sidebar: Active state indication, better hierarchy
3. Footer: Consistent with overall design language
4. Hero: Modernized homepage header section
5. Feature cards: Better visual balance and spacing
6. Code blocks: Theme-consistent styling
7. Buttons: Clear hover/focus states

### Phase 4: Responsive Design (P2)

Ensure mobile-first responsive behavior:
1. Verify navbar collapse at 996px
2. Touch-friendly targets (44x44px minimum)
3. Appropriate spacing at mobile breakpoints
4. Code block horizontal scrolling

### Phase 5: Final Polish (P2-P3)

1. Theme transition smoothness
2. Visual consistency audit
3. Edge case handling (long titles, deep nesting)
4. Cross-browser testing

## Key Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Styling Approach | CSS Variables | Maintainable, upgrade-safe |
| Component Modification | No swizzling | Reduces maintenance burden |
| Color Generation | ColorBox tool | Ensures mathematical shade consistency |
| Typography | System fonts | No additional dependencies |
| Breakpoints | Docusaurus defaults | Consistency with framework |

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| CSS variables not applying | Medium | Test incrementally, use DevTools |
| Contrast issues | High | Validate with WCAG tools before committing |
| Breaking on Docusaurus update | Low | Avoid targeting internal classes |
| Mobile layout issues | Medium | Test on real devices, not just DevTools |

## Dependencies

- **Docusaurus 3.x**: Already installed
- **Infima CSS**: Built-in with Docusaurus
- **No new dependencies required**

## Acceptance Criteria

From spec success criteria:
- [ ] SC-001: 4.5:1 contrast ratio for normal text, 3:1 for large text
- [ ] SC-002: Body text minimum 16px, line height 1.5-1.7
- [ ] SC-003: Functional on viewports 320px to 2560px
- [ ] SC-004: Touch targets minimum 44x44px
- [ ] SC-005: Consistent spacing, no orphaned elements
- [ ] SC-006: Light and dark themes visually consistent
- [ ] SC-007: No broken layouts or visual bugs
- [ ] SC-008: All changes use Docusaurus-supported methods

## Next Steps

Run `/sp.tasks` to generate detailed implementation tasks with test cases.
