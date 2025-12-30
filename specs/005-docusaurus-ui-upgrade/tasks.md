# Tasks: Docusaurus UI Upgrade

**Input**: Design documents from `/specs/005-docusaurus-ui-upgrade/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/css-architecture.md
**Branch**: `005-docusaurus-ui-upgrade`

**Tests**: Visual inspection and manual testing only (no automated tests requested)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Primary file**: `frontend_book/src/css/custom.css`
- **Homepage CSS**: `frontend_book/src/pages/index.module.css`
- **Feature cards CSS**: `frontend_book/src/components/HomepageFeatures/styles.module.css`

---

## Phase 1: Setup

**Purpose**: Prepare development environment and organize CSS file structure

- [x] T001 Start Docusaurus dev server with `npm start` in `frontend_book/`
- [x] T002 Backup current `frontend_book/src/css/custom.css` before making changes
- [x] T003 Add CSS section headers to `frontend_book/src/css/custom.css` per contracts/css-architecture.md structure

---

## Phase 2: Foundational (Color Palette & Typography)

**Purpose**: Establish the core design tokens that ALL user stories depend on

**CRITICAL**: No component styling can begin until colors and typography are defined

- [x] T004 Define light mode primary color 7-shade scale in `frontend_book/src/css/custom.css` (generate accessible palette using ColorBox)
- [x] T005 Define dark mode primary color 7-shade scale in `frontend_book/src/css/custom.css` under `[data-theme='dark']`
- [x] T006 [P] Add typography variables (line-height, heading weights) to `frontend_book/src/css/custom.css`
- [x] T007 [P] Add custom spacing scale variables (`--custom-space-*`) to `frontend_book/src/css/custom.css`
- [x] T008 [P] Add custom shadow variables (`--custom-shadow-*`) to `frontend_book/src/css/custom.css`
- [x] T009 [P] Add custom transition variables (`--custom-transition-*`) to `frontend_book/src/css/custom.css`
- [x] T010 Verify color contrast meets WCAG AA (4.5:1) using browser DevTools in both themes

**Checkpoint**: Foundation ready - colors, typography, and spacing tokens are defined

---

## Phase 3: User Story 1 - Reader Views Documentation (Priority: P1)

**Goal**: Readers can comfortably read documentation content in light and dark modes with improved typography and visual hierarchy

**Independent Test**: Navigate to any documentation page, switch between light/dark themes, verify text is legible with good contrast and comfortable spacing

### Implementation for User Story 1

- [x] T011 [US1] Override `--ifm-line-height-base` to 1.65 in `frontend_book/src/css/custom.css`
- [x] T012 [US1] Override heading font sizes (`--ifm-h1-font-size` through `--ifm-h6-font-size`) in `frontend_book/src/css/custom.css`
- [x] T013 [US1] Override paragraph and content spacing variables in `frontend_book/src/css/custom.css`
- [x] T014 [US1] Improve `.hero` section typography in `frontend_book/src/css/custom.css`
- [x] T015 [US1] Style `.hero__title` and `.hero__subtitle` for visual hierarchy in `frontend_book/src/css/custom.css`
- [x] T016 [P] [US1] Update `.heroBanner` styles in `frontend_book/src/pages/index.module.css`
- [x] T017 [US1] Add theme transition smoothness (200ms ease-in-out) in `frontend_book/src/css/custom.css`
- [x] T018 [US1] Verify documentation pages render correctly in both themes

**Checkpoint**: User Story 1 complete - readers can view documentation with improved typography

---

## Phase 4: User Story 2 - Reader Navigates Documentation (Priority: P1)

**Goal**: Readers can easily navigate using navbar, sidebar, and footer with clear visual hierarchy and hover states

**Independent Test**: Navigate through navbar items, expand/collapse sidebar sections, verify current section indication and hover states work properly

### Implementation for User Story 2

- [x] T019 [US2] Style `.navbar` background and shadow in `frontend_book/src/css/custom.css`
- [x] T020 [US2] Add `.navbar__link` hover and active states in `frontend_book/src/css/custom.css`
- [x] T021 [US2] Style `.navbar__title` font weight and color in `frontend_book/src/css/custom.css`
- [x] T022 [US2] Style `.menu` container (sidebar) in `frontend_book/src/css/custom.css`
- [x] T023 [US2] Style `.menu__link` hover states in `frontend_book/src/css/custom.css`
- [x] T024 [US2] Style `.menu__link--active` for current section indication in `frontend_book/src/css/custom.css`
- [x] T025 [US2] Style `.footer` background and padding in `frontend_book/src/css/custom.css`
- [x] T026 [US2] Style `.footer__title`, `.footer__link`, `.footer__copyright` in `frontend_book/src/css/custom.css`
- [x] T027 [US2] Add focus states for keyboard navigation accessibility in `frontend_book/src/css/custom.css`
- [x] T028 [US2] Verify navigation works correctly in both themes

**Checkpoint**: User Story 2 complete - navigation components have improved visual design

---

## Phase 5: User Story 3 - Reader Accesses Site on Mobile (Priority: P2)

**Goal**: Readers can comfortably use the site on mobile devices with touch-friendly interactions

**Independent Test**: Open site in responsive mode at 375px width, verify content scales without horizontal scroll, test mobile menu touch interactions

### Implementation for User Story 3

- [x] T029 [US3] Add responsive styles at `@media (max-width: 996px)` in `frontend_book/src/css/custom.css`
- [x] T030 [US3] Ensure touch targets (`.navbar__link`, `.menu__link`) are minimum 44x44px in `frontend_book/src/css/custom.css`
- [x] T031 [US3] Add responsive styles at `@media (max-width: 768px)` for mobile in `frontend_book/src/css/custom.css`
- [x] T032 [US3] Add responsive styles at `@media (max-width: 576px)` for small mobile in `frontend_book/src/css/custom.css`
- [x] T033 [P] [US3] Update `.heroBanner` responsive styles in `frontend_book/src/pages/index.module.css`
- [x] T034 [US3] Style `.navbar-sidebar` (mobile drawer) in `frontend_book/src/css/custom.css`
- [x] T035 [US3] Verify no horizontal scroll on mobile viewports (320px minimum)
- [x] T036 [US3] Test touch interactions work smoothly on mobile

**Checkpoint**: User Story 3 complete - site is fully responsive and touch-friendly

---

## Phase 6: User Story 4 - Reader Views Homepage Features (Priority: P2)

**Goal**: Homepage feature cards create strong first impression with balanced visual design

**Independent Test**: View homepage, scroll to feature section, verify cards are visually balanced with proper icons, titles, and descriptions

### Implementation for User Story 4

- [x] T037 [P] [US4] Update `.features` section container in `frontend_book/src/components/HomepageFeatures/styles.module.css`
- [x] T038 [P] [US4] Style `.featureSvg` icons in `frontend_book/src/components/HomepageFeatures/styles.module.css`
- [x] T039 [US4] Add feature card container styling (background, border-radius, shadow) in `frontend_book/src/components/HomepageFeatures/styles.module.css`
- [x] T040 [US4] Style feature card text (title, description) in `frontend_book/src/components/HomepageFeatures/styles.module.css`
- [x] T041 [P] [US4] Update `.buttons` container styling in `frontend_book/src/pages/index.module.css`
- [x] T042 [US4] Style `.button--primary` and `.button--secondary` in `frontend_book/src/css/custom.css`
- [x] T043 [US4] Add button hover and focus states in `frontend_book/src/css/custom.css`
- [x] T044 [US4] Verify feature cards render correctly in both themes

**Checkpoint**: User Story 4 complete - homepage has improved visual appeal

---

## Phase 7: User Story 5 - Reader Views Code Examples (Priority: P3)

**Goal**: Code blocks are readable and styled consistently with theme colors

**Independent Test**: View pages with code blocks, verify syntax highlighting is clear and container styling is consistent

### Implementation for User Story 5

- [x] T045 [US5] Override code block background variable (`--ifm-code-background`) in `frontend_book/src/css/custom.css`
- [x] T046 [US5] Style code block container (border-radius, padding) in `frontend_book/src/css/custom.css`
- [x] T047 [US5] Override inline code styling in `frontend_book/src/css/custom.css`
- [x] T048 [US5] Add dark mode code block styles in `frontend_book/src/css/custom.css`
- [x] T049 [US5] Ensure code blocks have horizontal scroll on mobile in `frontend_book/src/css/custom.css`
- [x] T050 [US5] Verify code blocks are readable in both themes

**Checkpoint**: User Story 5 complete - code examples have improved styling

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements affecting multiple user stories

- [x] T051 Run visual consistency audit across all pages in both themes
- [x] T052 Test edge cases: long titles, deeply nested sidebar, minimal content pages
- [x] T053 Verify footer stays at bottom on short pages (add sticky footer if needed)
- [ ] T054 Test on real mobile device (not just DevTools emulation)
- [x] T055 [P] Remove backup file created in T002 if changes are finalized
- [ ] T056 Run production build with `npm run build` and verify no errors
- [ ] T057 Final verification: run quickstart.md validation checklist

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - US1 and US2 are both P1, can proceed in parallel
  - US3 and US4 are both P2, can proceed in parallel after P1
  - US5 is P3, proceeds after P2
- **Polish (Phase 8)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational - No dependencies on other stories
- **User Story 3 (P2)**: Can start after Foundational - May refine styles from US1/US2
- **User Story 4 (P2)**: Can start after Foundational - No dependencies on other stories
- **User Story 5 (P3)**: Can start after Foundational - No dependencies on other stories

### File Conflict Avoidance

Most tasks modify `custom.css` - execute sequentially within each user story to avoid conflicts. Tasks marked [P] that modify different files can run in parallel:

- `custom.css` - Most component styling
- `index.module.css` - Homepage-specific styles (parallelizable)
- `HomepageFeatures/styles.module.css` - Feature cards (parallelizable)

---

## Parallel Examples

### Foundational Phase Parallel Tasks

```text
# Can run together (different variables, same file sections):
T006 Add typography variables
T007 Add custom spacing scale variables
T008 Add custom shadow variables
T009 Add custom transition variables
```

### User Story 1 + User Story 2 Parallel

```text
# Since both are P1 and modify different sections:
US1: T011-T018 (documentation content styling)
US2: T019-T028 (navigation component styling)
# However, coordinate on custom.css to avoid merge conflicts
```

### User Story 4 Parallel Tasks

```text
# Can run together (different files):
T037 Update .features in HomepageFeatures/styles.module.css
T038 Style .featureSvg in HomepageFeatures/styles.module.css
T041 Update .buttons in index.module.css
```

---

## Implementation Strategy

### MVP First (User Story 1 + 2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (colors, typography)
3. Complete Phase 3: User Story 1 (documentation readability)
4. Complete Phase 4: User Story 2 (navigation)
5. **STOP and VALIDATE**: Test core reading and navigation experience
6. Deploy/demo if ready

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add User Story 1 → Test readability → Deploy (Core MVP!)
3. Add User Story 2 → Test navigation → Deploy
4. Add User Story 3 → Test mobile → Deploy
5. Add User Story 4 → Test homepage → Deploy
6. Add User Story 5 → Test code blocks → Deploy
7. Polish → Final validation

---

## Task Summary

| Phase | User Story | Task Count | Key Focus | Status |
|-------|------------|------------|-----------|--------|
| 1 | Setup | 3 | Environment preparation | ✅ Complete |
| 2 | Foundational | 7 | Colors, typography, spacing | ✅ Complete |
| 3 | US1 (P1) | 8 | Documentation readability | ✅ Complete |
| 4 | US2 (P1) | 10 | Navigation components | ✅ Complete |
| 5 | US3 (P2) | 8 | Mobile responsiveness | ✅ Complete |
| 6 | US4 (P2) | 8 | Homepage features | ✅ Complete |
| 7 | US5 (P3) | 6 | Code block styling | ✅ Complete |
| 8 | Polish | 7 | Final validation | 🔄 In Progress |
| **Total** | | **57** | | **54/57** |

### Suggested MVP Scope

**MVP (Phases 1-4)**: 28 tasks ✅ COMPLETE
- Setup + Foundational + US1 + US2
- Core reading experience with improved typography and navigation

### Parallel Opportunities

- Foundational: 4 tasks parallelizable (T006-T009)
- US4: 3 tasks parallelizable (T037, T038, T041)
- Total parallelizable tasks: ~10 (marked with [P])

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Most tasks modify `custom.css` - coordinate to avoid conflicts
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Visual testing required after each phase (no automated tests)
