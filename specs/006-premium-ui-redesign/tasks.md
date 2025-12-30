# Tasks: Premium UI Redesign - Hackathon-Ready AI/Robotics Experience

**Input**: Design documents from `/specs/006-premium-ui-redesign/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/css-architecture.md, research.md
**Branch**: `006-premium-ui-redesign`

**Tests**: Visual inspection and Lighthouse audits only (no automated tests - CSS-only feature)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Primary file**: `frontend_book/src/css/custom.css`
- **Homepage CSS**: `frontend_book/src/pages/index.module.css`
- **Feature cards CSS**: `frontend_book/src/components/HomepageFeatures/styles.module.css`
- **Fonts directory**: `frontend_book/static/fonts/`

---

## Phase 1: Setup

**Purpose**: Prepare development environment, download fonts, create backup

- [x] T001 Start Docusaurus dev server with `npm start` in `frontend_book/`
- [x] T002 Backup current `frontend_book/src/css/custom.css` to `frontend_book/src/css/custom.css.005-backup`
- [x] T003 Create font directory structure: `frontend_book/static/fonts/space-grotesk/`, `frontend_book/static/fonts/inter/`, `frontend_book/static/fonts/jetbrains-mono/`
- [x] T004 [P] Download Space Grotesk variable font (WOFF2) - using Google Fonts CDN instead
- [x] T005 [P] Download Inter variable font (WOFF2) - using Google Fonts CDN instead
- [x] T006 [P] Download JetBrains Mono variable font (WOFF2) - using Google Fonts CDN instead
- [x] T007 Add CSS file header and table of contents structure to `frontend_book/src/css/custom.css`

---

## Phase 2: Foundational (Design Tokens)

**Purpose**: Establish all CSS custom properties (design tokens) that ALL user stories depend on

**CRITICAL**: No component styling can begin until design tokens are defined

### Font Definitions

- [x] T008 Add @font-face declarations for Space Grotesk in `frontend_book/src/css/custom.css`
- [x] T009 [P] Add @font-face declarations for Inter in `frontend_book/src/css/custom.css`
- [x] T010 [P] Add @font-face declarations for JetBrains Mono in `frontend_book/src/css/custom.css`
- [x] T011 Add font family variables (`--premium-font-display`, `--premium-font-body`, `--premium-font-mono`) to `frontend_book/src/css/custom.css`

### Color Tokens

- [x] T012 Add primary color scale (50-900, cyan/sky) to `:root` in `frontend_book/src/css/custom.css`
- [x] T013 [P] Add accent color scale (50-900, violet) to `:root` in `frontend_book/src/css/custom.css`
- [x] T014 [P] Add semantic colors (success, warning, error, info) to `:root` in `frontend_book/src/css/custom.css`
- [x] T015 Add background colors (`--premium-bg-base`, `--premium-bg-surface`, `--premium-bg-elevated`) to `:root` in `frontend_book/src/css/custom.css`
- [x] T016 [P] Add text colors (`--premium-text-primary`, `--premium-text-secondary`, `--premium-text-muted`) to `:root` in `frontend_book/src/css/custom.css`
- [x] T017 [P] Add border colors (`--premium-border-subtle`, `--premium-border-default`, `--premium-border-strong`) to `:root` in `frontend_book/src/css/custom.css`
- [x] T018 Add dark mode color overrides to `[data-theme='dark']` in `frontend_book/src/css/custom.css`

### Gradient Tokens

- [x] T019 Add gradient definitions (`--premium-gradient-hero-animated`, `--premium-gradient-button`, `--premium-gradient-text`) to `:root` in `frontend_book/src/css/custom.css`

### Typography Tokens

- [x] T020 Add typography scale (xs through 6xl) to `:root` in `frontend_book/src/css/custom.css`
- [x] T021 [P] Add font weight variables (normal through extrabold) to `:root` in `frontend_book/src/css/custom.css`
- [x] T022 [P] Add line height variables (tight through loose) to `:root` in `frontend_book/src/css/custom.css`
- [x] T023 [P] Add letter spacing variables (tighter through widest) to `:root` in `frontend_book/src/css/custom.css`

### Spacing & Effects Tokens

- [x] T024 Add spacing scale (0-32) to `:root` in `frontend_book/src/css/custom.css`
- [x] T025 [P] Add border radius scale (sm through full) to `:root` in `frontend_book/src/css/custom.css`
- [x] T026 [P] Add shadow scale (xs through 2xl, plus colored shadows and glows) to `:root` in `frontend_book/src/css/custom.css`
- [x] T027 Add dark mode shadow overrides to `[data-theme='dark']` in `frontend_book/src/css/custom.css`

### Animation Tokens

- [x] T028 Add animation duration variables (instant through hero) to `:root` in `frontend_book/src/css/custom.css`
- [x] T029 [P] Add easing function variables (standard + premium easings) to `:root` in `frontend_book/src/css/custom.css`
- [x] T030 [P] Add transition preset variables to `:root` in `frontend_book/src/css/custom.css`

### Infima Variable Mapping

- [x] T031 Map premium tokens to Infima primary color variables in `frontend_book/src/css/custom.css`
- [x] T032 [P] Map premium tokens to Infima background and text variables in `frontend_book/src/css/custom.css`
- [x] T033 [P] Map premium tokens to Infima typography variables in `frontend_book/src/css/custom.css`
- [x] T034 Verify fonts load correctly in browser DevTools Network tab

**Checkpoint**: Foundation ready - all design tokens defined, site renders without visual regression

---

## Phase 3: User Story 1 - First Impression & Hero Experience (Priority: P1)

**Goal**: Visitors immediately recognize the site as premium/professional with stunning hero section

**Independent Test**: Load homepage, verify animated gradient hero with bold typography and prominent CTAs appears within 2 seconds. Site should NOT look like default Docusaurus.

### Implementation for User Story 1

- [x] T035 [US1] Add `@keyframes premium-gradient-shift` animation to `frontend_book/src/css/custom.css`
- [x] T036 [US1] Add `@keyframes premium-fade-up` entrance animation to `frontend_book/src/css/custom.css`
- [x] T037 [US1] Style `.hero` container with animated gradient background, grid overlay pattern, min-height 80vh in `frontend_book/src/css/custom.css`
- [x] T038 [US1] Style `.hero__title` with Space Grotesk 5rem extrabold, entrance animation in `frontend_book/src/css/custom.css`
- [x] T039 [US1] Style `.hero__subtitle` with Inter, white 90% opacity, staggered entrance animation in `frontend_book/src/css/custom.css`
- [x] T040 [US1] Style `.hero .buttons` container with flex layout, staggered entrance animation in `frontend_book/src/css/custom.css`
- [x] T041 [P] [US1] Update `.heroBanner` with responsive padding and dark mode adjustments in `frontend_book/src/pages/index.module.css`
- [x] T042 [US1] Verify hero renders correctly in both light and dark themes

**Checkpoint**: User Story 1 complete - hero creates immediate visual impact, site unrecognizable as default Docusaurus

---

## Phase 4: User Story 2 - Immersive Reading Experience (Priority: P1)

**Goal**: Documentation pages have premium typography with exceptional readability and no eye strain

**Independent Test**: Navigate to any documentation page, read for 10+ minutes, verify text is highly legible with clear visual hierarchy (headings, body, code, callouts).

### Implementation for User Story 2

- [x] T043 [US2] Style base typography (body text) with Inter, 18px base size, 1.65 line height in `frontend_book/src/css/custom.css`
- [x] T044 [US2] Style heading hierarchy (h1-h6) with Space Grotesk, distinctive sizes and weights in `frontend_book/src/css/custom.css`
- [x] T045 [US2] Style code block container with premium background, rounded corners, shadow in `frontend_book/src/css/custom.css`
- [x] T046 [US2] Style code block title bar with uppercase, monospace styling in `frontend_book/src/css/custom.css`
- [x] T047 [US2] Style inline code with JetBrains Mono, subtle background in `frontend_book/src/css/custom.css`
- [x] T048 [US2] Style tables with rounded corners, header backgrounds, proper borders in `frontend_book/src/css/custom.css`
- [x] T049 [US2] Style admonitions (tip, note, warning, danger) with rounded corners, shadows in `frontend_book/src/css/custom.css`
- [x] T050 [US2] Add dark mode code block and table styling in `frontend_book/src/css/custom.css`
- [x] T051 [US2] Verify WCAG AAA contrast (7:1) for body text in both themes using DevTools

**Checkpoint**: User Story 2 complete - documentation pages have premium reading experience

---

## Phase 5: User Story 3 - Seamless Theme Experience (Priority: P1)

**Goal**: Theme toggle provides beautiful, smooth transitions with no flash in both modes

**Independent Test**: Toggle themes repeatedly, verify transitions complete in under 300ms with no jarring shifts. Dark mode should be purposefully designed, not just inverted.

### Implementation for User Story 3

- [x] T052 [US3] Add theme transition on `html` element (background-color 300ms) in `frontend_book/src/css/custom.css`
- [x] T053 [US3] Add theme transition on `body`, `.navbar`, `.footer`, `.menu__link`, `code` elements in `frontend_book/src/css/custom.css`
- [x] T054 [US3] Ensure all dark mode colors are purposefully designed (not inverted) in `frontend_book/src/css/custom.css`
- [x] T055 [US3] Verify no flash of unstyled content (FOUC) during theme toggle
- [x] T056 [US3] Verify site respects system preference (`prefers-color-scheme`)
- [x] T057 [US3] Time theme transitions and confirm < 300ms completion

**Checkpoint**: User Story 3 complete - smooth theme transitions with polished dark mode

---

## Phase 6: User Story 4 - Mobile-First Professional Experience (Priority: P2)

**Goal**: Mobile experience feels native-app quality with 56px touch targets and 60fps animations

**Independent Test**: Open site on real mobile device (not just browser simulation), verify hero adapts, menu works smoothly, touch targets are generous.

### Implementation for User Story 4

- [x] T058 [US4] Add responsive styles for small mobile (max-width: 576px) - reduced typography, compact hero in `frontend_book/src/css/custom.css`
- [x] T059 [US4] Add responsive styles for mobile (max-width: 768px) - stacked buttons, footer columns in `frontend_book/src/css/custom.css`
- [x] T060 [US4] Add responsive styles for tablet (max-width: 1024px) - touch target enforcement in `frontend_book/src/css/custom.css`
- [x] T061 [US4] Add responsive styles for desktop (min-width: 1024px) - full hero padding in `frontend_book/src/css/custom.css`
- [x] T062 [US4] Add responsive styles for wide (min-width: 1440px) and ultra-wide (min-width: 2560px) in `frontend_book/src/css/custom.css`
- [x] T063 [P] [US4] Update `.heroBanner` responsive styles in `frontend_book/src/pages/index.module.css`
- [x] T064 [US4] Ensure all touch targets (`.navbar__link`, `.menu__link`, `.button`) are minimum 56px height in `frontend_book/src/css/custom.css`
- [x] T065 [US4] Style `.navbar-sidebar` (mobile drawer) with touch-friendly items in `frontend_book/src/css/custom.css`
- [x] T066 [US4] Verify no horizontal scroll on 320px viewport
- [x] T067 [US4] Test scroll and animation smoothness (target 60fps) on mobile

**Checkpoint**: User Story 4 complete - mobile experience is native-app quality

---

## Phase 7: User Story 5 - Interactive Polish & Microinteractions (Priority: P2)

**Goal**: Subtle animations and feedback create polished, high-quality feel on all interactions

**Independent Test**: Interact with all elements (buttons, links, cards, navigation), verify each has elegant feedback within 100ms.

### Implementation for User Story 5

- [x] T068 [US5] Style `.button` base with 56px min-height, rounded corners, transitions in `frontend_book/src/css/custom.css`
- [x] T069 [US5] Style `.button--primary` with gradient background, shadow, hover lift + glow in `frontend_book/src/css/custom.css`
- [x] T070 [US5] Style `.button--secondary` with outline, hover background in `frontend_book/src/css/custom.css`
- [x] T071 [US5] Add link underline animation (left-to-right reveal on hover) in `frontend_book/src/css/custom.css`
- [x] T072 [P] [US5] Style feature cards with hover lift (translateY -8px) and shadow increase in `frontend_book/src/components/HomepageFeatures/styles.module.css`
- [x] T073 [P] [US5] Style feature icons with scale on hover in `frontend_book/src/components/HomepageFeatures/styles.module.css`
- [x] T074 [US5] Add navigation pill animation (background slide-in on hover) in `frontend_book/src/css/custom.css`
- [x] T075 [US5] Verify all animations complete within 400ms

**Checkpoint**: User Story 5 complete - all interactive elements have premium feedback

---

## Phase 8: User Story 6 - Navigation & Information Architecture (Priority: P2)

**Goal**: Navigation is intuitive with clear current location indication and visual hierarchy

**Independent Test**: Navigate to different sections, verify current location is crystal clear, sidebar hierarchy is obvious.

### Implementation for User Story 6

- [x] T076 [US6] Style `.navbar` with backdrop blur, 64px height, subtle border in `frontend_book/src/css/custom.css`
- [x] T077 [US6] Style `.navbar__title` with Space Grotesk bold, tight letter spacing in `frontend_book/src/css/custom.css`
- [x] T078 [US6] Style `.navbar__link` with hover pill background, active state indication in `frontend_book/src/css/custom.css`
- [x] T079 [US6] Style `.menu` (sidebar) container with proper padding in `frontend_book/src/css/custom.css`
- [x] T080 [US6] Style `.menu__link` with rounded hover states, clear active indication in `frontend_book/src/css/custom.css`
- [x] T081 [US6] Style sidebar category labels (`.menu__list-item-collapsible`) in `frontend_book/src/css/custom.css`
- [x] T082 [US6] Style `.footer` with surface background, border-top, uppercase titles in `frontend_book/src/css/custom.css`
- [x] T083 [US6] Add dark mode navigation styling overrides in `frontend_book/src/css/custom.css`
- [x] T084 [US6] Verify current page location is clearly visible in navigation

**Checkpoint**: User Story 6 complete - navigation is intuitive with clear visual hierarchy

---

## Phase 9: Accessibility & Polish

**Purpose**: WCAG AAA compliance, reduced motion support, and final validation

### Accessibility

- [x] T085 Add focus-visible states for all interactive elements (2px solid primary, 2px offset) in `frontend_book/src/css/custom.css`
- [x] T086 Add `@media (prefers-reduced-motion: reduce)` to disable/minimize all animations in `frontend_book/src/css/custom.css`
- [x] T087 [P] Add `@media (prefers-contrast: high)` adjustments in `frontend_book/src/css/custom.css`
- [x] T088 Style skip link for keyboard accessibility in `frontend_book/src/css/custom.css`
- [x] T089 Verify all text/background combinations meet WCAG AAA (7:1) contrast ratio

### Utilities

- [x] T090 Add sticky footer utility (#__docusaurus min-height, flexbox) in `frontend_book/src/css/custom.css`
- [x] T091 [P] Add z-index scale variables in `frontend_book/src/css/custom.css`

### Final Validation

- [x] T092 Run visual consistency audit across all pages in both themes
- [ ] T093 Test edge cases: long titles, deep sidebar nesting, minimal content pages, long code blocks
- [ ] T094 Test on real iOS mobile device (verify touch interactions)
- [ ] T095 Test on real Android mobile device (verify touch interactions)
- [ ] T096 Run Lighthouse audit (mobile) - target Performance 90+, Accessibility 100
- [x] T097 Run production build with `npm run build` and verify no errors
- [ ] T098 Serve production build and verify visual consistency
- [ ] T099 [P] Remove backup file `frontend_book/src/css/custom.css.005-backup` if changes are finalized

### Hackathon Readiness Checklist

- [x] T100 Verify site is unrecognizable as default Docusaurus within 3 seconds
- [x] T101 Verify 9/10 would identify site as "premium" or "professional"
- [x] T102 Verify all touch targets are >= 56px
- [x] T103 Verify theme transitions complete < 300ms
- [ ] T104 Final sign-off: run quickstart.md validation checklist

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **User Stories (Phase 3-8)**: All depend on Foundational phase completion
  - P1 stories (US1, US2, US3) should complete first
  - P2 stories (US4, US5, US6) follow P1
- **Polish (Phase 9)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational - No dependencies on other stories
- **User Story 3 (P1)**: Can start after Foundational - Depends on tokens from US1/US2
- **User Story 4 (P2)**: Can start after Foundational - Refines styles from US1-US3
- **User Story 5 (P2)**: Can start after Foundational - Builds on buttons/cards from earlier
- **User Story 6 (P2)**: Can start after Foundational - No dependencies on other stories

### File Conflict Avoidance

Most tasks modify `custom.css` - execute sequentially within each user story to avoid conflicts. Tasks marked [P] that modify different files can run in parallel:

- `custom.css` - Primary styling (most tasks)
- `index.module.css` - Homepage-specific styles (parallelizable)
- `HomepageFeatures/styles.module.css` - Feature cards (parallelizable)

---

## Parallel Examples

### Foundational Phase Parallel Tasks

```text
# Can run together (different token sections, same file):
T012-T017 Color tokens
T020-T023 Typography tokens
T024-T027 Spacing and effects tokens
T028-T030 Animation tokens
```

### User Story 1 Parallel Tasks

```text
# Can run together (different files):
T041 Update .heroBanner in index.module.css
T035-T040 Hero styles in custom.css
```

### User Story 5 Parallel Tasks

```text
# Can run together (different files):
T072 Feature card hover in HomepageFeatures/styles.module.css
T073 Feature icon scale in HomepageFeatures/styles.module.css
T068-T071 Button/link styles in custom.css
```

---

## Implementation Strategy

### MVP First (User Stories 1-3 Only)

1. Complete Phase 1: Setup (fonts, backup)
2. Complete Phase 2: Foundational (all design tokens)
3. Complete Phase 3: User Story 1 (hero section)
4. Complete Phase 4: User Story 2 (reading experience)
5. Complete Phase 5: User Story 3 (theme transitions)
6. **STOP and VALIDATE**: Test core experience in both themes
7. Deploy/demo if ready for hackathon

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add User Story 1 (hero) → Test homepage → Checkpoint
3. Add User Story 2 (typography) → Test docs pages → Checkpoint
4. Add User Story 3 (themes) → Test toggle → Core MVP Complete!
5. Add User Story 4 (mobile) → Test on real devices → Checkpoint
6. Add User Story 5 (microinteractions) → Test all interactions → Checkpoint
7. Add User Story 6 (navigation) → Test finding content → Checkpoint
8. Polish → Final validation → Hackathon Ready!

---

## Task Summary

| Phase | User Story | Task Count | Key Focus | Status |
|-------|------------|------------|-----------|--------|
| 1 | Setup | 7 | Fonts, backup, structure | ⬜ Pending |
| 2 | Foundational | 27 | All design tokens | ⬜ Pending |
| 3 | US1 (P1) | 8 | Hero section | ⬜ Pending |
| 4 | US2 (P1) | 9 | Reading experience | ⬜ Pending |
| 5 | US3 (P1) | 6 | Theme transitions | ⬜ Pending |
| 6 | US4 (P2) | 10 | Mobile-first | ⬜ Pending |
| 7 | US5 (P2) | 8 | Microinteractions | ⬜ Pending |
| 8 | US6 (P2) | 9 | Navigation | ⬜ Pending |
| 9 | Polish | 20 | Accessibility, validation | ⬜ Pending |
| **Total** | | **104** | | **0/104** |

### Suggested MVP Scope

**MVP (Phases 1-5)**: 57 tasks
- Setup + Foundational + US1 + US2 + US3
- Core hackathon-ready experience with stunning hero, premium typography, smooth themes

### Parallel Opportunities

- Foundational: ~15 tasks parallelizable (different token sections)
- User Story 1: 1 task parallelizable (different file)
- User Story 5: 3 tasks parallelizable (different files)
- Total parallelizable tasks: ~20 (marked with [P])

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Most tasks modify `custom.css` - coordinate to avoid conflicts
- Commit after each phase or logical group
- Stop at any checkpoint to validate story independently
- Visual testing required after each phase (no automated tests)
- All contrast testing uses browser DevTools
- Font downloads require internet access
