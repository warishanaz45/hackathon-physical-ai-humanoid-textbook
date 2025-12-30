# Research: Docusaurus UI Upgrade

**Feature**: 005-docusaurus-ui-upgrade
**Date**: 2025-12-30
**Branch**: 005-docusaurus-ui-upgrade

## Research Questions

### 1. What is the best approach to customize Docusaurus styling?

**Decision**: Use CSS variables override in `custom.css` as primary approach, with targeted CSS rules for component-specific styling.

**Rationale**:
- CSS variables provide centralized theming without modifying core files
- Ensures clean upgrades when updating Docusaurus
- Variables control colors, spacing, fonts across all components
- Follows official Docusaurus recommendation

**Alternatives Considered**:
- Swizzling components (ejecting): Creates maintenance burden on upgrades, more code to maintain
- Using external CSS framework: Would conflict with Infima, adds complexity
- Modifying core theme files: Not recommended, breaks on updates

**Sources**:
- [Docusaurus Styling and Layout](https://docusaurus.io/docs/styling-layout)
- [Docusaurus Community CSS Variables](https://docusaurus.community/knowledge/design/css/variables/)

---

### 2. How does Docusaurus/Infima color system work?

**Decision**: Override the 7-shade Infima color variables for primary, secondary, and accent colors.

**Rationale**:
- Infima uses consistent naming: `--ifm-color-{color}` with 7 shades per color
- 7 shades: base, dark, darker, darkest, light, lighter, lightest
- Dark mode scoped via `[data-theme='dark']` selector
- All components use these variables, so changing them updates entire site

**Color Variable Pattern**:
```css
:root {
  --ifm-color-primary: #2e8555;
  --ifm-color-primary-dark: #29784c;
  --ifm-color-primary-darker: #277148;
  --ifm-color-primary-darkest: #205d3b;
  --ifm-color-primary-light: #33925d;
  --ifm-color-primary-lighter: #359962;
  --ifm-color-primary-lightest: #3cad6e;
}
```

**Alternatives Considered**:
- Custom color system: Would require overriding many individual properties
- Using ColorBox tool: Recommended for generating shade scales from a base color

**Sources**:
- [Infima CSS Framework](https://infima.dev/)
- [GitHub Gist: Complete Infima Variables](https://gist.github.com/abhigyantrips/b828ca46b2460c6699c73e0162f6be80)

---

### 3. When should swizzling be used vs CSS customization?

**Decision**: Avoid swizzling unless absolutely necessary; use CSS-only approach for this UI upgrade.

**Rationale**:
- Swizzling creates maintenance burden on Docusaurus upgrades
- "Unsafe" swizzled components may break between minor versions
- CSS customization is sufficient for visual design changes
- Wrapping (if needed) is preferred over ejecting

**When Swizzling is Needed**:
- Adding entirely new navbar item types (use `custom-` prefix)
- Modifying component behavior/logic, not just appearance
- Adding interactive elements not supported by config

**When CSS is Sufficient**:
- Changing colors, typography, spacing
- Modifying hover/active states
- Adjusting responsive breakpoints
- Styling existing components

**Sources**:
- [Docusaurus Swizzling Guide](https://docusaurus.io/docs/swizzling)

---

### 4. What CSS class naming patterns should be targeted?

**Decision**: Target theme class names and Infima class names; avoid CSS module class names.

**Rationale**:
- Theme class names are stable and documented
- Infima class names (BEM: block__element--modifier) are relatively stable
- CSS module class names contain hashes that change between builds

**Class Types**:
| Type | Pattern | Stability | Target? |
|------|---------|-----------|---------|
| Theme class | Listed in docs | Stable | Yes |
| Infima class | `navbar`, `hero`, `footer` | Mostly stable | Yes, with care |
| CSS module | `codeBlockContainer_RIuc` | Unstable | Avoid |

**If Targeting CSS Module Classes**:
Use attribute selector: `[class*='codeBlockContainer']`

---

### 5. What typography and spacing variables are available?

**Decision**: Use Infima typography and spacing variables for consistent scaling.

**Key Typography Variables**:
```css
:root {
  --ifm-font-family-base: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
  --ifm-font-size-base: 100%;
  --ifm-font-weight-normal: 400;
  --ifm-font-weight-semibold: 600;
  --ifm-font-weight-bold: 700;
  --ifm-line-height-base: 1.65;
  --ifm-heading-font-weight: var(--ifm-font-weight-bold);
}
```

**Key Spacing Variables**:
```css
:root {
  --ifm-spacing-horizontal: 1rem;
  --ifm-spacing-vertical: 1rem;
  --ifm-global-spacing: 1rem;
  --ifm-global-radius: 0.4rem;
}
```

---

### 6. How to ensure accessibility contrast ratios?

**Decision**: Test all color combinations against WCAG AA standards (4.5:1 normal text, 3:1 large text).

**Approach**:
- Use browser dev tools to verify contrast
- Test both light and dark themes
- Use ColorBox or similar tool to generate accessible shade scales
- Primary button text must contrast with primary color background

**Tools**:
- Chrome DevTools Accessibility Inspector
- WebAIM Contrast Checker
- ColorBox for shade generation

---

### 7. What responsive breakpoints does Docusaurus use?

**Decision**: Follow existing Docusaurus/Infima breakpoints.

**Infima Breakpoints**:
```css
/* Default Docusaurus breakpoints */
@media (max-width: 996px)  /* Tablet and below */
@media (max-width: 768px)  /* Mobile */
@media (max-width: 576px)  /* Small mobile */
```

**Note**: `996px` is the Docusaurus navbar collapse breakpoint.

---

## Current Site Analysis

Based on codebase review:

**Current `custom.css` State**:
- Uses default Docusaurus green palette (#2e8555 primary)
- Minimal customization (only Infima variables)
- Dark mode uses teal (#25c2a0)

**Current Component Structure**:
- `src/pages/index.js` - Homepage with hero and features
- `src/components/HomepageFeatures/` - Feature cards component
- `src/css/custom.css` - Global styles

**Files to Modify**:
1. `src/css/custom.css` - Primary theming file
2. `src/pages/index.module.css` - Homepage-specific styles
3. `src/components/HomepageFeatures/styles.module.css` - Feature cards

---

## Design Decisions Summary

| Aspect | Decision | Implementation |
|--------|----------|----------------|
| Theming Approach | CSS Variables | Override in `custom.css` |
| Color System | 7-shade Infima pattern | New primary color with full shade scale |
| Typography | Infima variables | Adjust line-heights, font sizes |
| Spacing | Infima spacing scale | Consistent use of spacing variables |
| Components | CSS-only styling | No swizzling required |
| Dark Mode | `[data-theme='dark']` | Separate color palette |
| Responsive | Use existing breakpoints | 996px, 768px, 576px |
| Accessibility | WCAG AA | 4.5:1 contrast minimum |

---

## Technology Stack Confirmation

| Component | Technology | Notes |
|-----------|------------|-------|
| Framework | Docusaurus 3.x | Already installed |
| CSS Framework | Infima | Built-in with Docusaurus |
| Styling | CSS Variables + CSS Modules | No additional dependencies |
| Build | npm + Webpack | Existing configuration |
