# CSS Architecture Contract

**Feature**: 005-docusaurus-ui-upgrade
**Date**: 2025-12-30

## Overview

This contract defines the CSS organization, naming conventions, and modification patterns for the UI upgrade.

---

## File Organization

### Primary Files

```text
frontend_book/src/css/custom.css
├── /* ============================================
│    * 1. COLOR PALETTE
│    * ============================================ */
├── :root { /* Light mode colors */ }
├── [data-theme='dark'] { /* Dark mode colors */ }
│
├── /* ============================================
│    * 2. TYPOGRAPHY
│    * ============================================ */
├── :root { /* Font sizes, weights, line heights */ }
│
├── /* ============================================
│    * 3. SPACING & LAYOUT
│    * ============================================ */
├── :root { /* Spacing scale, radii */ }
│
├── /* ============================================
│    * 4. COMPONENT OVERRIDES
│    * ============================================ */
├── /* 4.1 Navbar */
├── /* 4.2 Sidebar */
├── /* 4.3 Footer */
├── /* 4.4 Code Blocks */
├── /* 4.5 Buttons */
│
├── /* ============================================
│    * 5. RESPONSIVE ADJUSTMENTS
│    * ============================================ */
├── @media (max-width: 996px) {}
├── @media (max-width: 768px) {}
└── @media (max-width: 576px) {}
```

### Component Files

```text
frontend_book/src/pages/index.module.css
├── .heroBanner { /* Hero section styling */ }
├── .buttons { /* CTA button container */ }
└── /* Additional homepage-specific styles */

frontend_book/src/components/HomepageFeatures/styles.module.css
├── .features { /* Features section container */ }
├── .featureSvg { /* Feature icons/images */ }
└── /* Additional feature card styles */
```

---

## CSS Variable Naming

### Infima Variables (Override These)

```css
/* Color variables - use full 7-shade scale */
--ifm-color-primary: value;
--ifm-color-primary-dark: value;
--ifm-color-primary-darker: value;
--ifm-color-primary-darkest: value;
--ifm-color-primary-light: value;
--ifm-color-primary-lighter: value;
--ifm-color-primary-lightest: value;

/* Typography variables */
--ifm-font-size-base: value;
--ifm-line-height-base: value;
--ifm-heading-font-weight: value;

/* Spacing variables */
--ifm-global-spacing: value;
--ifm-spacing-horizontal: value;
--ifm-spacing-vertical: value;
```

### Custom Variables (Add These)

Use `--custom-` prefix for project-specific variables:

```css
:root {
  /* Custom spacing scale */
  --custom-space-xs: 0.25rem;
  --custom-space-sm: 0.5rem;
  --custom-space-md: 1rem;
  --custom-space-lg: 1.5rem;
  --custom-space-xl: 2rem;

  /* Custom shadows */
  --custom-shadow-sm: 0 1px 2px rgba(0, 0, 0, 0.05);
  --custom-shadow-md: 0 4px 6px rgba(0, 0, 0, 0.1);
  --custom-shadow-lg: 0 10px 15px rgba(0, 0, 0, 0.1);

  /* Custom transitions */
  --custom-transition-fast: 150ms ease;
  --custom-transition-base: 200ms ease-in-out;
}
```

---

## Selector Patterns

### DO - Target These Selectors

```css
/* Theme class names (stable) */
.navbar { }
.hero { }
.footer { }
.menu { }

/* Infima utility classes (stable) */
.button { }
.button--primary { }
.button--secondary { }

/* Data attributes for theming */
[data-theme='dark'] { }
[data-theme='light'] { }

/* Custom component classes */
.heroBanner { }
.features { }
```

### AVOID - Don't Target These

```css
/* CSS module hashes (unstable) */
.codeBlockContainer_RIuc { } /* ❌ Hash changes on build */

/* Internal component classes */
.theme-doc-sidebar-item-link-level-1 { } /* ❌ May change */

/* Over-specific selectors */
.navbar .navbar__inner .navbar__items .navbar__link { } /* ❌ Too fragile */
```

### WORKAROUND - When Unavoidable

```css
/* Use attribute selector for CSS module classes */
[class*='codeBlockContainer'] { } /* ✓ Ignores hash */
```

---

## Modification Rules

### Rule 1: Variables First

Always try to achieve the desired effect by overriding CSS variables before adding custom CSS rules.

```css
/* ✓ Good - Override variable */
:root {
  --ifm-color-primary: #3b82f6;
}

/* ✗ Avoid - Direct property override */
.navbar a {
  color: #3b82f6;
}
```

### Rule 2: Minimal Specificity

Use the lowest specificity that works.

```css
/* ✓ Good - Single class */
.navbar { }

/* ✗ Avoid - Unnecessary nesting */
.navbar .navbar__inner .navbar__items { }
```

### Rule 3: Mobile-First

Write base styles, then enhance for larger screens.

```css
/* Base (mobile) */
.hero {
  padding: 2rem 1rem;
}

/* Enhanced (desktop) */
@media (min-width: 997px) {
  .hero {
    padding: 4rem 2rem;
  }
}
```

### Rule 4: Theme-Aware

Always consider both themes when adding custom styles.

```css
/* Light mode */
.card {
  background: var(--ifm-background-surface-color);
  box-shadow: var(--custom-shadow-md);
}

/* Dark mode */
[data-theme='dark'] .card {
  box-shadow: none;
  border: 1px solid rgba(255, 255, 255, 0.1);
}
```

---

## Responsive Breakpoints

### Standard Breakpoints

```css
/* Tablet - navbar collapses */
@media (max-width: 996px) {
  /* Mobile navigation active */
}

/* Mobile */
@media (max-width: 768px) {
  /* Single column layouts */
}

/* Small mobile */
@media (max-width: 576px) {
  /* Compact spacing */
}
```

### Touch Target Requirements

```css
/* Minimum 44x44px for touch targets */
@media (max-width: 996px) {
  .navbar__link,
  .menu__link {
    min-height: 44px;
    padding: 0.75rem 1rem;
  }
}
```

---

## Component Contracts

### Navbar Contract

| Selector | Properties Modified | Notes |
|----------|---------------------|-------|
| `.navbar` | background, shadow, height | Main container |
| `.navbar__link` | color, hover, padding | Navigation links |
| `.navbar__title` | font-weight, color | Site title |
| `.navbar-sidebar` | background, width | Mobile drawer |

### Sidebar Contract

| Selector | Properties Modified | Notes |
|----------|---------------------|-------|
| `.menu` | background, padding | Container |
| `.menu__link` | color, hover states | Links |
| `.menu__link--active` | background, color, weight | Current page |
| `.theme-doc-sidebar-item-category` | font-weight | Category labels |

### Footer Contract

| Selector | Properties Modified | Notes |
|----------|---------------------|-------|
| `.footer` | background, padding | Container |
| `.footer__title` | font-size, weight | Section headers |
| `.footer__link` | color, hover | Links |
| `.footer__copyright` | color, font-size | Copyright text |

### Hero Contract

| Selector | Properties Modified | Notes |
|----------|---------------------|-------|
| `.hero` | background, padding | Container |
| `.hero__title` | font-size, weight | Main heading |
| `.hero__subtitle` | font-size, color | Tagline |
| `.heroBanner` | background, gradient | CSS module |

---

## Testing Checklist

Before merging CSS changes:

- [ ] Light mode renders correctly
- [ ] Dark mode renders correctly
- [ ] No horizontal scroll on mobile (320px)
- [ ] Touch targets are 44x44px minimum
- [ ] Contrast meets WCAG AA (4.5:1)
- [ ] No visual regressions in existing components
- [ ] Code blocks are readable in both themes
- [ ] Footer stays at bottom on short pages
