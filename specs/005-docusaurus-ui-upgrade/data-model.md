# Data Model: Docusaurus UI Upgrade

**Feature**: 005-docusaurus-ui-upgrade
**Date**: 2025-12-30

## Overview

This feature does not involve traditional data entities (database, API models). Instead, it defines **design tokens** - the CSS variables and values that form the visual language of the site.

---

## Design Token Entities

### 1. Color Theme

**Purpose**: Define the complete color palette for light and dark modes.

**Light Mode Tokens**:

| Token | Property | Value | Usage |
|-------|----------|-------|-------|
| Primary | `--ifm-color-primary` | TBD | Links, buttons, accents |
| Primary Dark | `--ifm-color-primary-dark` | TBD | Hover states |
| Primary Darker | `--ifm-color-primary-darker` | TBD | Active states |
| Primary Darkest | `--ifm-color-primary-darkest` | TBD | Strong emphasis |
| Primary Light | `--ifm-color-primary-light` | TBD | Subtle accents |
| Primary Lighter | `--ifm-color-primary-lighter` | TBD | Backgrounds |
| Primary Lightest | `--ifm-color-primary-lightest` | TBD | Hover backgrounds |
| Background | `--ifm-background-color` | #ffffff | Page background |
| Background Surface | `--ifm-background-surface-color` | #f8f9fa | Card backgrounds |
| Text | `--ifm-font-color-base` | #1c1e21 | Body text |
| Text Secondary | `--ifm-font-color-secondary` | #525860 | Secondary text |

**Dark Mode Tokens** (`[data-theme='dark']`):

| Token | Property | Value | Usage |
|-------|----------|-------|-------|
| Primary | `--ifm-color-primary` | TBD | Links, buttons |
| Background | `--ifm-background-color` | #1b1b1d | Page background |
| Background Surface | `--ifm-background-surface-color` | #242526 | Card backgrounds |
| Text | `--ifm-font-color-base` | #e3e3e3 | Body text |

**Validation Rules**:
- Primary color must have WCAG AA contrast (4.5:1) against white text
- Background/text combinations must meet 4.5:1 contrast
- All 7 shades must be mathematically derived from base primary

---

### 2. Typography Scale

**Purpose**: Define consistent typography across the site.

| Token | Property | Value | Usage |
|-------|----------|-------|-------|
| Font Base | `--ifm-font-size-base` | 100% | Root font size (16px) |
| Font Family | `--ifm-font-family-base` | System stack | Body text |
| Font Mono | `--ifm-font-family-monospace` | Monospace stack | Code |
| Line Height | `--ifm-line-height-base` | 1.65 | Body text |
| Heading Weight | `--ifm-heading-font-weight` | 700 | All headings |
| H1 Size | `--ifm-h1-font-size` | 2.5rem | Main titles |
| H2 Size | `--ifm-h2-font-size` | 2rem | Section headers |
| H3 Size | `--ifm-h3-font-size` | 1.5rem | Subsections |
| H4 Size | `--ifm-h4-font-size` | 1.25rem | Minor headers |
| H5 Size | `--ifm-h5-font-size` | 1rem | Small headers |
| H6 Size | `--ifm-h6-font-size` | 0.875rem | Smallest headers |
| Code Font Size | `--ifm-code-font-size` | 90% | Inline/block code |

**Validation Rules**:
- Body text minimum 16px (1rem)
- Line height between 1.5-1.7 for readability
- Heading scale should be harmonious (golden ratio or similar)

---

### 3. Spacing Scale

**Purpose**: Consistent spacing for margins, padding, and gaps.

| Token | Property | Value | Usage |
|-------|----------|-------|-------|
| Global Spacing | `--ifm-global-spacing` | 1rem | Base unit |
| Spacing Horizontal | `--ifm-spacing-horizontal` | 1rem | Horizontal gaps |
| Spacing Vertical | `--ifm-spacing-vertical` | 1rem | Vertical gaps |
| Container Padding | `--ifm-container-padding` | 0 1rem | Page margins |
| Paragraph Margin | `--ifm-paragraph-margin-bottom` | 1rem | Text spacing |
| List Item Margin | `--ifm-list-item-margin` | 0.5rem | List spacing |

**Derived Spacing**:
- xs: 0.25rem (4px)
- sm: 0.5rem (8px)
- md: 1rem (16px)
- lg: 1.5rem (24px)
- xl: 2rem (32px)
- xxl: 3rem (48px)

---

### 4. Component Tokens

**Purpose**: Component-specific styling values.

#### Navbar
| Token | Property | Value |
|-------|----------|-------|
| Height | `--ifm-navbar-height` | 3.75rem |
| Background | `--ifm-navbar-background-color` | Derived from theme |
| Shadow | `--ifm-navbar-shadow` | 0 1px 2px rgba(0,0,0,0.1) |
| Link Hover | `--ifm-navbar-link-hover-color` | Primary color |
| Padding | `--ifm-navbar-padding-horizontal` | 1rem |

#### Sidebar
| Token | Property | Value |
|-------|----------|-------|
| Width | `--doc-sidebar-width` | 300px |
| Background | Transparent | Inherits page bg |
| Active Item | Primary background | Primary lightest |
| Hover Item | Subtle background | Gray-100 |

#### Footer
| Token | Property | Value |
|-------|----------|-------|
| Background | `--ifm-footer-background-color` | Dark gray |
| Text Color | `--ifm-footer-color` | Light text |
| Link Color | `--ifm-footer-link-color` | Primary light |
| Padding | `--ifm-footer-padding-vertical` | 2rem |

#### Buttons
| Token | Property | Value |
|-------|----------|-------|
| Border Radius | `--ifm-button-border-radius` | 0.4rem |
| Padding | `--ifm-button-padding-horizontal` | 1.5rem |
| Font Weight | `--ifm-button-font-weight` | 600 |
| Primary BG | Primary color | Main action |
| Primary Hover | Primary dark | Hover state |

#### Code Blocks
| Token | Property | Value |
|-------|----------|-------|
| Background | `--ifm-code-background` | Slightly darker than page |
| Border Radius | `--ifm-code-border-radius` | 0.4rem |
| Padding | `--ifm-code-padding-vertical` | 1rem |
| Font Size | `--ifm-code-font-size` | 90% |

#### Cards (Feature Cards)
| Token | Property | Value |
|-------|----------|-------|
| Background | Surface color | Elevated surface |
| Border Radius | `--ifm-card-border-radius` | 0.5rem |
| Shadow | Subtle shadow | Elevation effect |
| Padding | Internal padding | 1.5rem |

---

## State Transitions

### Interactive States

| Element | Default | Hover | Active | Focus |
|---------|---------|-------|--------|-------|
| Link | Primary | Primary Dark | Primary Darker | Outline |
| Button | Primary BG | Primary Dark BG | Primary Darker BG | Ring |
| Nav Item | Text | Primary + BG | Primary | Outline |
| Sidebar Item | Text | Gray BG | Primary BG | Outline |

### Theme Transition

| State | Duration | Easing |
|-------|----------|--------|
| Light → Dark | 200ms | ease-in-out |
| Color changes | 150ms | ease |
| Background | 200ms | ease-in-out |

---

## Responsive Breakpoints

| Name | Max Width | Description |
|------|-----------|-------------|
| Desktop | > 996px | Full layout |
| Tablet | 768px - 996px | Collapsed nav, adjusted spacing |
| Mobile | < 768px | Single column, mobile nav |
| Small Mobile | < 576px | Reduced padding/font sizes |

---

## File Structure

```text
frontend_book/src/
├── css/
│   └── custom.css          # All CSS variable overrides
├── pages/
│   └── index.module.css    # Homepage-specific styles
└── components/
    └── HomepageFeatures/
        └── styles.module.css  # Feature card styles
```
