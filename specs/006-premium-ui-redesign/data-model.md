# Design System: Premium UI Redesign

**Feature**: 006-premium-ui-redesign
**Date**: 2025-12-30
**Branch**: 006-premium-ui-redesign

## Overview

This design system defines the visual language for a hackathon-winning AI/Robotics documentation experience. All tokens are implemented as CSS custom properties for runtime theming.

---

## 1. Color System

### 1.1 Primary Colors (Cyan-Violet Gradient Anchor)

```css
/* Light Mode Primary Scale */
:root {
  --premium-primary-50: #f0f9ff;
  --premium-primary-100: #e0f2fe;
  --premium-primary-200: #bae6fd;
  --premium-primary-300: #7dd3fc;
  --premium-primary-400: #38bdf8;
  --premium-primary-500: #0ea5e9;   /* Base */
  --premium-primary-600: #0284c7;
  --premium-primary-700: #0369a1;
  --premium-primary-800: #075985;
  --premium-primary-900: #0c4a6e;
}

/* Dark Mode Primary (Shifted for visibility) */
[data-theme='dark'] {
  --premium-primary-500: #38bdf8;   /* Brighter base */
  --premium-primary-600: #0ea5e9;
}
```

### 1.2 Accent Colors (Violet)

```css
:root {
  --premium-accent-50: #f5f3ff;
  --premium-accent-100: #ede9fe;
  --premium-accent-200: #ddd6fe;
  --premium-accent-300: #c4b5fd;
  --premium-accent-400: #a78bfa;
  --premium-accent-500: #8b5cf6;    /* Base */
  --premium-accent-600: #7c3aed;
  --premium-accent-700: #6d28d9;
  --premium-accent-800: #5b21b6;
  --premium-accent-900: #4c1d95;
}
```

### 1.3 Semantic Colors

```css
:root {
  /* Success */
  --premium-success-500: #10b981;
  --premium-success-600: #059669;

  /* Warning */
  --premium-warning-500: #f59e0b;
  --premium-warning-600: #d97706;

  /* Error */
  --premium-error-500: #ef4444;
  --premium-error-600: #dc2626;

  /* Info */
  --premium-info-500: var(--premium-primary-500);
  --premium-info-600: var(--premium-primary-600);
}
```

### 1.4 Background Colors

```css
/* Light Mode Backgrounds */
:root {
  --premium-bg-base: #fefefe;
  --premium-bg-surface: #f8fafc;
  --premium-bg-elevated: #ffffff;
  --premium-bg-overlay: rgba(15, 23, 42, 0.5);
}

/* Dark Mode Backgrounds */
[data-theme='dark'] {
  --premium-bg-base: #0a0f1a;
  --premium-bg-surface: #111827;
  --premium-bg-elevated: #1e293b;
  --premium-bg-overlay: rgba(0, 0, 0, 0.7);
}
```

### 1.5 Text Colors

```css
/* Light Mode Text */
:root {
  --premium-text-primary: #0f172a;
  --premium-text-secondary: #334155;
  --premium-text-muted: #475569;
  --premium-text-subtle: #64748b;
  --premium-text-inverse: #f8fafc;
}

/* Dark Mode Text */
[data-theme='dark'] {
  --premium-text-primary: #f1f5f9;
  --premium-text-secondary: #e2e8f0;
  --premium-text-muted: #94a3b8;
  --premium-text-subtle: #64748b;
  --premium-text-inverse: #0f172a;
}
```

### 1.6 Border Colors

```css
:root {
  --premium-border-subtle: rgba(15, 23, 42, 0.08);
  --premium-border-default: rgba(15, 23, 42, 0.12);
  --premium-border-strong: rgba(15, 23, 42, 0.2);
}

[data-theme='dark'] {
  --premium-border-subtle: rgba(248, 250, 252, 0.08);
  --premium-border-default: rgba(248, 250, 252, 0.12);
  --premium-border-strong: rgba(248, 250, 252, 0.2);
}
```

### 1.7 Gradient Definitions

```css
:root {
  /* Hero gradient */
  --premium-gradient-hero: linear-gradient(
    135deg,
    var(--premium-primary-500) 0%,
    var(--premium-accent-500) 50%,
    var(--premium-primary-600) 100%
  );

  /* Animated hero gradient */
  --premium-gradient-hero-animated: linear-gradient(
    -45deg,
    var(--premium-primary-500),
    var(--premium-accent-400),
    var(--premium-primary-600),
    var(--premium-accent-500)
  );

  /* Text gradient */
  --premium-gradient-text: linear-gradient(
    90deg,
    var(--premium-primary-500) 0%,
    var(--premium-accent-500) 100%
  );

  /* Button gradient */
  --premium-gradient-button: linear-gradient(
    135deg,
    var(--premium-primary-500) 0%,
    var(--premium-primary-600) 100%
  );

  /* Card glow */
  --premium-gradient-glow: radial-gradient(
    circle at center,
    var(--premium-primary-500) 0%,
    transparent 70%
  );
}
```

---

## 2. Typography System

### 2.1 Font Families

```css
:root {
  /* Display/Headings - Space Grotesk */
  --premium-font-display: 'Space Grotesk', system-ui, -apple-system, sans-serif;

  /* Body/UI - Inter */
  --premium-font-body: 'Inter', system-ui, -apple-system, sans-serif;

  /* Code - JetBrains Mono */
  --premium-font-mono: 'JetBrains Mono', 'Fira Code', 'Consolas', monospace;
}
```

### 2.2 Font Sizes (Fluid Scale)

```css
:root {
  /* Base: 18px for body text */
  --premium-text-xs: 0.75rem;     /* 12px */
  --premium-text-sm: 0.875rem;    /* 14px */
  --premium-text-base: 1.125rem;  /* 18px - Premium larger body */
  --premium-text-lg: 1.25rem;     /* 20px */
  --premium-text-xl: 1.5rem;      /* 24px */
  --premium-text-2xl: 1.875rem;   /* 30px */
  --premium-text-3xl: 2.25rem;    /* 36px */
  --premium-text-4xl: 3rem;       /* 48px */
  --premium-text-5xl: 4rem;       /* 64px */
  --premium-text-6xl: 5rem;       /* 80px - Hero */
}
```

### 2.3 Font Weights

```css
:root {
  --premium-font-normal: 400;
  --premium-font-medium: 500;
  --premium-font-semibold: 600;
  --premium-font-bold: 700;
  --premium-font-extrabold: 800;
}
```

### 2.4 Line Heights

```css
:root {
  --premium-leading-none: 1;
  --premium-leading-tight: 1.2;
  --premium-leading-snug: 1.375;
  --premium-leading-normal: 1.5;
  --premium-leading-relaxed: 1.65;  /* Body text */
  --premium-leading-loose: 1.8;
}
```

### 2.5 Letter Spacing

```css
:root {
  --premium-tracking-tighter: -0.04em;
  --premium-tracking-tight: -0.025em;
  --premium-tracking-normal: 0;
  --premium-tracking-wide: 0.025em;
  --premium-tracking-wider: 0.05em;
  --premium-tracking-widest: 0.1em;
}
```

### 2.6 Typography Presets

```css
/* Hero Title */
.premium-hero-title {
  font-family: var(--premium-font-display);
  font-size: var(--premium-text-6xl);
  font-weight: var(--premium-font-extrabold);
  line-height: var(--premium-leading-tight);
  letter-spacing: var(--premium-tracking-tighter);
}

/* H1 */
.premium-h1 {
  font-family: var(--premium-font-display);
  font-size: var(--premium-text-4xl);
  font-weight: var(--premium-font-bold);
  line-height: var(--premium-leading-tight);
  letter-spacing: var(--premium-tracking-tight);
}

/* H2 */
.premium-h2 {
  font-family: var(--premium-font-display);
  font-size: var(--premium-text-3xl);
  font-weight: var(--premium-font-semibold);
  line-height: var(--premium-leading-snug);
  letter-spacing: var(--premium-tracking-tight);
}

/* H3 */
.premium-h3 {
  font-family: var(--premium-font-display);
  font-size: var(--premium-text-2xl);
  font-weight: var(--premium-font-semibold);
  line-height: var(--premium-leading-snug);
}

/* Body */
.premium-body {
  font-family: var(--premium-font-body);
  font-size: var(--premium-text-base);
  font-weight: var(--premium-font-normal);
  line-height: var(--premium-leading-relaxed);
}

/* Code */
.premium-code {
  font-family: var(--premium-font-mono);
  font-size: 0.9em;
  font-weight: var(--premium-font-normal);
  line-height: var(--premium-leading-normal);
}
```

---

## 3. Spacing System

### 3.1 Spacing Scale

```css
:root {
  --premium-space-0: 0;
  --premium-space-px: 1px;
  --premium-space-0-5: 0.125rem;   /* 2px */
  --premium-space-1: 0.25rem;      /* 4px */
  --premium-space-1-5: 0.375rem;   /* 6px */
  --premium-space-2: 0.5rem;       /* 8px */
  --premium-space-2-5: 0.625rem;   /* 10px */
  --premium-space-3: 0.75rem;      /* 12px */
  --premium-space-3-5: 0.875rem;   /* 14px */
  --premium-space-4: 1rem;         /* 16px */
  --premium-space-5: 1.25rem;      /* 20px */
  --premium-space-6: 1.5rem;       /* 24px */
  --premium-space-7: 1.75rem;      /* 28px */
  --premium-space-8: 2rem;         /* 32px */
  --premium-space-9: 2.25rem;      /* 36px */
  --premium-space-10: 2.5rem;      /* 40px */
  --premium-space-12: 3rem;        /* 48px */
  --premium-space-14: 3.5rem;      /* 56px - Touch target */
  --premium-space-16: 4rem;        /* 64px */
  --premium-space-20: 5rem;        /* 80px */
  --premium-space-24: 6rem;        /* 96px */
  --premium-space-32: 8rem;        /* 128px */
}
```

### 3.2 Component Spacing

```css
:root {
  /* Touch targets (56px minimum) */
  --premium-touch-target: var(--premium-space-14);

  /* Container padding */
  --premium-container-padding-x: var(--premium-space-6);
  --premium-container-padding-y: var(--premium-space-8);

  /* Section spacing */
  --premium-section-gap: var(--premium-space-24);

  /* Card padding */
  --premium-card-padding: var(--premium-space-6);

  /* Button padding */
  --premium-button-padding-x: var(--premium-space-6);
  --premium-button-padding-y: var(--premium-space-3);
}
```

---

## 4. Effects System

### 4.1 Border Radius

```css
:root {
  --premium-radius-none: 0;
  --premium-radius-sm: 0.25rem;    /* 4px */
  --premium-radius-md: 0.5rem;     /* 8px */
  --premium-radius-lg: 0.75rem;    /* 12px */
  --premium-radius-xl: 1rem;       /* 16px */
  --premium-radius-2xl: 1.5rem;    /* 24px */
  --premium-radius-full: 9999px;
}
```

### 4.2 Shadows

```css
/* Light Mode Shadows */
:root {
  --premium-shadow-xs: 0 1px 2px rgba(15, 23, 42, 0.04);
  --premium-shadow-sm: 0 2px 4px rgba(15, 23, 42, 0.06);
  --premium-shadow-md: 0 4px 8px rgba(15, 23, 42, 0.08),
                       0 2px 4px rgba(15, 23, 42, 0.04);
  --premium-shadow-lg: 0 8px 16px rgba(15, 23, 42, 0.1),
                       0 4px 8px rgba(15, 23, 42, 0.06);
  --premium-shadow-xl: 0 16px 32px rgba(15, 23, 42, 0.12),
                       0 8px 16px rgba(15, 23, 42, 0.08);
  --premium-shadow-2xl: 0 24px 48px rgba(15, 23, 42, 0.16),
                        0 12px 24px rgba(15, 23, 42, 0.1);

  /* Colored shadows for buttons/accents */
  --premium-shadow-primary: 0 4px 16px rgba(14, 165, 233, 0.25);
  --premium-shadow-accent: 0 4px 16px rgba(139, 92, 246, 0.25);

  /* Glow effects */
  --premium-glow-primary: 0 0 24px rgba(14, 165, 233, 0.3);
  --premium-glow-accent: 0 0 24px rgba(139, 92, 246, 0.3);
}

/* Dark Mode Shadows */
[data-theme='dark'] {
  --premium-shadow-xs: 0 1px 2px rgba(0, 0, 0, 0.2);
  --premium-shadow-sm: 0 2px 4px rgba(0, 0, 0, 0.25);
  --premium-shadow-md: 0 4px 8px rgba(0, 0, 0, 0.3),
                       0 2px 4px rgba(0, 0, 0, 0.2);
  --premium-shadow-lg: 0 8px 16px rgba(0, 0, 0, 0.35),
                       0 4px 8px rgba(0, 0, 0, 0.25);
  --premium-shadow-xl: 0 16px 32px rgba(0, 0, 0, 0.4),
                       0 8px 16px rgba(0, 0, 0, 0.3);
  --premium-shadow-2xl: 0 24px 48px rgba(0, 0, 0, 0.5),
                        0 12px 24px rgba(0, 0, 0, 0.35);

  --premium-shadow-primary: 0 4px 16px rgba(56, 189, 248, 0.2);
  --premium-shadow-accent: 0 4px 16px rgba(167, 139, 250, 0.2);

  --premium-glow-primary: 0 0 32px rgba(56, 189, 248, 0.25);
  --premium-glow-accent: 0 0 32px rgba(167, 139, 250, 0.25);
}
```

---

## 5. Animation System

### 5.1 Durations

```css
:root {
  --premium-duration-instant: 50ms;
  --premium-duration-fast: 150ms;
  --premium-duration-normal: 200ms;
  --premium-duration-moderate: 300ms;
  --premium-duration-slow: 400ms;
  --premium-duration-slower: 600ms;
  --premium-duration-hero: 800ms;
}
```

### 5.2 Easing Functions

```css
:root {
  /* Standard easings */
  --premium-ease-linear: linear;
  --premium-ease-in: cubic-bezier(0.4, 0, 1, 1);
  --premium-ease-out: cubic-bezier(0, 0, 0.2, 1);
  --premium-ease-in-out: cubic-bezier(0.4, 0, 0.2, 1);

  /* Premium easings */
  --premium-ease-bounce: cubic-bezier(0.34, 1.56, 0.64, 1);
  --premium-ease-spring: cubic-bezier(0.175, 0.885, 0.32, 1.275);
  --premium-ease-smooth: cubic-bezier(0.25, 0.1, 0.25, 1);
}
```

### 5.3 Transition Presets

```css
:root {
  /* Common transitions */
  --premium-transition-colors: color var(--premium-duration-fast) var(--premium-ease-out),
                               background-color var(--premium-duration-fast) var(--premium-ease-out),
                               border-color var(--premium-duration-fast) var(--premium-ease-out);

  --premium-transition-transform: transform var(--premium-duration-normal) var(--premium-ease-out);

  --premium-transition-shadow: box-shadow var(--premium-duration-normal) var(--premium-ease-out);

  --premium-transition-all: all var(--premium-duration-normal) var(--premium-ease-out);

  --premium-transition-theme: background-color var(--premium-duration-moderate) var(--premium-ease-in-out),
                              color var(--premium-duration-moderate) var(--premium-ease-in-out);
}
```

### 5.4 Animation Keyframes

```css
/* Hero gradient shift */
@keyframes premium-gradient-shift {
  0%, 100% { background-position: 0% 50%; }
  50% { background-position: 100% 50%; }
}

/* Fade in from bottom */
@keyframes premium-fade-up {
  from {
    opacity: 0;
    transform: translateY(20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

/* Fade in from left */
@keyframes premium-fade-right {
  from {
    opacity: 0;
    transform: translateX(-20px);
  }
  to {
    opacity: 1;
    transform: translateX(0);
  }
}

/* Scale in */
@keyframes premium-scale-in {
  from {
    opacity: 0;
    transform: scale(0.95);
  }
  to {
    opacity: 1;
    transform: scale(1);
  }
}

/* Pulse glow */
@keyframes premium-pulse-glow {
  0%, 100% {
    box-shadow: var(--premium-shadow-md);
  }
  50% {
    box-shadow: var(--premium-shadow-lg), var(--premium-glow-primary);
  }
}

/* Shimmer effect */
@keyframes premium-shimmer {
  0% {
    background-position: -200% 0;
  }
  100% {
    background-position: 200% 0;
  }
}
```

---

## 6. Responsive Breakpoints

### 6.1 Breakpoint Values

```css
:root {
  --premium-breakpoint-xs: 320px;
  --premium-breakpoint-sm: 576px;
  --premium-breakpoint-md: 768px;
  --premium-breakpoint-lg: 1024px;
  --premium-breakpoint-xl: 1280px;
  --premium-breakpoint-2xl: 1440px;
  --premium-breakpoint-3xl: 2560px;
}
```

### 6.2 Media Query Usage

```css
/* Mobile-first approach */
/* Base styles: Mobile (320px+) */

/* Small tablet and up */
@media (min-width: 576px) { }

/* Tablet and up */
@media (min-width: 768px) { }

/* Desktop and up */
@media (min-width: 1024px) { }

/* Large desktop and up */
@media (min-width: 1280px) { }

/* Wide screens */
@media (min-width: 1440px) { }

/* Ultra-wide */
@media (min-width: 2560px) { }

/* Reduced motion */
@media (prefers-reduced-motion: reduce) { }

/* High contrast */
@media (prefers-contrast: high) { }
```

---

## 7. Z-Index Scale

```css
:root {
  --premium-z-base: 0;
  --premium-z-dropdown: 100;
  --premium-z-sticky: 200;
  --premium-z-fixed: 300;
  --premium-z-backdrop: 400;
  --premium-z-modal: 500;
  --premium-z-popover: 600;
  --premium-z-tooltip: 700;
  --premium-z-toast: 800;
  --premium-z-max: 9999;
}
```

---

## 8. Component Tokens

### 8.1 Button Tokens

```css
:root {
  /* Primary button */
  --premium-button-primary-bg: var(--premium-gradient-button);
  --premium-button-primary-text: var(--premium-text-inverse);
  --premium-button-primary-shadow: var(--premium-shadow-primary);

  /* Secondary button */
  --premium-button-secondary-bg: transparent;
  --premium-button-secondary-border: var(--premium-primary-500);
  --premium-button-secondary-text: var(--premium-primary-600);

  /* Ghost button */
  --premium-button-ghost-bg: transparent;
  --premium-button-ghost-text: var(--premium-text-primary);
  --premium-button-ghost-hover-bg: var(--premium-bg-surface);
}
```

### 8.2 Card Tokens

```css
:root {
  --premium-card-bg: var(--premium-bg-elevated);
  --premium-card-border: var(--premium-border-subtle);
  --premium-card-shadow: var(--premium-shadow-md);
  --premium-card-hover-shadow: var(--premium-shadow-lg);
  --premium-card-hover-transform: translateY(-4px);
}
```

### 8.3 Navigation Tokens

```css
:root {
  --premium-nav-height: 64px;
  --premium-nav-bg: var(--premium-bg-base);
  --premium-nav-blur: 12px;
  --premium-nav-border: var(--premium-border-subtle);

  --premium-nav-link-color: var(--premium-text-secondary);
  --premium-nav-link-hover: var(--premium-primary-500);
  --premium-nav-link-active: var(--premium-primary-600);
}

[data-theme='dark'] {
  --premium-nav-bg: rgba(10, 15, 26, 0.8);
}
```

### 8.4 Code Block Tokens

```css
:root {
  --premium-code-bg: #f1f5f9;
  --premium-code-border: var(--premium-border-default);
  --premium-code-text: var(--premium-text-primary);
  --premium-code-title-bg: #e2e8f0;
}

[data-theme='dark'] {
  --premium-code-bg: #1e293b;
  --premium-code-border: rgba(248, 250, 252, 0.1);
  --premium-code-text: var(--premium-text-primary);
  --premium-code-title-bg: #334155;
}
```

---

## 9. Accessibility Tokens

### 9.1 Focus States

```css
:root {
  --premium-focus-ring-width: 2px;
  --premium-focus-ring-offset: 2px;
  --premium-focus-ring-color: var(--premium-primary-500);
  --premium-focus-ring: var(--premium-focus-ring-width) solid var(--premium-focus-ring-color);
}
```

### 9.2 Touch Target Sizes

```css
:root {
  /* Minimum 56px for premium feel */
  --premium-touch-min: 56px;

  /* Interactive element heights */
  --premium-button-height: 56px;
  --premium-input-height: 56px;
  --premium-nav-item-height: 56px;
}
```

---

## 10. Token Usage Guidelines

### 10.1 Color Usage

| Use Case | Token |
|----------|-------|
| Primary actions | `--premium-primary-500` |
| Hover states | `--premium-primary-600` |
| Accent/highlights | `--premium-accent-500` |
| Body text | `--premium-text-primary` |
| Secondary text | `--premium-text-secondary` |
| Disabled text | `--premium-text-subtle` |
| Backgrounds | `--premium-bg-*` scale |

### 10.2 Typography Usage

| Element | Font Family | Size | Weight |
|---------|-------------|------|--------|
| Hero title | Display | 5xl-6xl | Extrabold |
| Page title (H1) | Display | 4xl | Bold |
| Section (H2) | Display | 3xl | Semibold |
| Subsection (H3) | Display | 2xl | Semibold |
| Body | Body | base | Normal |
| Small/Labels | Body | sm | Medium |
| Code | Mono | 0.9em | Normal |

### 10.3 Spacing Usage

| Context | Token |
|---------|-------|
| Touch targets | `--premium-space-14` (56px) |
| Section gaps | `--premium-space-24` (96px) |
| Card padding | `--premium-space-6` (24px) |
| Element gaps | `--premium-space-4` (16px) |
| Tight spacing | `--premium-space-2` (8px) |
