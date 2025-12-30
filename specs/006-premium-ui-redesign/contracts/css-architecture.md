# CSS Architecture: Premium UI Redesign

**Feature**: 006-premium-ui-redesign
**Date**: 2025-12-30

## Overview

This document defines the CSS architecture for implementing the premium UI redesign. All styles are implemented through CSS custom properties (design tokens) and targeted CSS rules overriding Docusaurus/Infima defaults.

---

## 1. File Structure

```text
frontend_book/
├── static/
│   └── fonts/                    # Self-hosted premium fonts
│       ├── space-grotesk/
│       │   └── SpaceGrotesk-Variable.woff2
│       ├── inter/
│       │   └── Inter-Variable.woff2
│       └── jetbrains-mono/
│           └── JetBrainsMono-Variable.woff2
└── src/
    └── css/
        └── custom.css            # All premium styles (single file)
```

---

## 2. CSS Section Organization

The `custom.css` file is organized into clearly labeled sections:

```css
/**
 * Premium UI Redesign - Hackathon-Ready AI/Robotics Experience
 * Feature: 006-premium-ui-redesign
 *
 * TABLE OF CONTENTS
 * =================
 * 1. FONT IMPORTS & DEFINITIONS
 * 2. DESIGN TOKENS (CSS Custom Properties)
 *    2.1 Colors - Primary
 *    2.2 Colors - Accent
 *    2.3 Colors - Semantic
 *    2.4 Colors - Background
 *    2.5 Colors - Text
 *    2.6 Colors - Border
 *    2.7 Gradients
 *    2.8 Typography
 *    2.9 Spacing
 *    2.10 Effects (Shadows, Radius)
 *    2.11 Animation
 *    2.12 Z-Index
 * 3. INFIMA VARIABLE OVERRIDES
 * 4. BASE STYLES
 * 5. COMPONENT STYLES
 *    5.1 Hero Section
 *    5.2 Navigation
 *    5.3 Sidebar
 *    5.4 Footer
 *    5.5 Buttons
 *    5.6 Cards
 *    5.7 Code Blocks
 *    5.8 Tables
 *    5.9 Admonitions
 *    5.10 Links
 * 6. ANIMATIONS & KEYFRAMES
 * 7. RESPONSIVE STYLES
 *    7.1 Small Mobile (max-width: 576px)
 *    7.2 Mobile (max-width: 768px)
 *    7.3 Tablet (max-width: 1024px)
 *    7.4 Desktop (min-width: 1024px)
 *    7.5 Wide (min-width: 1440px)
 *    7.6 Ultra-wide (min-width: 2560px)
 * 8. ACCESSIBILITY
 *    8.1 Focus States
 *    8.2 Reduced Motion
 *    8.3 High Contrast
 * 9. UTILITIES
 *    9.1 Theme Transitions
 *    9.2 Sticky Footer
 */
```

---

## 3. Design Token Implementation

### 3.1 Light Mode Tokens (`:root`)

```css
:root {
  /* ========================================
   * 2.1 Colors - Primary (Cyan/Sky)
   * ======================================== */
  --premium-primary-50: #f0f9ff;
  --premium-primary-100: #e0f2fe;
  --premium-primary-200: #bae6fd;
  --premium-primary-300: #7dd3fc;
  --premium-primary-400: #38bdf8;
  --premium-primary-500: #0ea5e9;
  --premium-primary-600: #0284c7;
  --premium-primary-700: #0369a1;
  --premium-primary-800: #075985;
  --premium-primary-900: #0c4a6e;

  /* ========================================
   * 2.2 Colors - Accent (Violet)
   * ======================================== */
  --premium-accent-50: #f5f3ff;
  --premium-accent-100: #ede9fe;
  --premium-accent-200: #ddd6fe;
  --premium-accent-300: #c4b5fd;
  --premium-accent-400: #a78bfa;
  --premium-accent-500: #8b5cf6;
  --premium-accent-600: #7c3aed;
  --premium-accent-700: #6d28d9;
  --premium-accent-800: #5b21b6;
  --premium-accent-900: #4c1d95;

  /* ... (all other tokens from data-model.md) */
}
```

### 3.2 Dark Mode Tokens (`[data-theme='dark']`)

```css
[data-theme='dark'] {
  /* Shifted primary for dark backgrounds */
  --premium-primary-500: #38bdf8;
  --premium-primary-600: #0ea5e9;

  /* Dark backgrounds */
  --premium-bg-base: #0a0f1a;
  --premium-bg-surface: #111827;
  --premium-bg-elevated: #1e293b;

  /* Text for dark mode */
  --premium-text-primary: #f1f5f9;
  --premium-text-secondary: #e2e8f0;
  --premium-text-muted: #94a3b8;

  /* ... (all other dark mode overrides) */
}
```

---

## 4. Infima Variable Mapping

Map premium tokens to Infima variables for framework compatibility:

```css
:root {
  /* Map to Infima primary colors */
  --ifm-color-primary: var(--premium-primary-500);
  --ifm-color-primary-dark: var(--premium-primary-600);
  --ifm-color-primary-darker: var(--premium-primary-700);
  --ifm-color-primary-darkest: var(--premium-primary-800);
  --ifm-color-primary-light: var(--premium-primary-400);
  --ifm-color-primary-lighter: var(--premium-primary-300);
  --ifm-color-primary-lightest: var(--premium-primary-100);

  /* Background overrides */
  --ifm-background-color: var(--premium-bg-base);
  --ifm-background-surface-color: var(--premium-bg-surface);

  /* Text overrides */
  --ifm-font-color-base: var(--premium-text-primary);
  --ifm-font-color-secondary: var(--premium-text-secondary);

  /* Typography overrides */
  --ifm-font-family-base: var(--premium-font-body);
  --ifm-heading-font-family: var(--premium-font-display);
  --ifm-font-family-monospace: var(--premium-font-mono);
  --ifm-font-size-base: 112.5%; /* 18px base */
  --ifm-line-height-base: 1.65;

  /* Spacing overrides */
  --ifm-global-radius: var(--premium-radius-lg);
  --ifm-button-border-radius: var(--premium-radius-lg);

  /* Code overrides */
  --ifm-code-font-size: 90%;
}
```

---

## 5. Component Contracts

### 5.1 Hero Section

```css
/* Hero Container */
.hero {
  /* Gradient background with animation */
  background: var(--premium-gradient-hero-animated);
  background-size: 400% 400%;
  animation: premium-gradient-shift 15s ease infinite;

  /* Spacing */
  padding: var(--premium-space-32) var(--premium-space-6);
  min-height: 80vh;

  /* Layout */
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  text-align: center;

  /* Overlay for text readability */
  position: relative;
}

.hero::before {
  /* Grid pattern overlay */
  content: '';
  position: absolute;
  inset: 0;
  background-image: radial-gradient(
    circle at 1px 1px,
    rgba(255, 255, 255, 0.1) 1px,
    transparent 0
  );
  background-size: 40px 40px;
  pointer-events: none;
}

/* Hero Title */
.hero__title {
  font-family: var(--premium-font-display);
  font-size: var(--premium-text-6xl);
  font-weight: var(--premium-font-extrabold);
  letter-spacing: var(--premium-tracking-tighter);
  line-height: var(--premium-leading-tight);
  color: white;
  text-shadow: 0 2px 10px rgba(0, 0, 0, 0.2);

  /* Entrance animation */
  animation: premium-fade-up var(--premium-duration-hero) var(--premium-ease-out) forwards;
  animation-delay: 200ms;
  opacity: 0;
}

/* Hero Subtitle */
.hero__subtitle {
  font-family: var(--premium-font-body);
  font-size: var(--premium-text-xl);
  font-weight: var(--premium-font-normal);
  color: rgba(255, 255, 255, 0.9);
  max-width: 600px;
  margin: var(--premium-space-6) auto var(--premium-space-10);

  /* Entrance animation (staggered) */
  animation: premium-fade-up var(--premium-duration-hero) var(--premium-ease-out) forwards;
  animation-delay: 400ms;
  opacity: 0;
}

/* Hero Buttons Container */
.hero .buttons {
  display: flex;
  gap: var(--premium-space-4);

  /* Entrance animation (staggered) */
  animation: premium-fade-up var(--premium-duration-hero) var(--premium-ease-out) forwards;
  animation-delay: 600ms;
  opacity: 0;
}
```

### 5.2 Navigation

```css
/* Navbar */
.navbar {
  height: var(--premium-nav-height);
  background: var(--premium-nav-bg);
  backdrop-filter: blur(var(--premium-nav-blur));
  border-bottom: 1px solid var(--premium-border-subtle);
  box-shadow: none;
  transition: var(--premium-transition-theme);
}

/* Navbar Title */
.navbar__title {
  font-family: var(--premium-font-display);
  font-weight: var(--premium-font-bold);
  font-size: var(--premium-text-lg);
  letter-spacing: var(--premium-tracking-tight);
}

/* Navbar Links */
.navbar__link {
  font-weight: var(--premium-font-medium);
  color: var(--premium-nav-link-color);
  padding: var(--premium-space-2) var(--premium-space-4);
  border-radius: var(--premium-radius-md);
  transition: var(--premium-transition-colors);
  min-height: var(--premium-touch-min);
  display: flex;
  align-items: center;
}

.navbar__link:hover {
  color: var(--premium-nav-link-hover);
  background: var(--premium-bg-surface);
}

.navbar__link--active {
  color: var(--premium-nav-link-active);
  font-weight: var(--premium-font-semibold);
}
```

### 5.3 Buttons

```css
/* Base Button */
.button {
  font-family: var(--premium-font-body);
  font-weight: var(--premium-font-semibold);
  font-size: var(--premium-text-base);
  padding: var(--premium-space-3) var(--premium-space-6);
  min-height: var(--premium-touch-min);
  border-radius: var(--premium-radius-lg);
  transition: var(--premium-transition-all);
  cursor: pointer;
  display: inline-flex;
  align-items: center;
  justify-content: center;
  gap: var(--premium-space-2);
}

/* Primary Button */
.button--primary {
  background: var(--premium-gradient-button);
  color: var(--premium-text-inverse);
  border: none;
  box-shadow: var(--premium-shadow-primary);
}

.button--primary:hover {
  transform: translateY(-2px);
  box-shadow: var(--premium-shadow-lg), var(--premium-glow-primary);
}

.button--primary:active {
  transform: translateY(0);
}

/* Secondary Button */
.button--secondary {
  background: transparent;
  color: var(--premium-primary-600);
  border: 2px solid var(--premium-primary-500);
}

.button--secondary:hover {
  background: var(--premium-primary-50);
  border-color: var(--premium-primary-600);
}

[data-theme='dark'] .button--secondary {
  color: var(--premium-primary-400);
  border-color: var(--premium-primary-400);
}

[data-theme='dark'] .button--secondary:hover {
  background: rgba(56, 189, 248, 0.1);
}
```

### 5.4 Cards / Feature Section

```css
/* Feature Card Container */
[class*='features'] {
  padding: var(--premium-space-24) var(--premium-space-6);
  background: var(--premium-bg-surface);
}

/* Feature Card */
.feature {
  background: var(--premium-card-bg);
  border-radius: var(--premium-radius-xl);
  border: 1px solid var(--premium-card-border);
  box-shadow: var(--premium-card-shadow);
  padding: var(--premium-card-padding);
  transition: var(--premium-transition-all);
}

.feature:hover {
  transform: var(--premium-card-hover-transform);
  box-shadow: var(--premium-card-hover-shadow);
}

/* Feature Icon */
.featureSvg {
  width: 80px;
  height: 80px;
  margin-bottom: var(--premium-space-4);
  transition: transform var(--premium-duration-normal) var(--premium-ease-spring);
}

.feature:hover .featureSvg {
  transform: scale(1.05);
}
```

### 5.5 Code Blocks

```css
/* Code Block Container */
[class*='codeBlockContainer'] {
  background: var(--premium-code-bg);
  border: 1px solid var(--premium-code-border);
  border-radius: var(--premium-radius-lg);
  box-shadow: var(--premium-shadow-sm);
  overflow: hidden;
  margin-bottom: var(--premium-space-6);
}

/* Code Block Title */
[class*='codeBlockTitle'] {
  background: var(--premium-code-title-bg);
  font-family: var(--premium-font-mono);
  font-size: var(--premium-text-xs);
  font-weight: var(--premium-font-semibold);
  text-transform: uppercase;
  letter-spacing: var(--premium-tracking-wider);
  padding: var(--premium-space-2) var(--premium-space-4);
  border-bottom: 1px solid var(--premium-code-border);
}

/* Code Content */
pre {
  font-family: var(--premium-font-mono);
  font-size: 0.9em;
  line-height: var(--premium-leading-relaxed);
  padding: var(--premium-space-4);
  margin: 0;
}

/* Inline Code */
code:not([class*='codeBlock']) {
  font-family: var(--premium-font-mono);
  font-size: 0.9em;
  background: var(--premium-code-bg);
  padding: var(--premium-space-0-5) var(--premium-space-1-5);
  border-radius: var(--premium-radius-sm);
}
```

---

## 6. Animation Keyframes

```css
/* ========================================
 * 6. ANIMATIONS & KEYFRAMES
 * ======================================== */

/* Hero gradient animation */
@keyframes premium-gradient-shift {
  0%, 100% {
    background-position: 0% 50%;
  }
  50% {
    background-position: 100% 50%;
  }
}

/* Fade up entrance */
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

/* Fade right entrance */
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

/* Pulse glow for interactive elements */
@keyframes premium-pulse-glow {
  0%, 100% {
    box-shadow: var(--premium-shadow-md);
  }
  50% {
    box-shadow: var(--premium-shadow-lg), var(--premium-glow-primary);
  }
}
```

---

## 7. Responsive Breakpoints

```css
/* ========================================
 * 7. RESPONSIVE STYLES
 * ======================================== */

/* 7.1 Small Mobile (max-width: 576px) */
@media (max-width: 576px) {
  :root {
    --premium-text-6xl: 2.5rem;   /* Reduced hero */
    --premium-text-4xl: 2rem;
    --premium-text-3xl: 1.75rem;
    --premium-text-base: 1rem;    /* Standard body on small screens */
  }

  .hero {
    padding: var(--premium-space-16) var(--premium-space-4);
    min-height: 70vh;
  }
}

/* 7.2 Mobile (max-width: 768px) */
@media (max-width: 768px) {
  :root {
    --premium-text-6xl: 3rem;
    --premium-space-section: var(--premium-space-16);
  }

  .hero .buttons {
    flex-direction: column;
    width: 100%;
  }

  .button {
    width: 100%;
    justify-content: center;
  }
}

/* 7.3 Tablet (max-width: 1024px) */
@media (max-width: 1024px) {
  .navbar__link,
  .menu__link {
    min-height: var(--premium-touch-min);
  }
}

/* 7.4 Desktop (min-width: 1024px) */
@media (min-width: 1024px) {
  .hero {
    padding: var(--premium-space-32) var(--premium-space-8);
  }
}

/* 7.5 Wide (min-width: 1440px) */
@media (min-width: 1440px) {
  .container {
    max-width: 1400px;
  }
}

/* 7.6 Ultra-wide (min-width: 2560px) */
@media (min-width: 2560px) {
  :root {
    --premium-text-6xl: 6rem;
    --premium-text-4xl: 4rem;
  }
}
```

---

## 8. Accessibility

```css
/* ========================================
 * 8. ACCESSIBILITY
 * ======================================== */

/* 8.1 Focus States */
:focus-visible {
  outline: var(--premium-focus-ring);
  outline-offset: var(--premium-focus-ring-offset);
  border-radius: var(--premium-radius-sm);
}

.button:focus-visible,
.navbar__link:focus-visible,
.menu__link:focus-visible {
  outline: var(--premium-focus-ring);
  outline-offset: var(--premium-focus-ring-offset);
}

/* 8.2 Reduced Motion */
@media (prefers-reduced-motion: reduce) {
  *,
  *::before,
  *::after {
    animation-duration: 0.01ms !important;
    animation-iteration-count: 1 !important;
    transition-duration: 0.01ms !important;
    scroll-behavior: auto !important;
  }

  .hero {
    animation: none;
    background-size: 100% 100%;
  }
}

/* 8.3 High Contrast */
@media (prefers-contrast: high) {
  :root {
    --premium-border-default: var(--premium-text-primary);
  }

  .button--secondary {
    border-width: 3px;
  }
}
```

---

## 9. Theme Transitions

```css
/* ========================================
 * 9. UTILITIES
 * ======================================== */

/* 9.1 Theme Transitions */
html {
  transition: background-color var(--premium-duration-moderate) var(--premium-ease-in-out);
}

body,
.navbar,
.footer,
.menu__link,
code,
[class*='codeBlockContainer'] {
  transition: var(--premium-transition-theme);
}

/* 9.2 Sticky Footer */
#__docusaurus {
  min-height: 100vh;
  display: flex;
  flex-direction: column;
}

main {
  flex: 1;
}
```

---

## 10. Implementation Checklist

- [ ] Download and self-host fonts (Space Grotesk, Inter, JetBrains Mono)
- [ ] Implement design tokens in `:root` and `[data-theme='dark']`
- [ ] Map tokens to Infima variables
- [ ] Implement hero section with gradient animation
- [ ] Style navigation with 56px touch targets
- [ ] Style buttons with gradient and hover effects
- [ ] Style code blocks with premium aesthetics
- [ ] Implement responsive breakpoints
- [ ] Add accessibility features (focus, reduced motion)
- [ ] Test WCAG AAA contrast compliance
- [ ] Performance audit (60fps animations)
