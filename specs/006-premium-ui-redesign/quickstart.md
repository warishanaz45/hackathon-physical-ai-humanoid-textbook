# Quickstart: Premium UI Redesign

**Feature**: 006-premium-ui-redesign
**Date**: 2025-12-30
**Estimated Setup Time**: 15 minutes

## Prerequisites

- Node.js 18+ installed
- npm 9+ installed
- Frontend book running (verify with `npm start`)
- Browser with DevTools for visual testing

---

## Step 1: Verify Current State

```bash
# Navigate to frontend_book
cd frontend_book

# Verify the site runs
npm start

# Open http://localhost:3000 in browser
```

**Expected**: Site loads with current blue (#3b82f6) color scheme from 005-docusaurus-ui-upgrade.

---

## Step 2: Download Premium Fonts

Download the following fonts and place in `frontend_book/static/fonts/`:

### Space Grotesk (Headings)
Source: [Google Fonts](https://fonts.google.com/specimen/Space+Grotesk)
- Download Variable version (WOFF2)
- Save to `static/fonts/space-grotesk/SpaceGrotesk-Variable.woff2`

### Inter (Body)
Source: [Google Fonts](https://fonts.google.com/specimen/Inter) or [rsms.me/inter](https://rsms.me/inter/)
- Download Variable version (WOFF2)
- Save to `static/fonts/inter/Inter-Variable.woff2`

### JetBrains Mono (Code)
Source: [JetBrains](https://www.jetbrains.com/lp/mono/)
- Download Variable version (WOFF2)
- Save to `static/fonts/jetbrains-mono/JetBrainsMono-Variable.woff2`

**Alternative**: Use google-webfonts-helper for optimized downloads:
```bash
# Install helper (optional)
npm install -g google-webfonts-helper

# Or use web interface: https://gwfh.mranftl.com/fonts
```

---

## Step 3: Backup Current CSS

```bash
# Create backup of current custom.css
cp src/css/custom.css src/css/custom.css.005-backup
```

---

## Step 4: Implementation Order

Follow this order for minimal regression risk:

### Phase A: Foundation (Non-breaking)
1. Add font imports and definitions
2. Add design tokens (CSS custom properties)
3. Map tokens to Infima variables

### Phase B: Hero Section
4. Style hero gradient and animations
5. Style hero typography
6. Style hero buttons

### Phase C: Navigation
7. Style navbar with premium look
8. Style sidebar with touch targets
9. Style footer

### Phase D: Content Components
10. Style code blocks
11. Style buttons globally
12. Style cards/features

### Phase E: Polish
13. Add microinteractions
14. Implement responsive styles
15. Add accessibility features

---

## Step 5: Development Workflow

```bash
# Start dev server with hot reload
npm start

# Keep browser open at http://localhost:3000
# CSS changes will hot-reload automatically

# Test theme toggle frequently
# Keyboard shortcut: Ctrl+Shift+D (or use navbar toggle)
```

---

## Step 6: Testing Checkpoints

### After Phase A (Foundation):
- [ ] Site renders without errors
- [ ] Theme toggle works
- [ ] No visual regressions

### After Phase B (Hero):
- [ ] Hero has animated gradient background
- [ ] Hero text is visible in both themes
- [ ] Buttons are clickable

### After Phase C (Navigation):
- [ ] Navigation links have hover states
- [ ] Mobile menu works
- [ ] Touch targets are tappable

### After Phase D (Content):
- [ ] Code blocks have new styling
- [ ] Cards have hover effects
- [ ] All pages render correctly

### After Phase E (Polish):
- [ ] Animations are smooth (60fps)
- [ ] Reduced motion is respected
- [ ] WCAG AAA contrast verified

---

## Step 7: Contrast Testing

Use browser DevTools to verify contrast ratios:

1. Open DevTools (F12)
2. Go to Elements panel
3. Select text element
4. In Styles panel, hover over color value
5. Check "Contrast ratio" - should show AAA (7+:1)

**Key Combinations to Test**:
| Element | Light Mode | Dark Mode | Target |
|---------|------------|-----------|--------|
| Body text | #0f172a on #fefefe | #f1f5f9 on #0a0f1a | 7:1+ |
| Primary link | #0284c7 on #fefefe | #38bdf8 on #0a0f1a | 7:1+ |
| Muted text | #475569 on #fefefe | #94a3b8 on #0a0f1a | 7:1+ |

---

## Step 8: Mobile Testing

### Browser DevTools:
1. Open DevTools (F12)
2. Toggle device toolbar (Ctrl+Shift+M)
3. Test at these widths:
   - 320px (small mobile)
   - 375px (iPhone)
   - 768px (tablet)
   - 1024px (desktop)

### Real Device:
1. Find your local IP: `ipconfig` (Windows) or `ifconfig` (Mac/Linux)
2. On mobile, visit `http://YOUR-IP:3000`
3. Test touch interactions:
   - Navigation taps (56px targets)
   - Scroll smoothness (60fps)
   - Theme toggle

---

## Step 9: Performance Audit

```bash
# Build production version
npm run build

# Serve locally
npm run serve

# Open http://localhost:3000
```

### Lighthouse Check:
1. Open DevTools > Lighthouse
2. Select: Performance, Accessibility, Best Practices, SEO
3. Device: Mobile
4. Run audit

**Targets**:
- Performance: 90+
- Accessibility: 100
- Best Practices: 90+
- SEO: 90+

---

## Step 10: Final Validation Checklist

### Visual Impact (0-3 seconds)
- [ ] Hero creates immediate visual impact
- [ ] Site does NOT look like default Docusaurus
- [ ] Color palette is distinctive and premium

### Readability (3-30 seconds)
- [ ] Typography is premium (Space Grotesk headings visible)
- [ ] Code blocks are beautifully styled
- [ ] Dark mode is equally polished

### Interaction (30+ seconds)
- [ ] Button hover effects work
- [ ] Card hover effects work
- [ ] Theme transitions are smooth (<300ms)
- [ ] Navigation highlights current page

### Technical (Hackathon)
- [ ] Mobile Lighthouse > 90
- [ ] Touch targets >= 56px
- [ ] WCAG AAA contrast (7:1+)
- [ ] Reduced motion respected

---

## Troubleshooting

### Fonts not loading:
```css
/* Check font paths in custom.css */
@font-face {
  font-family: 'Space Grotesk';
  src: url('/fonts/space-grotesk/SpaceGrotesk-Variable.woff2') format('woff2');
  /* Path must start with /fonts/ for static folder */
}
```

### Theme transition flash:
```css
/* Ensure transition is on html, not just body */
html {
  transition: background-color 300ms ease-in-out;
}
```

### Animations janky:
```css
/* Use only GPU-accelerated properties */
.element {
  /* GOOD */
  transform: translateY(-2px);
  opacity: 0.8;

  /* BAD - causes repaints */
  top: -2px;
  margin-top: -2px;
}
```

### Touch targets too small:
```css
/* Ensure min-height on interactive elements */
.navbar__link,
.menu__link,
.button {
  min-height: 56px;
  padding: 16px; /* Adds to touch area */
}
```

---

## Resources

- **Design System**: `specs/006-premium-ui-redesign/data-model.md`
- **CSS Architecture**: `specs/006-premium-ui-redesign/contracts/css-architecture.md`
- **Research**: `specs/006-premium-ui-redesign/research.md`
- **Specification**: `specs/006-premium-ui-redesign/spec.md`

### External Tools:
- Contrast Checker: [WebAIM](https://webaim.org/resources/contrastchecker/)
- Font Helper: [google-webfonts-helper](https://gwfh.mranftl.com/fonts)
- CSS Variables: [Infima Docs](https://infima.dev/)
- Docusaurus Theming: [Official Guide](https://docusaurus.io/docs/styling-layout)
