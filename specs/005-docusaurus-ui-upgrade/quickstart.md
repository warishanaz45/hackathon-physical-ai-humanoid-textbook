# Quickstart: Docusaurus UI Upgrade

**Feature**: 005-docusaurus-ui-upgrade
**Date**: 2025-12-30

## Prerequisites

- Node.js 18+ installed
- npm 9+ installed
- Git installed
- Access to the repository

## Quick Setup

### 1. Clone and Navigate

```bash
# Clone the repository (if not already cloned)
git clone https://github.com/your-username/Hackathon-book-creation-2025.git

# Navigate to the book directory
cd Hackathon-book-creation-2025/frontend_book
```

### 2. Install Dependencies

```bash
npm install
```

**Expected Output**:
```
added XXX packages in XXs
```

### 3. Start Development Server

```bash
npm start
```

**Expected Output**:
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at http://localhost:3000/
```

The site opens automatically at `http://localhost:3000/Hackathon-book-creation-2025/`

### 4. Switch to Feature Branch

```bash
git checkout 005-docusaurus-ui-upgrade
```

---

## Development Workflow

### Editing Styles

1. **Open `src/css/custom.css`** in your editor
2. **Make changes** to CSS variables or add custom rules
3. **Save the file** - hot reload updates the browser automatically

### Testing Changes

| Action | Command |
|--------|---------|
| Start dev server | `npm start` |
| Build production | `npm run build` |
| Preview build | `npm run serve` |
| Clear cache | `npm run clear` |

### Checking Both Themes

- **Light mode**: Click sun icon in navbar (or system default)
- **Dark mode**: Click moon icon in navbar

### Testing Responsiveness

1. Open browser DevTools (F12)
2. Toggle device toolbar (Ctrl+Shift+M)
3. Test at these widths:
   - Desktop: 1200px+
   - Tablet: 996px (navbar collapses here)
   - Mobile: 768px
   - Small mobile: 375px

---

## File Locations

| File | Purpose |
|------|---------|
| `src/css/custom.css` | Global CSS variable overrides |
| `src/pages/index.module.css` | Homepage-specific styles |
| `src/components/HomepageFeatures/styles.module.css` | Feature card styles |
| `docusaurus.config.js` | Site configuration (colors, metadata) |

---

## Useful Commands

```bash
# Start development server with hot reload
npm start

# Build production bundle
npm run build

# Serve production build locally
npm run serve

# Clear Docusaurus cache (if styles aren't updating)
npm run clear && npm start

# Check for broken links
npm run build
```

---

## Troubleshooting

### Styles Not Updating

```bash
# Clear cache and restart
npm run clear
npm start
```

### Port Already in Use

```bash
# Start on different port
npm start -- --port 3001
```

### Build Errors

```bash
# Check for errors in terminal output
npm run build

# Common fixes:
# 1. Delete node_modules and reinstall
rm -rf node_modules
npm install

# 2. Clear cache
npm run clear
```

---

## Verification Checklist

After setup, verify these work:

- [ ] Site loads at `http://localhost:3000/Hackathon-book-creation-2025/`
- [ ] Can switch between light and dark mode
- [ ] Changes to `custom.css` hot reload in browser
- [ ] Site is responsive (test at 375px width)
- [ ] All documentation pages load without errors

---

## Next Steps

1. Review `specs/005-docusaurus-ui-upgrade/data-model.md` for design tokens
2. Review `specs/005-docusaurus-ui-upgrade/contracts/css-architecture.md` for CSS patterns
3. Begin implementing CSS changes in `src/css/custom.css`
4. Run `/sp.tasks` to generate implementation tasks
