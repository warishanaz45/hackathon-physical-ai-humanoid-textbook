# Quickstart: Module 1 - The Robotic Nervous System (ROS 2)

**Feature**: `001-ros2-module-1`
**Date**: 2025-12-29
**Purpose**: Local development setup and commands

---

## Prerequisites

- **Node.js**: 18.0 or higher
- **npm**: 9.0 or higher (included with Node.js)
- **Git**: For version control

Verify installation:
```bash
node --version   # Expected: v18.x.x or higher
npm --version    # Expected: 9.x.x or higher
git --version    # Expected: git version 2.x.x
```

---

## Initial Setup

### 1. Clone the Repository

```bash
git clone https://github.com/<username>/Hackathon-book-creation-2025.git
cd Hackathon-book-creation-2025
```

### 2. Initialize Docusaurus (First Time Only)

```bash
npx create-docusaurus@latest book classic
```

**Expected Output**:
```
[SUCCESS] Created book.
[INFO] Inside that directory, you can run several commands:
  npm start
  npm run build
  npm run serve
  npm run deploy
```

### 3. Install Dependencies

```bash
cd book
npm install
```

**Expected Output**:
```
added XXX packages in Xs
```

---

## Development Commands

### Start Development Server

```bash
cd book
npm start
```

**Expected Output**:
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

The browser will open automatically. Changes to Markdown files trigger hot reload.

### Build for Production

```bash
npm run build
```

**Expected Output**:
```
[SUCCESS] Generated static files in "build".
```

### Preview Production Build

```bash
npm run serve
```

**Expected Output**:
```
[INFO] Serving "build" directory at: http://localhost:3000/
```

### Clear Cache (If Issues)

```bash
npm run clear
```

---

## Project Structure

After setup, the structure should be:

```
Hackathon-book-creation-2025/
├── book/                          # Docusaurus project
│   ├── docs/                      # Documentation content
│   │   └── module-1-ros2/         # Module 1 chapters
│   │       ├── _category_.json
│   │       ├── 01-introduction.md
│   │       ├── 02-communication.md
│   │       └── 03-urdf.md
│   ├── src/                       # Custom React components
│   ├── static/                    # Static assets
│   ├── docusaurus.config.js       # Site configuration
│   ├── sidebars.js                # Sidebar configuration
│   └── package.json               # Node dependencies
├── specs/                         # Specifications
│   └── 001-ros2-module-1/
├── history/                       # Prompt history
└── .specify/                      # SpecKit Plus templates
```

---

## Deployment

### Manual Deployment

```bash
cd book
GIT_USER=<username> npm run deploy
```

### Automated Deployment (GitHub Actions)

Push to `main` branch triggers automatic deployment via `.github/workflows/deploy.yml`.

**GitHub Pages Settings**:
1. Go to repository Settings > Pages
2. Source: "Deploy from a branch"
3. Branch: `gh-pages` / `/ (root)`

---

## Troubleshooting

### Port Already in Use

```bash
# Use a different port
npm start -- --port 3001
```

### Build Fails with Link Errors

```bash
# Check for broken links
npm run build 2>&1 | grep -i "broken"
```

### Sidebar Not Updating

```bash
# Clear cache and restart
npm run clear
npm start
```

### Node Version Issues

```bash
# Use nvm to switch Node versions
nvm use 18
```

---

## Verification Checklist

After setup, verify:

- [ ] `npm start` launches site at localhost:3000
- [ ] Module 1 appears in sidebar
- [ ] All 3 chapters are visible
- [ ] Code blocks have syntax highlighting
- [ ] `npm run build` completes without errors
