# Phase 5: Docusaurus Build & Deploy — Implementation Guide

**Feature**: 004-vla-pipeline | **Status**: IN PROGRESS | **Date**: 2025-12-09

**Branch**: `004-vla-pipeline` | **Objective**: Deploy textbook website to GitHub Pages

---

## Quick Start (5 Minutes)

### Prerequisites Check

```bash
# Verify Node.js 18+ is installed
node --version    # Should be v18.0.0+
npm --version     # Should be 9.0.0+

# Verify Docusaurus site structure
cd my-website
ls -la
```

Expected structure:
```
my-website/
├── docusaurus.config.ts      ✅ Already configured
├── sidebars.ts               ✅ Already configured
├── docs/                     ✅ All chapters present
│   ├── module1/
│   ├── module2/
│   ├── module3/
│   └── module4/
├── src/                      ✅ Theme + styling
└── package.json              ✅ Dependencies listed
```

### 3-Step Deployment

```bash
# Step 1: Install dependencies (if not already done)
cd my-website
npm install

# Step 2: Build the site
npm run build
# Output: build/ directory created (~50 MB static files)

# Step 3: Deploy to GitHub Pages
npm run deploy
# Automatically pushes to gh-pages branch
```

**Expected Result**: Website live at `https://Shahb.github.io/hackthon_humanoid_book/`

---

## Detailed Setup & Configuration

### Step 1: Install Dependencies

```bash
cd my-website

# Install all npm packages
npm install

# Verify installation
npm list docusaurus
# Should show: @docusaurus/core@3.x.x
```

**What gets installed**:
- `@docusaurus/core` — Docusaurus framework
- `@docusaurus/preset-classic` — Default theme & plugins
- `@docusaurus/theme-mermaid` — Diagram support
- `react` & `react-dom` — React framework
- `prism-react-renderer` — Code syntax highlighting
- Build tools (webpack, babel, etc.)

### Step 2: Verify Configuration

#### Check docusaurus.config.ts

The configuration file defines:

```typescript
const config: Config = {
  title: 'ROS 2 Fundamentals for Humanoid Robotics',
  url: 'https://Shahb.github.io',
  baseUrl: '/hackthon_humanoid_book/',
  organizationName: 'Shahb',
  projectName: 'hackthon_humanoid_book',
  // ... rest of config
}
```

**Key sections**:

1. **Site Metadata**:
   - `title`: Browser tab title
   - `tagline`: Subtitle
   - `favicon`: Icon (public/img/favicon.ico)

2. **GitHub Pages Config**:
   - `url`: Your GitHub user domain
   - `baseUrl`: Repository name (with trailing slash!)
   - `organizationName`: GitHub username
   - `projectName`: Repository name

3. **Docs Configuration**:
   - `routeBasePath: '/'` — Docs at site root
   - `sidebarPath: './sidebars.ts'` — Navigation structure
   - `editUrl` — Link to edit on GitHub

4. **Theme Config**:
   - `navbar` — Top navigation
   - `footer` — Bottom links
   - `colorMode` — Dark/light mode
   - Mermaid diagrams enabled

#### Verify sidebars.ts

The sidebar config defines navigation:

```typescript
const sidebars: SidebarsConfig = {
  // Each module has its own sidebar
  module1Sidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module1/chapter1-ros2-core',
        'module1/chapter2-agent-bridge',
        'module1/chapter3-urdf-model',
      ],
    },
  ],
  // module2Sidebar, module3Sidebar, module4Sidebar...
}
```

**What's verified**:
- ✅ All 15 chapters referenced
- ✅ All module intro.md files present
- ✅ Sidebar paths match file structure

### Step 3: Local Build & Test

```bash
# Start development server (with hot reload)
npm run start
# Opens http://localhost:3000 in browser

# Check all chapters load
# Navigate through navbar > each module
# Verify diagrams render (Mermaid, ASCII)
# Test dark/light mode toggle
# Check mobile responsiveness
```

**What to verify locally**:
- [ ] Home page loads
- [ ] Module 1–4 sidebars show
- [ ] All chapters accessible
- [ ] Mermaid diagrams render correctly
- [ ] Links work (internal & external)
- [ ] Code syntax highlighting works
- [ ] Mobile navigation works
- [ ] Dark mode toggle works

### Step 4: Production Build

```bash
# Create optimized production build
npm run build

# Output:
# build/ directory created
# ~50 MB static files
# Ready for deployment

# Verify build succeeded
ls -la build/
# Should have index.html and many .js/.css files
```

**Build process**:
1. Parses all markdown files
2. Generates HTML + JavaScript
3. Minifies CSS & JavaScript
4. Creates search index
5. Generates sitemap

**Expected files**:
```
build/
├── index.html          (Entry point)
├── docs/              (Compiled documentation)
├── assets/            (Images, fonts)
├── js/                (Bundled JavaScript)
└── css/               (Compiled stylesheets)
```

### Step 5: GitHub Pages Deployment

#### Option A: Using npm deploy (Recommended)

```bash
# Requires git credentials
npm run deploy

# What happens:
# 1. Runs npm run build
# 2. Commits build/ to gh-pages branch
# 3. Pushes gh-pages to GitHub
# 4. GitHub automatically publishes
```

#### Option B: Manual GitHub Pages Setup

If `npm run deploy` doesn't work:

```bash
# 1. Ensure gh-pages package is installed
npm install --save-dev gh-pages

# 2. Verify package.json has deploy script
# Should have: "deploy": "docusaurus deploy"

# 3. Try deploy again
npm run deploy
```

#### Option C: Using GitHub Actions (Continuous Deployment)

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches:
      - main
      - 004-vla-pipeline

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
        with:
          fetch-depth: 0

      - uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        working-directory: ./my-website
        run: npm ci

      - name: Build website
        working-directory: ./my-website
        run: npm run build

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./my-website/build
          cname: 'humanoid-book.example.com'  # Optional: custom domain
```

### Step 6: Verify Live Deployment

After deployment (takes ~1 minute):

```bash
# Check GitHub Pages settings
# https://github.com/Shahb/hackthon_humanoid_book/settings/pages

# Verify site is live
# https://Shahb.github.io/hackthon_humanoid_book/

# Check for SSL certificate (should be automatic)
# Lock icon should appear in address bar

# Test all pages load
# Check for 404 errors in browser console
```

**Common issues & fixes**:

| Issue | Cause | Fix |
|-------|-------|-----|
| 404 errors on pages | baseUrl not set correctly | Verify `/hackthon_humanoid_book/` in docusaurus.config.ts |
| Styles not loading | CSS baseUrl wrong | npm run build && npm run deploy |
| Images broken | Relative paths incorrect | Use `/docs/image.png` format |
| Deployment fails | Git credentials missing | Configure git: `git config user.email` |
| Site shows old version | Cache issue | Hard refresh (Ctrl+Shift+R) or clear browser cache |

---

## Configuration Checklist

### Required Changes (Before Deployment)

- [ ] Verify `url` in docusaurus.config.ts matches your GitHub domain
- [ ] Verify `baseUrl` ends with repository name + slash
- [ ] Verify `organizationName` matches GitHub username
- [ ] Verify `projectName` matches repository name
- [ ] Verify `editUrl` points to correct GitHub repo
- [ ] All chapter files exist in `docs/` directory
- [ ] sidebars.ts references exist (no broken sidebar links)

### Recommended Enhancements

- [ ] Update navbar links (currently only shows Module 1)
- [ ] Add footer with copyright/license
- [ ] Update logo (public/img/logo.svg)
- [ ] Add social media links
- [ ] Configure analytics (Google Analytics / Plausible)
- [ ] Add custom domain (optional)

### Navigation Updates Needed

The navbar currently only shows Module 1. Update navbar in docusaurus.config.ts:

```typescript
navbar: {
  items: [
    // Current (Module 1 only):
    {
      type: 'docSidebar',
      sidebarId: 'module1Sidebar',
      position: 'left',
      label: 'Module 1: ROS 2 Fundamentals',
    },
    // ADD: Module 2–4 navigation
    {
      type: 'docSidebar',
      sidebarId: 'module2Sidebar',
      position: 'left',
      label: 'Module 2: The Digital Twin',
    },
    {
      type: 'docSidebar',
      sidebarId: 'module3Sidebar',
      position: 'left',
      label: 'Module 3: The AI-Robot Brain',
    },
    {
      type: 'docSidebar',
      sidebarId: 'module4Sidebar',
      position: 'left',
      label: 'Module 4: Vision-Language-Action',
    },
    // GitHub link
    {
      href: 'https://github.com/Shahb/hackthon_humanoid_book',
      label: 'GitHub',
      position: 'right',
    },
  ],
}
```

---

## Performance Optimization

### Build Optimization

```bash
# Analyze bundle size
npm run build -- --analyze

# Expected sizes:
# - JavaScript: ~500 KB (gzipped)
# - CSS: ~50 KB (gzipped)
# - Total: ~2 MB (uncompressed) / 500 KB (gzipped)
```

### SEO & Metadata

Add to docusaurus.config.ts:

```typescript
themeConfig: {
  // ... existing config
  metadata: [
    {
      name: 'description',
      content: 'Learn ROS 2, humanoid robotics, NVIDIA Isaac Sim, and AI-driven robot behavior',
    },
    {
      name: 'keywords',
      content: 'ROS 2, robotics, humanoid, Isaac Sim, URDF, VSLAM, Nav2',
    },
  ],
}
```

### Search Integration

Docusaurus automatically generates search index. No additional config needed.

Test search:
1. Deploy site
2. Click search icon (magnifying glass)
3. Type: "VSLAM" or "ROS 2"
4. Should show matching chapters

---

## Troubleshooting Guide

### Build Issues

**Error: "Cannot find module '@docusaurus/core'"**
```bash
rm -rf node_modules package-lock.json
npm install
npm run build
```

**Error: "The following files have invalid IDs"**
- Cause: Sidebar references non-existent files
- Fix: Check sidebars.ts against docs/ directory structure

**Error: "onBrokenLinks" when building**
- Cause: Broken internal links
- Fix: Run Phase 3 link validator: `python tools/validate-links.py --all`

### Deployment Issues

**Site doesn't appear after deployment**
- Check GitHub Pages is enabled (Settings > Pages)
- Verify publish branch is "gh-pages"
- Wait 1–2 minutes for GitHub Pages to build

**Styles look broken on deployed site**
- Cause: baseUrl incorrect
- Fix: Ensure baseUrl = `/hackthon_humanoid_book/` (with slashes!)
- Rebuild: `npm run build && npm run deploy`

**404 errors on chapters**
- Cause: Page structure vs. sidebar mismatch
- Fix: Verify sidebars.ts references match docs/ file paths

### Diagram Issues

**Mermaid diagrams not rendering**
- Verify `markdown: { mermaid: true }` in docusaurus.config.ts
- Verify `themes: ['@docusaurus/theme-mermaid']` included
- Try: `npm install @docusaurus/theme-mermaid`

**ASCII diagrams showing code instead of text**
- This is expected (ASCII in code blocks)
- They display correctly on deployed site

---

## Post-Deployment Checklist

After site goes live:

- [ ] Site loads at GitHub Pages URL
- [ ] All 4 modules accessible via navbar
- [ ] All 15 chapters load without 404s
- [ ] Mermaid diagrams render correctly
- [ ] Links (internal & external) work
- [ ] Search functionality works
- [ ] Mobile view is responsive
- [ ] Dark/light mode toggle works
- [ ] Code blocks display correctly
- [ ] Images load properly
- [ ] No console errors (open DevTools)
- [ ] SSL certificate active (lock icon)

## Metrics & Monitoring

### Site Analytics Setup (Optional)

Add Google Analytics:

```typescript
themeConfig: {
  // ... existing config
  metadata: [
    {
      name: 'google-site-verification',
      content: 'YOUR_VERIFICATION_CODE',
    },
  ],
}
```

### Performance Monitoring

Test site with:
- **Lighthouse**: chrome://inspect > Lighthouse
  - Target: 90+ Performance, 95+ Accessibility
- **WebPageTest**: webpagetest.org
  - First Contentful Paint: <1s
  - Largest Contentful Paint: <2.5s

---

## What Happens Next (Phases 6–13)

### Immediate (Phases 6–7)

**Phase 6**: Build FastAPI backend
- RAG query endpoint
- Chunk retrieval
- LLM integration

**Phase 7**: Integrate RAG chatbot
- Chat interface
- Semantic search
- Citation generation

### Future (Phases 8–13)

**Phase 8**: Add authentication (optional)
**Phase 9**: Multi-language support (optional)
**Phase 10**: Analytics & feedback loop
**Phase 11**: QA & testing
**Phase 12**: Production deployment
**Phase 13**: Iteration & improvement

---

## Success Criteria

Phase 5 is complete when:

✅ **Site Builds**
- npm run build succeeds with no errors
- build/ directory created with all assets

✅ **Site Deploys**
- npm run deploy succeeds
- GitHub Pages shows publication
- SSL certificate active

✅ **Site Works**
- All 4 modules accessible
- All 15 chapters load
- No 404 errors
- All diagrams render
- Search works
- Links work

✅ **Site Performs**
- Page load: <2s
- Search response: <500ms
- Mobile responsive
- Dark mode works

---

## Summary

**Phase 5 Deliverables**:
1. ✅ Docusaurus configured (already done)
2. ⏳ Build pipeline tested
3. ⏳ GitHub Pages deployment verified
4. ⏳ Site live at GitHub Pages URL
5. ⏳ Post-deployment QA passed

**Estimated Time**: 30 minutes (if no issues)

**Next**: Phase 6 — FastAPI Backend Setup

---

## Quick Reference: Common Commands

```bash
# Start development server
cd my-website && npm run start

# Build for production
npm run build

# Deploy to GitHub Pages
npm run deploy

# Check for broken links
cd .. && python tools/validate-links.py --all

# Check file structure
find docs -name "*.md" | wc -l  # Should be ~19 files (15 chapters + 4 intros)
```

---

**Status**: Phase 5 implementation guide complete
**Next Step**: Execute deployment (npm run build && npm run deploy)
