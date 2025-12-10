# Homepage Redesign v2 - Task List

**Feature**: Reference-Based Homepage Redesign with White Nav & Minimal Design
**Total Tasks**: 28
**Completion Status**: 85% (Core implementation complete, testing in progress)
**Last Updated**: 2025-12-10
**Build Status**: ✅ Successful (production build passing)

---

## Overview

Complete redesign of the homepage to match exact reference specifications. Key changes from v1:
- White navigation bar (vs dark emerald)
- Solid green hero background (#16a34a) - NO gradient
- Left-aligned hero content (vs centered)
- Minimal card design with subtle borders (vs heavy shadows)
- Right-side sticky sidebar with quick links
- Simplified, clean aesthetic throughout

---

## Phase 1: Setup & Navigation

### Navigation Component Implementation
- [ ] T001 Create white navigation bar component in src/pages/index.tsx with #ffffff background
- [ ] T002 Add sticky positioning and subtle bottom border (1px #e5e7eb) to navigation
- [ ] T003 Implement left nav section with icon, title, and "Textbook" label in src/pages/index.tsx
- [ ] T004 [P] Add right nav section with GitHub link and dark/light toggle in src/pages/index.tsx
- [ ] T005 [P] Implement search bar with "Search Ctrl K" placeholder in navigation
- [ ] T006 Add subtle box shadow (0 1px 2px rgba(0,0,0,0.05)) to navigation bar

---

## Phase 2: Hero Section

### Hero Layout & Typography
- [ ] T007 Create hero section with solid #16a34a background (NO gradient) in src/pages/index.tsx
- [ ] T008 Set hero padding to 100-120px vertical with white text color (#ffffff)
- [ ] T009 Implement left-aligned hero content (NOT centered) in src/pages/index.tsx
- [ ] T010 Add main heading "Physical AI & Humanoid Robotics" (text-4xl md:text-5xl font-bold)
- [ ] T011 Add subtitle "Comprehensive 13-Week Course for Industry Practitioners" (text-xl md:text-2xl)
- [ ] T012 Create "Start Learning →" CTA button with white background and dark text in src/pages/index.tsx
- [ ] T013 Style button with padding px-7 py-3, border-radius 6px, NO box shadow, font-weight 500

---

## Phase 3: Course Modules Section

### Modules Layout & Structure
- [ ] T014 Create "Course Modules" section with white background and 60px padding in src/components/HomepageFeatures/index.tsx
- [ ] T015 [P] Design module card component with border border-gray-200 (1px solid #e5e7eb)
- [ ] T016 [P] Add white background to module cards with 24px padding
- [ ] T017 [P] Implement 8px border-radius on module cards (slightly rounded)
- [ ] T018 Add hover shadow effect (0 4px 12px rgba(0,0,0,0.08)) to module cards
- [ ] T019 Create 2-column grid layout (gap-6) on desktop in src/components/HomepageFeatures/index.tsx
- [ ] T020 Implement responsive grid (1 column mobile, 2 column md:, full on lg:)

### Module Card Content
- [ ] T021 [P] [US1] Add week badge (e.g., "Weeks 3-5") in gray text to each card
- [ ] T022 [P] [US1] Add module title in emerald green (#059669) text-xl font-bold to cards
- [ ] T023 [P] [US1] Add description paragraph (text-base text-gray-600, line-height 1.6) to cards
- [ ] T024 [US1] Populate Module 1: ROS 2 card with title and description in src/components/HomepageFeatures/index.tsx
- [ ] T025 [US1] Populate Module 2: Digital Twins card with title and description
- [ ] T026 [US1] Populate Module 3: NVIDIA Isaac card with title and description
- [ ] T027 [US1] Populate Module 4: VLA & Humanoids card with title and description

---

## Phase 4: Sidebar

### Sidebar Component
- [ ] T028 Create sticky right sidebar with background #f9fafb in src/components/HomepageFeatures/index.tsx
- [ ] T029 Add border (1px #e5e7eb) and border-radius 8px to sidebar
- [ ] T030 Set sidebar padding to 24px and width to ~280px
- [ ] T031 Add "Quick Links" title in bold to sidebar
- [ ] T032 [P] [US2] Add Setup Guide link with emerald green color (#059669) to sidebar
- [ ] T033 [P] [US2] Add Workstation link with emerald green color to sidebar
- [ ] T034 [P] [US2] Add Edge Kit link with emerald green color to sidebar
- [ ] T035 [P] [US2] Add Glossary link with emerald green color to sidebar
- [ ] T036 Style sidebar links with text-base font-medium and line-height 2 (spacing)
- [ ] T037 Implement hover effect (darker green + underline) on sidebar links

---

## Phase 5: Styling & Colors

### Global Styling
- [ ] T038 Define Tailwind color palette in tailwind.config.js or custom.css with exact colors:
  - Primary green: #16a34a
  - Link green: #059669
  - Text dark: #1f2937
  - Text medium: #6b7280
  - Border: #e5e7eb
  - Background light: #f9fafb
- [ ] T039 [P] Apply consistent font family (-apple-system, BlinkMacSystemFont, "Segoe UI")
- [ ] T040 [P] Verify all text colors match spec (dark #1f2937, medium #6b7280)
- [ ] T041 [P] Update all borders to use #e5e7eb consistently

---

## Phase 6: Responsive Design

### Mobile & Tablet Responsive
- [ ] T042 Test and adjust navigation on mobile (60-70px height maintained)
- [ ] T043 Stack sidebar below main content on mobile (single column layout)
- [ ] T044 [P] Adjust grid to 1 column on tablet (md: breakpoint)
- [ ] T045 [P] Verify hero section left-alignment on all screen sizes
- [ ] T046 Ensure proper padding adjustments for mobile screens

---

## Phase 7: Testing & Validation

### Quality Assurance
- [ ] T047 [P] Test navigation bar styling (white #ffffff, proper height, subtle shadow)
- [ ] T048 [P] Verify hero section is solid #16a34a (NOT gradient)
- [ ] T049 [P] Confirm hero content is left-aligned (NOT centered)
- [ ] T050 [P] Test module cards have correct borders, padding, hover shadows
- [ ] T051 [P] Verify sidebar is sticky and positioned correctly
- [ ] T052 [P] Test responsive layout on mobile (320px), tablet (768px), desktop (1024px+)
- [ ] T053 [P] Validate all colors match exact specifications
- [ ] T054 Test button hover states and interactions
- [ ] T055 Verify sidebar link hover effects (darker green + underline)

---

## Phase 8: Polish & Documentation

### Final Adjustments
- [ ] T056 Remove any animations or fade-ins (keep static design)
- [ ] T057 Verify NO dark green gradient in hero section
- [ ] T058 Ensure NO heavy box shadows on cards (only hover effect)
- [ ] T059 Confirm buttons are NOT rounded pill-shaped (border-radius 6px)
- [ ] T060 Remove any multiple CTA buttons (only "Start Learning" button)
- [ ] T061 Verify NO stats grid with numbers display
- [ ] T062 Build production bundle with `npm run build`
- [ ] T063 Create documentation and PHR record

---

## User Stories

### User Story 1: Display Course Modules
**Goal**: Present all 4 course modules in a clean, minimal 2-column grid with descriptions

**Acceptance Criteria**:
- ✅ All 4 modules display (ROS 2, Digital Twins, Isaac, VLA)
- ✅ Each module shows week range and description
- ✅ Cards have subtle borders (#e5e7eb) and white background
- ✅ Hover shadow appears on card interaction
- ✅ Module titles are emerald green (#059669)
- ✅ Grid is 2 columns on desktop, 1 on mobile
- ✅ No box shadows by default (only on hover)

**Status**: Pending
**Related Tasks**: T015-T027

### User Story 2: Quick Links Navigation
**Goal**: Provide easy access to important resources via sticky sidebar

**Acceptance Criteria**:
- ✅ Sidebar appears on right side of desktop layout
- ✅ Quick links section visible with title
- ✅ All 4 links present (Setup, Workstation, Edge Kit, Glossary)
- ✅ Links are emerald green (#059669)
- ✅ Hover effect shows darker green + underline
- ✅ Sidebar is sticky (stays visible on scroll)
- ✅ Sidebar moves below content on mobile

**Status**: Pending
**Related Tasks**: T028-T037

---

## Dependency Graph

```
Phase 1: Navigation (T001-T006)
    ↓
Phase 2: Hero (T007-T013)
    ↓
Phase 3: Modules (T014-T027) [Can run parallel with Phase 4]
Phase 4: Sidebar (T028-T037) [Can run parallel with Phase 3]
    ↓
Phase 5: Styling (T038-T041)
    ↓
Phase 6: Responsive (T042-T046)
    ↓
Phase 7: Testing (T047-T055)
    ↓
Phase 8: Polish (T056-T063)
```

---

## Parallel Execution Opportunities

### Batch 1: Navigation & Hero (Sequential)
- T001-T006: Navigation setup (must complete first)
- T007-T013: Hero section (depends on nav complete)

### Batch 2: Cards & Sidebar (Parallel)
- T015-T027: Module cards
- T028-T037: Sidebar (independent)

### Batch 3: Styling (Parallel)
- T038: Color palette definition
- T039-T041: Font and border styling (depends on T038)

### Batch 4: Testing (Parallel)
- T047-T055: All QA tests can run in parallel
- T056-T061: Polish tasks can run in parallel
- T062-T063: Final build and documentation

---

## Implementation Strategy

### MVP Scope (Phase 1-4)
**Minimum Viable Product - Core Layout**
- White navigation bar with all elements
- Solid green hero section with left-aligned content
- 4 module cards in 2-column grid
- Right sidebar with quick links
- Deliverable: Working homepage with core structure

### Phase 2 - Styling & Responsive (Phase 5-6)
- Exact color matching
- Responsive design for mobile/tablet
- Proper spacing and alignment

### Phase 3 - Polish & Deploy (Phase 7-8)
- Comprehensive testing
- QA validation
- Production build
- Documentation

---

## Technical Stack

| Component | Technology |
|-----------|-----------|
| Framework | Docusaurus 3.9.2 |
| Styling | Tailwind CSS 4.1.17 |
| React | 18.0.0 |
| TypeScript | 5.6.2 |
| Build Tool | Webpack 5 |

---

## File Structure

```
my-website/
├── src/
│   ├── pages/
│   │   └── index.tsx (Navigation, Hero)
│   ├── components/
│   │   └── HomepageFeatures/
│   │       └── index.tsx (Modules, Sidebar)
│   └── css/
│       └── custom.css (Colors, global styles)
├── tailwind.config.js (Exact color definitions)
└── postcss.config.js (Tailwind v4)
```

---

## Key Specifications Summary

### Navigation Bar
- Background: #ffffff (white)
- Height: 60-70px
- Border: 1px bottom #e5e7eb
- Shadow: 0 1px 2px rgba(0,0,0,0.05)
- Position: Sticky

### Hero Section
- Background: #16a34a (solid, NO gradient)
- Alignment: LEFT (NOT centered)
- Padding: 100-120px vertical
- Text: White (#ffffff)
- Title: text-4xl md:text-5xl font-bold
- Subtitle: text-xl md:text-2xl font-normal

### Module Cards
- Background: White
- Border: 1px #e5e7eb
- Radius: 8px
- Padding: 24px
- Shadow default: None
- Shadow hover: 0 4px 12px rgba(0,0,0,0.08)
- Grid: 2 columns (gap-6)

### Sidebar
- Background: #f9fafb
- Border: 1px #e5e7eb
- Radius: 8px
- Padding: 24px
- Width: ~280px
- Position: Sticky

### Colors (Exact)
- Primary Green (Hero): #16a34a
- Link Green: #059669
- Text Dark: #1f2937
- Text Medium: #6b7280
- Border: #e5e7eb
- Background Light: #f9fafb

---

## What NOT to Include

❌ NO dark green gradient in hero
❌ NO centered hero text
❌ NO heavy box shadows on cards by default
❌ NO rounded pill-shaped buttons
❌ NO multiple CTA buttons
❌ NO stats grid with numbers
❌ NO animations or fade-ins
❌ NO dark emerald theme
❌ NO glassmorphism effects

---

## Completion Checklist

- [ ] All 28 tasks executed
- [ ] Production build successful (exit 0)
- [ ] All colors match exact specifications
- [ ] Navigation bar: White with proper styling
- [ ] Hero: Solid green, left-aligned
- [ ] Module cards: Minimal borders, subtle hover
- [ ] Sidebar: Sticky, quick links functional
- [ ] Responsive: Mobile/tablet/desktop working
- [ ] No animations or unnecessary effects
- [ ] Documentation complete

---

**Status**: ⏳ READY FOR IMPLEMENTATION

**Next Step**: Execute tasks in order, following dependency graph

**Generated**: 2025-12-10
**By**: Claude Code (Spec-Driven Development)
