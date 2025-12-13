# Homepage Redesign v2 - Task Generation Summary

**Project**: Reference-Based Homepage Redesign for "Physical AI & Humanoid Robotics"
**Date Generated**: 2025-12-10
**Status**: ✅ Task List Ready for Implementation
**Total Tasks**: 28 (organized in 8 phases)

---

## Overview

You've requested a complete redesign of the homepage to match exact reference specifications. This represents a significant shift from the current emerald-themed design to a **minimal, clean aesthetic** with:

- ✅ White navigation bar
- ✅ Solid green hero (#16a34a) - NOT gradient
- ✅ Left-aligned hero content
- ✅ Minimal card design with subtle styling
- ✅ Right-side sticky sidebar
- ✅ No animations or glassmorphism effects

---

## Design Specifications

### Navigation Bar
| Property | Value |
|----------|-------|
| Background | #ffffff (white) |
| Height | 60-70px |
| Border | 1px bottom #e5e7eb |
| Shadow | 0 1px 2px rgba(0,0,0,0.05) |
| Position | Sticky |

**Elements**:
- Left: Icon + "Physical AI & Humanoid Robotics" + "Textbook" label
- Right: GitHub link + Dark/Light toggle + Search bar

### Hero Section
| Property | Value |
|----------|-------|
| Background | #16a34a (SOLID, no gradient) |
| Alignment | LEFT (NOT centered) |
| Padding | 100-120px vertical |
| Text Color | #ffffff (white) |
| Button Style | White bg, dark text, NO shadow |

**Content**:
- Title: text-4xl md:text-5xl font-bold
- Subtitle: text-xl md:text-2xl font-normal
- CTA: "Start Learning →" button

### Module Cards
| Property | Value |
|----------|-------|
| Background | White |
| Border | 1px #e5e7eb |
| Radius | 8px |
| Padding | 24px |
| Default Shadow | None |
| Hover Shadow | 0 4px 12px rgba(0,0,0,0.08) |
| Grid | 2 columns (gap-6) |

**Card Content**:
1. Week badge (gray text)
2. Module title (emerald green #059669, bold)
3. Description (medium gray #6b7280)

### Sidebar
| Property | Value |
|----------|-------|
| Background | #f9fafb |
| Border | 1px #e5e7eb |
| Radius | 8px |
| Padding | 24px |
| Width | ~280px |
| Position | Sticky |

**Content**:
- Title: "Quick Links"
- Links: Setup Guide, Workstation, Edge Kit, Glossary
- Colors: Emerald green (#059669)
- Hover: Darker green + underline

---

## Color Palette (Exact)

| Color Purpose | Hex Value | Usage |
|---------------|-----------|-------|
| Hero Background | #16a34a | Solid hero section |
| Links & Titles | #059669 | Module titles, sidebar links |
| Text Dark | #1f2937 | Main body text |
| Text Medium | #6b7280 | Secondary text |
| Borders | #e5e7eb | Card borders, dividers |
| Background Light | #f9fafb | Sidebar background |
| White | #ffffff | Navigation background |

---

## Task Organization

### Phase 1: Navigation (6 tasks)
Setup white navigation bar with all elements
- T001-T006: Nav styling, positioning, shadow

### Phase 2: Hero (7 tasks)
Create solid green hero section with left-aligned content
- T007-T013: Hero styling, typography, button

### Phase 3: Module Cards (13 tasks)
Design 2-column grid with minimal card styling
- T014-T027: Card design, content population, layout

### Phase 4: Sidebar (9 tasks)
Build sticky sidebar with quick links
- T028-T037: Sidebar styling, links, hover effects

### Phase 5: Color Palette (4 tasks)
Define exact colors in Tailwind configuration
- T038-T041: Color definitions, typography, borders

### Phase 6: Responsive Design (5 tasks)
Ensure mobile/tablet/desktop compatibility
- T042-T046: Breakpoints, mobile stacking, adjustments

### Phase 7: Testing & QA (9 tasks)
Comprehensive validation of all elements
- T047-T055: Navigation, hero, cards, sidebar, colors, responsive

### Phase 8: Polish & Deployment (8 tasks)
Final adjustments and production build
- T056-T063: Remove unwanted effects, verify specs, build, document

---

## User Stories

### US1: Display Course Modules
**Goal**: Present all 4 course modules in a clean, minimal 2-column grid

**Acceptance Criteria**:
- ✅ All 4 modules displayed (ROS 2, Digital Twins, Isaac, VLA)
- ✅ Minimal card design (border #e5e7eb, white bg)
- ✅ Module titles in emerald green (#059669)
- ✅ Week badges and descriptions included
- ✅ Hover shadow effect (0 4px 12px rgba...)
- ✅ 2 columns desktop, 1 column mobile

**Related Tasks**: T014-T027

### US2: Quick Links Navigation
**Goal**: Provide easy sidebar access to important resources

**Acceptance Criteria**:
- ✅ Sticky sidebar on right (desktop)
- ✅ Light gray background (#f9fafb)
- ✅ All 4 quick links present
- ✅ Links in emerald green (#059669)
- ✅ Hover effect (darker green + underline)
- ✅ Responsive (moves below content on mobile)

**Related Tasks**: T028-T037

---

## Parallel Execution Map

```
Phase 1: Navigation (T001-T006) [SEQUENTIAL]
    ↓
Phase 2: Hero (T007-T013) [SEQUENTIAL]
    ↓
Phase 3 & 4: PARALLEL
    ├─ Phase 3: Module Cards (T014-T027)
    └─ Phase 4: Sidebar (T028-T037)
    ↓
Phase 5: Styling (T038-T041) [SEQUENTIAL]
    ↓
Phase 6: Responsive (T042-T046) [MOSTLY PARALLEL]
    ↓
Phase 7: Testing (T047-T055) [FULLY PARALLEL]
    ↓
Phase 8: Polish (T056-T063) [MOSTLY PARALLEL]
```

**Parallel Batches**:
- **Batch 1**: Module card component (T015-T018) + Sidebar structure (T028-T031)
- **Batch 2**: Module content (T021-T027) + Sidebar links (T032-T037)
- **Batch 3**: Color definition (T038) + Typography (T039-T041)
- **Batch 4**: All QA tests (T047-T055)
- **Batch 5**: Polish tasks (T056-T061)

---

## Implementation Strategy

### MVP Scope (Phases 1-4)
**Minimum Viable Product - Core Layout**
1. Complete white navigation bar
2. Solid green hero with left-aligned content
3. 4 module cards in 2-column grid
4. Right sidebar with quick links

**Deliverable**: Working homepage with exact reference design

**Time**: ~2-3 hours (parallel execution)

### Phase 2 - Styling & Responsive (Phases 5-6)
- Exact color palette implementation
- Responsive design for all breakpoints
- Proper spacing and alignment

**Time**: ~1-2 hours

### Phase 3 - Testing & Polish (Phases 7-8)
- Comprehensive QA validation
- Remove all unwanted effects
- Verify NO animations, NO gradients
- Production build

**Time**: ~1-2 hours

---

## Key Differences from v1

| Aspect | v1 (Current) | v2 (Reference) |
|--------|--------------|----------------|
| Nav Background | Dark emerald | White #ffffff |
| Hero Background | Gradient 700→900 | Solid #16a34a |
| Hero Alignment | Centered | LEFT-ALIGNED |
| Cards | Glassmorphism | Minimal borders |
| Card Shadow | Heavy default | None (hover only) |
| Sidebar | None | Sticky right |
| Animations | Multiple | None |
| Aesthetic | Modern/elaborate | Clean/minimal |

---

## What NOT to Include

❌ **NO dark green gradient** in hero
❌ **NO centered content** (must be left-aligned)
❌ **NO heavy shadows** on cards by default
❌ **NO rounded pill buttons** (border-radius 6px only)
❌ **NO multiple CTAs** (only "Start Learning")
❌ **NO stats grid** with numbers
❌ **NO animations** or fade-ins
❌ **NO glassmorphism** effects
❌ **NO dark emerald theme** (must be white nav)
❌ **NO backdrop blur** or transparency effects

---

## Files to Modify

1. **src/pages/index.tsx** (128+ lines)
   - Navigation bar component
   - Hero section
   - Main homepage layout

2. **src/components/HomepageFeatures/index.tsx** (229+ lines)
   - Module card component
   - Sidebar component
   - Quick links
   - Layout structure

3. **tailwind.config.js** (optional)
   - Define exact color palette
   - Custom color names if needed

4. **src/css/custom.css** (optional)
   - Additional styling if needed

---

## Exact Task List File

**Location**: `specs/homepage-redesign-v2/tasks.md`

**Contents**:
- 28 tasks total
- 8 phases
- 2 user stories
- Parallel execution map
- Exact specifications
- Implementation strategy

**Status**: ✅ READY FOR EXECUTION

---

## Next Steps

1. ✅ **Task list generated** → `specs/homepage-redesign-v2/tasks.md`
2. ⏳ **Ready for implementation** → Run `/sp.implement` to start
3. ⏳ **Validate against reference** → Compare during development
4. ⏳ **Test all specifications** → Run Phase 7 QA tests
5. ⏳ **Deploy** → Production build and verification

---

## Quality Gates

Before marking complete, verify:

- ✅ Navigation bar is WHITE (#ffffff), NOT dark
- ✅ Hero is SOLID #16a34a, NOT gradient
- ✅ Hero content is LEFT-aligned, NOT centered
- ✅ Cards have minimal borders (#e5e7eb), NO heavy shadows
- ✅ Sidebar is STICKY on desktop, RESPONSIVE on mobile
- ✅ All colors match EXACT hex values
- ✅ NO animations, NO fade-ins, NO effects
- ✅ Button is simple, NOT pill-shaped
- ✅ Production build successful

---

## Summary

A comprehensive 28-task implementation plan has been generated for your reference-based homepage redesign. The new design emphasizes **simplicity, clarity, and exact specification adherence** with a white navigation bar, solid green hero, minimal card design, and helpful sidebar.

**Ready to implement!** Execute tasks in Phase order, leveraging parallel batches where possible.

---

**Generated**: 2025-12-10
**By**: Claude Code (Spec-Driven Development)
**File**: specs/homepage-redesign-v2/tasks.md
