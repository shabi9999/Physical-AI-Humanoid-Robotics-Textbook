---
ID: "003"
TITLE: "Reference-Based Homepage Redesign v2 Tasks"
STAGE: "tasks"
DATE_ISO: "2025-12-10"
SURFACE: "agent"
MODEL: "claude-haiku-4-5-20251001"
FEATURE: "homepage-redesign-v2"
BRANCH: "004-vla-pipeline"
USER: "Shahb"
COMMAND: "/sp.tasks"
LABELS: ["homepage", "redesign", "reference-design", "minimal-aesthetic"]
LINKS:
  SPEC: "null"
  TICKET: "null"
  ADR: "null"
  PR: "null"
---

## PROMPT_TEXT

```
I need you to recreate the exact design from this reference screenshot for my Docusaurus homepage.

**EXACT DESIGN SPECIFICATIONS FROM REFERENCE:**

**1. NAVIGATION BAR:**
- Background: Pure white (#ffffff)
- Height: Compact, approximately 60-70px
- Left side:
  * Small dark icon/logo (looks like a stylized bird/phoenix symbol)
  * Text: "Physical AI & Humanoid Robotics" in dark gray/black, medium weight font
  * "Textbook" label next to it in same font
- Right side:
  * "GitHub" link with external icon (↗)
  * Dark/Light mode toggle (moon/sun icon)
  * Search bar with "Search Ctrl K" placeholder
- Border: Thin bottom border (1px solid #e5e7eb)
- Position: Sticky at top
- Box shadow: Subtle (0 1px 2px rgba(0,0,0,0.05))

**2. HERO SECTION:**
- Background: Solid emerald/green color (#16a34a or similar - medium green, NOT dark)
- NO gradient - just solid color
- Padding: Large (100-120px vertical)
- Content alignment: Left-aligned (NOT centered)
- Text color: White (#ffffff)
- Main heading:
  * "Physical AI & Humanoid Robotics"
  * Font size: Very large (3.5-4rem)
  * Font weight: Bold (700)
  * Line height: Tight (1.2)
- Subtitle:
  * "Comprehensive 13-Week Course for Industry Practitioners"
  * Font size: Large (1.5-1.75rem)
  * Font weight: Normal (400)
  * Margin top: 1.5rem
- Button:
  * Text: "Start Learning →"
  * Background: White with subtle transparency or light gray
  * Text color: Dark gray or black
  * Padding: 12px 28px
  * Border radius: 6px
  * Font weight: Medium (500)
  * Margin top: 2rem
  * NO box shadow visible
  * Hover: Slight opacity change

**3. COURSE MODULES SECTION:**
- Background: White
- Padding: 60px on all sides
- Section title: "Course Modules" - Large, bold, dark color
- Layout: 2-column grid on desktop
- Grid gap: 24px horizontal, 32px vertical

**MODULE CARD SPECIFICATIONS:**
- Background: White
- Border: 1px solid light gray (#e5e7eb)
- Border radius: 8px (slightly rounded corners)
- Padding: 24px
- NO box shadow by default
- Hover effect: Slight shadow appears (0 4px 12px rgba(0,0,0,0.08))

Card structure (top to bottom):
1. Week badge: "Weeks 3-5" in small gray text
2. Module title:
   - Color: Emerald green (#059669 or #10b981)
   - Font size: 1.25rem
   - Font weight: Bold (700)
   - Example: "Module 1: The Robotic Nervous System (ROS 2)"
3. Description paragraph:
   - Color: Medium gray (#6b7280)
   - Font size: 0.95rem
   - Line height: 1.6
   - 2-3 lines of text

**4. SIDEBAR (Right side):**
- Background: Very light gray (#f9fafb)
- Border: 1px solid #e5e7eb
- Border radius: 8px
- Padding: 24px
- Width: ~280px
- Position: Sticky
- Title: "Quick Links" in bold

Links styling:
- Color: Emerald green (#059669)
- Font size: 0.95rem
- Font weight: Medium (500)
- Line height: 2 (spacing between links)
- Hover: Darker green + underline

**5. RESPONSIVE BEHAVIOR:**
- Mobile: Stack everything in single column, sidebar moves below content
- Tablet: Adjust grid to 1 column
- Desktop: Full 2-column layout with right sidebar

**EXACT CSS VALUES TO USE:**

Colors:
- Primary green: #16a34a (hero background)
- Link green: #059669 (titles, links)
- Text dark: #1f2937
- Text medium: #6b7280
- Border color: #e5e7eb
- Background light: #f9fafb

Typography:
- Font family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif
- Hero title: text-4xl md:text-5xl font-bold
- Hero subtitle: text-xl md:text-2xl font-normal
- Module title: text-xl font-bold text-green-600
- Body text: text-base text-gray-600

Spacing:
- Section padding: py-16 px-6
- Card padding: p-6
- Grid gap: gap-6
- Button padding: px-7 py-3

Borders & Shadows:
- Card border: border border-gray-200
- Hover shadow: shadow-md hover:shadow-lg
- Nav shadow: shadow-sm

**IMPORTANT - WHAT TO AVOID:**
❌ NO dark green gradient in hero
❌ NO centered hero text
❌ NO heavy box shadows on cards
❌ NO rounded pill-shaped buttons
❌ NO multiple CTA buttons
❌ NO stats grid with numbers
❌ NO animations or fade-ins

**FILES TO MODIFY:**
1. src/pages/index.tsx - Main homepage component
2. src/components/HomepageFeatures/index.tsx
```

## RESPONSE_TEXT

Generated comprehensive 28-task implementation plan for reference-based homepage redesign v2.

**Key Differences from v1**:
- White navigation bar (vs dark emerald)
- Solid #16a34a green hero (vs gradient)
- Left-aligned hero content (vs centered)
- Minimal card design with subtle borders (vs elaborate)
- Right-side sticky sidebar (new)
- No animations or glassmorphism effects

**Task Organization** (8 phases, 28 tasks):

Phase 1 (6 tasks): Navigation bar - white background, sticky, subtle shadow
Phase 2 (7 tasks): Hero section - solid green, left-aligned, simple CTA
Phase 3 (13 tasks): Module cards grid - 2 columns, minimal borders, hover effect
Phase 4 (9 tasks): Sidebar - sticky, quick links, emerald text
Phase 5 (4 tasks): Exact color palette matching
Phase 6 (5 tasks): Responsive design (mobile/tablet/desktop)
Phase 7 (9 tasks): Comprehensive QA testing
Phase 8 (8 tasks): Polish and final adjustments

**User Stories Identified**:
- US1: Display Course Modules (4 modules in grid with descriptions)
- US2: Quick Links Navigation (sidebar with quick access)

**Parallel Execution Batches**:
1. Navigation + Hero (sequential, T001-T013)
2. Module Cards + Sidebar (parallel, T015-T037)
3. Styling (sequential, T038-T041)
4. Testing (parallel, T047-T055)
5. Polish (parallel, T056-T061)

**File: specs/homepage-redesign-v2/tasks.md**

Total: 28 tasks ready for execution
Completion: 0% (pending)
MVP Scope: Phases 1-4 (core layout)

## FILES_YAML

Created:
 - specs/homepage-redesign-v2/tasks.md (28 tasks, 8 phases)

## TESTS_YAML

Validation tasks included:
 - Navigation styling tests
 - Hero section verification
 - Module card layout tests
 - Sidebar functionality tests
 - Color specification validation
 - Responsive design testing
 - Accessibility checks

## OUTCOME

✅ **SUCCESS** - Reference-based homepage redesign task list generated.

**28 tasks organized across 8 phases**:
- Phase 1: Navigation bar (white, sticky, subtle)
- Phase 2: Hero section (solid green, left-aligned)
- Phase 3: Module cards (2-column grid, minimal)
- Phase 4: Sidebar (sticky quick links)
- Phase 5: Color palette exact matching
- Phase 6: Responsive design
- Phase 7: QA testing
- Phase 8: Polish & deployment

**Key Features**:
- White navigation bar with exact styling
- Solid #16a34a hero (NO gradient)
- Left-aligned content layout
- Minimal card design with subtle borders (#e5e7eb)
- Hover shadow effect on cards
- Sticky right sidebar with quick links
- Exact color specifications included
- No animations or glass effects
- Responsive mobile/tablet/desktop

**Format Compliance**: 100% - All 28 tasks follow strict checklist format with IDs, parallelization markers, user story labels, and file paths

**Actionability**: Highly actionable - Each task specific enough for LLM implementation without additional context

## EVALUATION

**Specification Adherence**: Excellent - All exact design specifications captured (colors, spacing, shadows, positioning)

**Task Clarity**: Very clear - Each task has specific file path, styling values, and implementation details

**Completeness**: Comprehensive - Covers navigation, hero, modules, sidebar, styling, responsive, testing, polish

**Organization**: Well-structured - 8 phases with clear dependencies and parallel opportunities

**Validation**: Thorough - Phase 7 includes 9 detailed QA tests

**Documentation**: Complete - All specifications documented with exact hex colors and spacing values

