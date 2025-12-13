# Homepage Design Guide - Visual Structure

## Page Layout

```
┌─────────────────────────────────────────────────────────┐
│  🤖 Physical AI       Textbook  |  GitHub              │  <- NAVBAR (Sticky)
├─────────────────────────────────────────────────────────┤
│                                                         │
│              HERO SECTION (Min 100vh)                  │
│                                                         │
│          Physical AI & Humanoid Robotics              │
│    Comprehensive 13-Week Course for Industry         │
│              Practitioners                             │
│                                                         │
│          Master ROS 2, Digital Twins...               │
│                                                         │
│     [START LEARNING →]    [VIEW ON GITHUB]            │
│                                                         │
│  ┌────────┐  ┌────────┐  ┌────────┐  ┌────────┐     │
│  │  4     │  │  20    │  │  56+   │  │  13    │     │
│  │Modules │  │Chapters│  │Topics  │  │ Weeks  │     │  <- STATS CARDS
│  └────────┘  └────────┘  └────────┘  └────────┘     │
│                                                         │
│  ┌────────┐  ┌────────┐                               │
│  │  20+   │  │  66+   │                               │
│  │Diagrams│  │C-Links │                               │
│  └────────┘  └────────┘                               │
│                                                         │
├─────────────────────────────────────────────────────────┤
│                                                         │
│            COURSE MODULES SECTION                      │
│    A comprehensive learning path spanning...           │
│                                                         │
│  ┌─────────────────────┐  ┌─────────────────────┐    │
│  │ 🧠 Module 1: ROS 2  │  │ 🎮 Module 2: Twins  │    │
│  │ Weeks 3-5           │  │ Weeks 6-7           │    │
│  │                     │  │                     │    │
│  │ Description...      │  │ Description...      │    │
│  │                     │  │                     │    │
│  │ ✓ Learning O...     │  │ ✓ Learning O...     │    │  <- MODULE CARDS
│  │ ✓ Learning O...     │  │ ✓ Learning O...     │    │
│  │ ✓ Learning O...     │  │ ✓ Learning O...     │    │
│  │                     │  │                     │    │
│  │ [EXPLORE MODULE →]  │  │ [EXPLORE MODULE →]  │    │
│  └─────────────────────┘  └─────────────────────┘    │
│                                                         │
│  ┌─────────────────────┐  ┌─────────────────────┐    │
│  │ 🚀 Module 3: Isaac  │  │ 🤖 Module 4: VLA    │    │
│  │ Weeks 8-10          │  │ Weeks 11-13         │    │
│  │                     │  │                     │    │
│  │ Description...      │  │ Description...      │    │
│  │                     │  │                     │    │
│  │ ✓ Learning O...     │  │ ✓ Learning O...     │    │
│  │ ✓ Learning O...     │  │ ✓ Learning O...     │    │
│  │ ✓ Learning O...     │  │ ✓ Learning O...     │    │
│  │                     │  │                     │    │
│  │ [EXPLORE MODULE →]  │  │ [EXPLORE MODULE →]  │    │
│  └─────────────────────┘  └─────────────────────┘    │
│                                                         │
├─────────────────────────────────────────────────────────┤
│                                                         │
│              QUICK LINKS SECTION                       │
│                                                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────┐ │
│  │    ⚙️    │  │    📦    │  │    ☁️    │  │  📚  │ │
│  │ Workst. │  │  Edge    │  │  Cloud   │  │ Gloss│ │
│  │ Setup   │  │  Kit     │  │  Setup   │  │ ary  │ │
│  └──────────┘  └──────────┘  └──────────┘  └──────┘ │
│                                                         │
├─────────────────────────────────────────────────────────┤
│                                                         │
│            RECENT UPDATES SECTION                      │
│                                                         │
│  📰 Recent Updates                                     │
│                                                         │
│  │  2025-12-09 | Docusaurus Deployment Complete      │
│  │     Homepage redesigned with Tailwind CSS...       │
│  │                                                     │
│  │  2025-12-09 | Textbook Structure Initialized       │
│  │     Complete textbook structure with 4 modules...  │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## Color Scheme

### Background Colors
- **Primary BG**: `#0f172a` (slate-900)
- **Secondary BG**: `#1e293b` (slate-800)
- **Hover BG**: `#334155` (slate-700)
- **Card BG**: `rgba(30, 41, 59, 0.5)` (slate-800 with transparency)

### Text Colors
- **Primary Text**: `#ffffff` (white)
- **Secondary Text**: `#cbd5e1` (slate-300)
- **Tertiary Text**: `#94a3b8` (slate-400)

### Accent Colors
```
Module 1: Cyan (#22d3ee) → Blue (#3b82f6)
Module 2: Green (#22c55e) → Emerald (#10b981)
Module 3: Purple (#a855f7) → Pink (#ec4899)
Module 4: Orange (#f97316) → Red (#ef4444)
```

### Interactive Colors
- **Hover Text**: Cyan (#22d3ee)
- **Hover Border**: Cyan with opacity
- **Gradient Hover**: Cyan to Blue gradient

---

## Typography

### Headings
- **H1 (Hero)**: 48px (mobile) → 56px (tablet) → 72px (desktop)
- **H2 (Section)**: 36px → 48px
- **H3 (Card)**: 28px
- **Font Weight**: Bold (700)

### Body Text
- **Large**: 20px, Light (300)
- **Medium**: 18px, Regular (400)
- **Small**: 16px, Regular (400)
- **XSmall**: 14px, Regular (400)

### Special
- **Navigation**: 16px, Medium (500)
- **Labels**: 12px, Bold (700), Uppercase
- **Stats**: 30px, Bold (700)

---

## Spacing

### Vertical Spacing
- **Hero Min Height**: 100vh (100% viewport height)
- **Sections**: 96px padding (py-24)
- **Cards Gap**: 32px (gap-8)
- **Quick Links Gap**: 16px (gap-4)

### Horizontal Spacing
- **Mobile Padding**: 24px (px-6)
- **Desktop Padding**: 48px (px-12)
- **Max Width**: 1280px (max-w-7xl)

---

## Components & States

### Module Card States

**Default State:**
```
┌─────────────────────┐
│ 🧠                  │
│ [CYAN→BLUE BADGE]   │
│                     │
│ Module 1: Title     │ <- Gray text
│                     │
│ Description...      │ <- Light gray
│                     │
│ ✓ Learning O1       │
│ ✓ Learning O2       │
│ ✓ Learning O3       │
│                     │
│ [EXPLORE MODULE →]  │
└─────────────────────┘
```

**Hover State:**
```
┌═════════════════════┐
│ 🧠                  │
│ [CYAN→BLUE BADGE]   │
│                     │
│ Module 1: Title ✨   │ <- Gradient text
│                     │
│ Description...      │ <- Light gray
│                     │
│ ✓ Learning O1       │
│ ✓ Learning O2       │
│ ✓ Learning O3       │
│                     │
│ [EXPLORE MODULE →]  │
└═════════════════════┘
(Larger shadow, lighter border)
```

### Button States

**Primary Button (Start Learning)**

Default:
- BG: Gradient Cyan→Blue
- Text: White
- Padding: 32px 32px
- Border Radius: 8px

Hover:
- Shadow: Cyan glow (shadow-cyan-500/50)
- Scale: 105% (scale-up effect)
- Smooth transition

**Secondary Button (GitHub)**

Default:
- BG: Transparent
- Border: 2px slate-400
- Text: White
- Padding: 32px 32px

Hover:
- Border: Cyan
- Text: Cyan
- Smooth transition

---

## Responsive Breakpoints

### Mobile (< 640px)
- Single column module grid
- 3-column stats grid (2 on very small)
- Stacked buttons (flex-col)
- 24px padding (px-6)
- 48px font size for title

### Tablet (640px - 1024px)
- 2-column module grid
- 6-column stats grid
- Row buttons (flex-row)
- 48px padding (px-12)
- 56px font size for title

### Desktop (> 1024px)
- Full 2x2 module grid
- 6-column stats perfect
- Row buttons (flex-row)
- 48px+ padding
- 72px font size for title

---

## Visual Hierarchy

```
LEVEL 1: Hero Title (Largest, gradient, most prominent)
  ↓
LEVEL 2: Subtitles & Module Titles
  ↓
LEVEL 3: Body Text & Descriptions
  ↓
LEVEL 4: Labels & Small Text
```

---

## Animation Timings

- **Hover Transitions**: 200-300ms
- **Scale Effects**: 105% (slight)
- **Color Changes**: Smooth gradient transitions
- **Shadow Effects**: Glowing cyan glow

---

## Accessibility Features

✅ **Color Contrast**:
- White text on dark background (WCAG AAA)
- All interactive elements meet contrast ratios

✅ **Focus States**:
- Visible focus rings on interactive elements
- Keyboard navigation support

✅ **Responsive Text**:
- Mobile-optimized font sizes
- Line heights for readability

✅ **Semantic HTML**:
- Proper heading hierarchy
- Link tags for navigation

---

## Design Principles Applied

1. **Dark Theme** - Less eye strain, modern look
2. **Gradient Accents** - Visual interest without clutter
3. **Clear Hierarchy** - Users know where to focus
4. **Ample Whitespace** - Breathable, clean layout
5. **Micro-interactions** - Subtle hover effects
6. **Mobile-First** - Responsive on all devices
7. **Performance** - CSS-based, minimal JavaScript
8. **Accessibility** - WCAG compliant

---

## File Structure

```
my-website/src/
├── pages/
│   └── index.tsx (Hero + Stats + Navigation)
├── components/
│   └── HomepageFeatures/
│       └── index.tsx (Module Cards + Quick Links)
└── css/
    └── custom.css (Tailwind directives)
```

---

## Browser Rendering

The page uses:
- ✅ Tailwind CSS classes (compiled to static CSS)
- ✅ React functional components
- ✅ CSS Grid & Flexbox layouts
- ✅ CSS gradients and filters
- ✅ Responsive media queries

**Result**: Fast, lightweight, no JavaScript overhead!

---

**Design Status**: ✅ **COMPLETE & LIVE**
**Technology**: React + Tailwind CSS
**Performance**: Optimized for all devices
**Accessibility**: WCAG AAA compliant

