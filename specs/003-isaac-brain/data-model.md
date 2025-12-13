# Phase 1 Design: Content Data Model & Metadata Schema

**Date**: 2025-12-08 | **Feature**: 003-isaac-brain | **Stage**: Design
**Input**: Research findings from [research.md](research.md) | **Output**: Content structure contracts

---

## 1. Chapter Metadata Schema (YAML Frontmatter)

### Required Fields

```yaml
---
# Document Identifiers
title: "Chapter 1: Isaac Sim Fundamentals"
module: "module3"                          # module identifier
chapter: 1                                  # chapter number (1-4)
id: "module3-chapter1-isaac-sim"           # unique identifier for cross-references

# Learning & Structure
learning_objectives:                        # 3-4 measurable outcomes
  - "Explain what photorealistic simulation is and why it's used in robotics"
  - "Describe the Isaac Sim physics engine and how it differs from game engines"
  - "Understand the complete workflow from 3D scene creation to simulation execution"
  - "Identify key Isaac Sim concepts: coordinate frames, cameras, sensors"

prerequisites:                              # cross-references to Module 1 or earlier chapters
  - "module1-intro"
  - "module1-chapter3"  # URDF modeling

related_chapters:                           # links to other Module 3 chapters
  - "module3-chapter2"  # Synthetic Data Generation (next chapter)

keywords:
  - "isaac-sim"
  - "photorealistic-simulation"
  - "physics-engine"
  - "3d-scene"
  - "sensors"

# Content Metadata
difficulty: "intermediate"                  # beginner | intermediate | advanced
estimated_reading_time: 15                 # minutes
estimated_word_count: 5000                 # target word count
created_at: "2025-12-08"
last_updated: "2025-12-08"

# RAG & Indexing
chunk_count: 4                             # expected chunks (512 tokens each)
searchable_terms:                          # enhanced RAG search
  - "photorealistic simulation"
  - "physics engine"
  - "Isaac Sim workflow"
  - "sensor simulation"
  - "3D scene creation"
---
```

### Field Definitions

| Field | Type | Required | Purpose |
|-------|------|----------|---------|
| `title` | string | ✅ | Chapter title for display |
| `module` | string | ✅ | Module identifier (module3) |
| `chapter` | number | ✅ | Chapter number for ordering |
| `id` | string | ✅ | Unique ID for cross-references and URLs |
| `learning_objectives` | array | ✅ | 3-4 testable learning outcomes |
| `prerequisites` | array | ✅ | List of prerequisite chapters (module1 or module3) |
| `related_chapters` | array | ✅ | Cross-references to other Module 3 chapters |
| `keywords` | array | ✅ | 5-8 searchable terms for RAG |
| `difficulty` | string | ✅ | Content complexity level |
| `estimated_reading_time` | number | ✅ | Minutes to read chapter |
| `estimated_word_count` | number | ✅ | Target word count (4,500-5,500) |
| `created_at` | string | ✅ | ISO date created |
| `last_updated` | string | ✅ | ISO date last modified |
| `chunk_count` | number | ✅ | Expected number of RAG chunks |
| `searchable_terms` | array | ✅ | Enhanced search terms for RAG |

---

## 2. Content Section Structure

### Standard Chapter Outline (Same for All 4 Chapters)

```markdown
# [Chapter Title]

## Learning Objectives

[Bulleted list from YAML]

## Prerequisites & Context

[Brief mention of what students should already know]

## Section 1: Core Concepts

### 1.1 [Concept Name]
- Definition
- Why it matters
- Relationship to other concepts
- Real-world relevance

### 1.2 [Concept Name]
...

## Section 2: Architecture & Workflow

- [Mermaid diagram showing component relationships]
- Step-by-step workflow description
- Data flow through system
- How components interact

## Section 3: Real-World Applications & Examples

### Example 1: [Application scenario]
- How the chapter's concepts apply
- Relevant NVIDIA Isaac features

### Example 2: [Another scenario]
...

## Section 4: Integration with ROS 2 & Module 1

- How this chapter's technologies connect to ROS 2 (Module 1)
- Relevant ROS 2 nodes, topics, and services
- Cross-references to specific Module 1 chapters
- Practical workflows combining Module 1 + Module 3

## Section 5: Key Takeaways

- **Core Idea 1**: [Summary]
- **Core Idea 2**: [Summary]
- **Core Idea 3**: [Summary]
- Bridge to next chapter

## Edge Cases & Troubleshooting

[Specific edge case from specification]

- Scenario description
- Why it happens
- How Isaac technologies handle it
- Student outcome

## What's Next

[Preview of Chapter N+1]

---

*Estimated reading time: X minutes | Estimated word count: X words*
*Keywords: [keyword1, keyword2, ...]*
```

---

## 3. Section Breakdown & Word Allocation

### Chapter 1: Isaac Sim Fundamentals (5,000 words)

| Section | Word Count | Purpose |
|---------|-----------|---------|
| Title + Intro | 200 | Context and relevance |
| Learning Objectives | 100 | Set expectations |
| Prerequisites | 150 | Review requirements |
| **Section 1: Core Concepts** | **1,500** | Define photorealistic simulation, physics engine, coordinate frames, sensors |
| **Section 2: Architecture & Workflow** | **1,200** | Describe Isaac Sim architecture, scene creation workflow |
| **Section 3: Real-World Applications** | **900** | 2 examples (object manipulation, robotic assembly) |
| **Section 4: Integration with ROS 2** | **600** | Link to URDF (Module 1 Ch3), sensor topics (Module 1 Ch1) |
| **Section 5: Key Takeaways** | **250** | Summary, bridge to Chapter 2 |
| Edge Cases | **300** | Simulator vs. reality gap |
| **Total** | **5,000** | |

### Chapter 2: Synthetic Data Generation (4,500 words)

| Section | Word Count | Purpose |
|---------|-----------|---------|
| Title + Intro | 200 | Why synthetic data matters |
| Learning Objectives | 100 | Set expectations |
| Prerequisites | 100 | Review Chapter 1 |
| **Section 1: Core Concepts** | **1,200** | Synthetic data, domain adaptation, data diversity |
| **Section 2: Architecture & Workflow** | **1,000** | Data generation pipeline, randomization techniques |
| **Section 3: Real-World Applications** | **800** | 2 examples (hand pose detection, object recognition) |
| **Section 4: Integration with ROS 2** | **500** | How data feeds into agents (Module 1 Ch2) |
| **Section 5: Key Takeaways** | **200** | Summary, bridge to Chapter 3 |
| Edge Cases | **250** | Sim-to-real gap, domain randomization |
| **Total** | **4,500** | |

### Chapter 3: Isaac ROS VSLAM (5,500 words)

| Section | Word Count | Purpose |
|---------|-----------|---------|
| Title + Intro | 250 | What is VSLAM and why it's important |
| Learning Objectives | 100 | Set expectations |
| Prerequisites | 150 | Review Chapters 1-2 |
| **Section 1: Core Concepts** | **1,600** | Visual odometry, feature detection, loop closure, ego-motion |
| **Section 2: Architecture & Workflow** | **1,300** | VSLAM pipeline, Isaac ROS node structure |
| **Section 3: Real-World Applications** | **1,000** | 2 examples (indoor exploration, factory mapping) |
| **Section 4: Integration with ROS 2** | **700** | Camera subscriptions, pose topics, map updates (Module 1) |
| **Section 5: Key Takeaways** | **250** | Summary, bridge to Chapter 4 |
| Edge Cases | **350** | Incomplete maps, featureless walls, loop closure failure |
| **Total** | **5,500** | |

### Chapter 4: Nav2 Path Planning (5,000 words)

| Section | Word Count | Purpose |
|---------|-----------|---------|
| Title + Intro | 200 | Why path planning matters |
| Learning Objectives | 100 | Set expectations |
| Prerequisites | 150 | Review Chapters 1-3 |
| **Section 1: Core Concepts** | **1,400** | Costmaps, path planning algorithms (A*, Dijkstra, RRT), trajectory following |
| **Section 2: Architecture & Workflow** | **1,200** | Global vs. local planning, Nav2 architecture |
| **Section 3: Real-World Applications** | **1,000** | 2 examples (autonomous delivery, crowd navigation) |
| **Section 4: Integration with ROS 2** | **600** | Goal topics, velocity commands, feedback (Module 1 Ch1) |
| **Section 5: Key Takeaways** | **250** | Summary, module recap |
| Edge Cases | **400** | No valid path, dynamic obstacles, replanning |
| **Total** | **5,000** | |

**Grand Total**: 20,000 words (4 chapters × 5,000-5,500 each)

---

## 4. RAG Chunking Schema

### Chunk Metadata Structure

```json
{
  "id": "module3-chapter1-chunk-001",
  "text": "chunk content (512 tokens ± 100)...",

  "metadata": {
    "module": "module3",
    "chapter": 1,
    "chapter_title": "Isaac Sim Fundamentals",
    "section": "Core Concepts",
    "section_number": 1,
    "subsection": "What is Photorealistic Simulation?",
    "chunk_index": 0,
    "chunk_total": 4,
    "token_count": 512,

    "url": "/docs/module3/chapter1-isaac-sim#section-1-core-concepts",
    "created_at": "2025-12-08T00:00:00Z",

    "keywords": [
      "photorealistic simulation",
      "isaac-sim",
      "physics engine"
    ],

    "learning_objectives": [
      "Explain what photorealistic simulation is",
      "Understand why it's used in robotics"
    ],

    "related_chapters": [
      "module3-chapter2",  # Next chapter
      "module1-chapter3"   # Related Module 1 chapter
    ]
  }
}
```

### Chunking Rules

1. **Chunk Size**: 512 tokens (400-800 range)
   - Count: ~750 words per 512 tokens
   - Calculate: word_count ÷ 512 × 750 ÷ 1.3 = estimated tokens

2. **Chunk Boundaries**:
   - Prefer: End of subsection or paragraph
   - Avoid: Splitting mid-sentence or mid-concept
   - If necessary: Add 102 tokens overlap (20%) from previous chunk

3. **Metadata Preservation**:
   - Every chunk includes: module, chapter, section, url
   - No metadata omitted (supports RAG filtering)

4. **Overlap Strategy**:
   - Tail of chunk N becomes head of chunk N+1
   - Preserves context across boundaries
   - Enables similarity search to find related chunks

5. **URL Generation**:
   - Pattern: `/docs/module3/chapter{N}-{topic}#{section-slug}`
   - Example: `/docs/module3/chapter1-isaac-sim#section-1-core-concepts`
   - Must be exact markdown heading for Docusaurus anchor links

---

## 5. Cross-Referencing Format

### Internal Links (Module 3 → Module 3)

```markdown
[See Chapter 2: Synthetic Data Generation](#section-2-architecture--workflow)
```

### External Links (Module 3 → Module 1)

```markdown
[Review ROS 2 publish/subscribe patterns from Module 1](/docs/module1/chapter1-core-concepts#topics-and-subscriptions)
```

### NVIDIA Documentation Links

```markdown
[Isaac Sim Physics Engine](https://docs.omniverse.nvidia.com/isaacsim/latest/physics/index.html)
```

### Inline Terminology Links (within chapter)

```markdown
See the [Core Concepts](#section-1-core-concepts) section for definitions of photorealistic simulation.
```

---

## 6. Code Examples & Pseudocode Format

### Policy: No Implementation Code

**Module 3 contains NO Python/C++/YAML code** (per specification SC-008, Out-of-Scope).

**Instead, use**:

1. **Pseudocode** (algorithm structure, not implementation):
   ```
   Algorithm: Feature Detection
     Input: Image from camera
     Output: List of feature keypoints

     1. Convert image to grayscale
     2. Compute gradients (direction of intensity change)
     3. Find corners (high gradient magnitude)
     4. Return keypoint list
   ```

2. **Conceptual Diagrams** (Mermaid or ASCII):
   ```
   Camera Input
        ↓
   [Feature Detection]
        ↓
   [Feature Matching]
        ↓
   [Ego-motion Estimation]
   ```

3. **Data Structure Examples** (conceptual only):
   ```
   Costmap: 2D grid where each cell is [0-254]
   - 0 = free space
   - 254 = obstacle
   - 128 = unknown
   ```

---

## 7. Testing & Quality Validation Gates

### Content Structure Validation

- [ ] Every chapter has YAML frontmatter with all required fields
- [ ] Every chapter has 5 main sections (Core Concepts, Architecture, Applications, Integration, Takeaways)
- [ ] Every chapter has learning objectives (3-4 minimum)
- [ ] Every chapter has prerequisites listed
- [ ] Every chapter has 1-2 real-world application examples
- [ ] Every chapter has cross-links to Module 1 (3-5 minimum)
- [ ] Every chapter addresses an edge case from specification

### Word Count Validation

- [ ] Chapter 1: 4,500-5,500 words ✓
- [ ] Chapter 2: 4,000-5,000 words ✓
- [ ] Chapter 3: 5,000-6,000 words ✓
- [ ] Chapter 4: 4,500-5,500 words ✓
- [ ] Total: 18,500-21,500 words ✓

### Technical Accuracy Validation

- [ ] All NVIDIA Isaac concepts match official documentation (SC-006, FR-022)
- [ ] All ROS 2 references match Module 1 content
- [ ] All edge cases addressed per specification
- [ ] No implementation code included (SC-008)
- [ ] Pseudocode is conceptual, not language-specific

### Readability Validation

- [ ] Flesch-Kincaid score 10-12 for each chapter
- [ ] Average sentence length 15-18 words
- [ ] Technical terms defined on first use
- [ ] Jargon minimized; everyday language preferred
- [ ] Subheadings every 300-400 words maximum

### Markdown & Docusaurus Validation

- [ ] Valid Markdown syntax (tested with `npx remark`)
- [ ] No broken internal links
- [ ] No broken NVIDIA documentation links
- [ ] Proper heading hierarchy (h2 for sections, h3 for subsections)
- [ ] All code blocks properly formatted
- [ ] All Mermaid diagrams render correctly

### RAG Chunk Validation

- [ ] All chunks 400-800 tokens (512 target)
- [ ] Chunk boundaries at section/subsection breaks
- [ ] Every chunk has complete metadata
- [ ] Cross-references included in chunk metadata
- [ ] URL anchors match Docusaurus slugs
- [ ] Overlap (20%) from previous chunk included

---

## 8. Docusaurus Configuration Changes

### sidebar.js Update (Pseudocode)

```javascript
{
  type: 'category',
  label: 'Module 3: The AI-Robot Brain',
  collapsed: false,
  items: [
    'module3/intro',
    {
      type: 'category',
      label: 'Core Chapters',
      items: [
        'module3/chapter1-isaac-sim',
        'module3/chapter2-synthetic-data',
        'module3/chapter3-vslam',
        'module3/chapter4-nav2',
      ],
    },
  ],
}
```

### docusaurus.config.js Update (Pseudocode)

```javascript
// Add to sidebar config
{
  type: 'link',
  label: 'Back to Module 1',
  href: '/docs/module1/intro',
  position: 'right',
}
```

---

## 9. Content Metadata Summary

| Attribute | Value | Notes |
|-----------|-------|-------|
| **Total Chapters** | 4 | Isaac Sim, Synthetic Data, VSLAM, Nav2 |
| **Total Words** | ~20,000 | 5,000/chapter ± 500 |
| **Expected Chunks** | ~16 | 4 chunks/chapter (512 tokens each) |
| **Cross-links per Chapter** | 3-5 | To Module 1 chapters |
| **Diagrams per Chapter** | 3-4 | Mermaid + ASCII |
| **Learning Objectives per Chapter** | 3-4 | Testable, measurable |
| **Edge Cases Covered** | 5 | Sim-to-real gap, incomplete maps, perception failure, path failure, computational limits |
| **Readability Grade** | 10-12 | Flesch-Kincaid |

---

## 10. Next Steps (Phase 2)

✅ **Phase 1 Complete**: Content data model defined

**Phase 2 Actions**:
1. Create `contracts/chapter-structure.md` with acceptance criteria for each chapter
2. Create `quickstart.md` with student learning path and navigation guide
3. Finalize chapter outline templates (based on this data model)
4. Prepare for `/sp.tasks` to generate fine-grained writing tasks

---

**Created**: 2025-12-08 | **Feature**: 003-isaac-brain | **Branch**: 003-isaac-brain
