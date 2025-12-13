# Module 4 Data Model: Content Structure & Metadata Schema

**Feature**: 004-vla-pipeline | **Status**: Phase 1 Design Complete | **Date**: 2025-12-08

**Specification**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md) | **Research**: [research.md](research.md) | **Branch**: `004-vla-pipeline`

---

## Executive Summary

Phase 1 defines the complete content structure, metadata schema, and technical specifications for Module 4 chapter development. This document provides writers with the exact structure, word counts, and formatting requirements for all 4 chapters.

---

## Chapter Metadata Schema (YAML Frontmatter - 14 Fields)

### Required Fields

Every chapter file (`chapter1-whisper.md`, `chapter2-llm-planning.md`, etc.) must include this YAML frontmatter:

```yaml
---
# Document Identifiers
title: "Chapter Title Here"
module: 4
chapter: 1
id: "ch1-whisper"

# Learning & Structure
learning_objectives:
  - "Learning objective 1"
  - "Learning objective 2"
  - "Learning objective 3"
prerequisites:
  - "Module 1 completed"
  - "Any other prerequisites"
related_chapters:
  - "chapter2-llm-planning"
  - "chapter3-ros2-actions"
  - "chapter4-complete-vla"
keywords:
  - "keyword1"
  - "keyword2"
  - "keyword3"

# Content Metadata
difficulty: "Beginner"
estimated_reading_time: "15 minutes"
estimated_word_count: 5000
created_at: "2025-12-08"

# RAG & Indexing
chunk_count: 10
searchable_terms:
  - "term1"
  - "term2"
  - "term3"
---
```

### Field Definitions

| Field | Type | Example | Purpose |
|-------|------|---------|---------|
| `title` | String | "Speech Recognition with Whisper" | Chapter display name |
| `module` | Integer | 4 | Module number |
| `chapter` | Integer | 1, 2, 3, or 4 | Chapter number within module |
| `id` | String | "ch1-whisper" | Unique identifier (used in URLs, cross-links) |
| `learning_objectives` | Array | ["Explain Whisper role", ...] | 3 specific learning outcomes |
| `prerequisites` | Array | ["Module 1 completed"] | Required prior knowledge |
| `related_chapters` | Array | ["chapter2-llm-planning"] | Cross-referenced chapters |
| `keywords` | Array | ["speech", "audio", "Whisper"] | Searchable keywords (3-5) |
| `difficulty` | String | "Beginner" or "Intermediate" | Target audience level |
| `estimated_reading_time` | String | "15 minutes" | Time to read chapter |
| `estimated_word_count` | Integer | 5000 | Target word count |
| `created_at` | String | "2025-12-08" | Creation date (YYYY-MM-DD) |
| `chunk_count` | Integer | 10 | Expected RAG chunks (512 ± 100 tokens) |
| `searchable_terms` | Array | ["speech", "Whisper", "VLA"] | Terms indexed for RAG chatbot |

### Chapter-Specific Metadata Examples

**Chapter 1: Speech Recognition with Whisper**
```yaml
title: "Speech Recognition with Whisper"
module: 4
chapter: 1
id: "ch1-whisper"
learning_objectives:
  - "Explain Whisper's role in the VLA pipeline"
  - "Understand multilingual speech recognition capabilities"
  - "Recognize limitations of transcription-only approaches"
prerequisites:
  - "Module 1: ROS 2 Fundamentals completed"
difficulty: "Beginner"
estimated_reading_time: "15 minutes"
estimated_word_count: 5000
chunk_count: 10
searchable_terms:
  - "speech recognition"
  - "Whisper"
  - "audio transcription"
  - "VLA entry point"
  - "multilingual support"
```

**Chapter 2: LLM Cognitive Planning**
```yaml
title: "LLM Cognitive Planning"
module: 4
chapter: 2
id: "ch2-llm-planning"
learning_objectives:
  - "Understand how LLMs convert text to robot plans"
  - "Recognize intent, entity, and constraint extraction"
  - "Apply prompting techniques for robotics"
prerequisites:
  - "Module 1: ROS 2 Fundamentals completed"
  - "Chapter 1: Speech Recognition completed"
difficulty: "Beginner"
estimated_reading_time: "15 minutes"
estimated_word_count: 5000
chunk_count: 10
searchable_terms:
  - "LLM"
  - "intent recognition"
  - "semantic understanding"
  - "planning"
  - "prompt engineering"
```

**Chapter 3: ROS 2 Action Integration**
```yaml
title: "ROS 2 Action Integration"
module: 4
chapter: 3
id: "ch3-ros2-actions"
learning_objectives:
  - "Understand ROS 2 Action Servers and lifecycle"
  - "Recognize trajectory planning and execution"
  - "Apply action feedback for robust robotics"
prerequisites:
  - "Module 1: ROS 2 Fundamentals completed"
  - "Chapters 1-2 completed"
difficulty: "Intermediate"
estimated_reading_time: "18 minutes"
estimated_word_count: 5000
chunk_count: 10
searchable_terms:
  - "ROS 2 actions"
  - "action server"
  - "trajectory planning"
  - "feedback mechanisms"
  - "goal execution"
```

**Chapter 4: Complete VLA Pipeline**
```yaml
title: "Complete VLA Pipeline"
module: 4
chapter: 4
id: "ch4-complete-vla"
learning_objectives:
  - "Trace voice commands through complete VLA system"
  - "Understand feedback loops and error recovery"
  - "Recognize VLA patterns in humanoid robotics"
prerequisites:
  - "Module 1: ROS 2 Fundamentals completed"
  - "Chapters 1-3 completed"
  - "Module 3: Perception (optional)"
difficulty: "Intermediate"
estimated_reading_time: "20 minutes"
estimated_word_count: 5000
chunk_count: 10
searchable_terms:
  - "VLA pipeline"
  - "voice to action"
  - "end-to-end system"
  - "perception feedback"
  - "humanoid robotics"
```

---

## Content Section Structure (5 Sections + Edge Cases)

### Standard Chapter Template

Every chapter follows this consistent structure with defined word allocations:

#### **Section 1: Core Concepts (1,000-1,200 words)**

**Purpose**: Introduce fundamental concepts without assuming deep technical knowledge

**Subsections**:
1. What is [Topic]? (200-300 words)
   - Definition clear on first use
   - Why it matters for robotics
   - Real-world motivation

2. How does [Topic] work? (300-400 words)
   - Conceptual overview (no implementation details)
   - Key components or steps
   - Visual descriptions preparing for diagrams

3. Why is this important for VLA? (200-300 words)
   - Connection to VLA pipeline
   - Integration with other components
   - Leading to next chapter

**Flesch-Kincaid Target**: 10-12
**Writing Style**: Conversational, no jargon without definition

---

#### **Section 2: Architecture & Workflow (1,000-1,200 words)**

**Purpose**: Explain system-level design and step-by-step processes

**Subsections**:
1. System Architecture (400-500 words)
   - Diagram Type A: Architecture diagram with explanation
   - Components and their roles
   - Data flow between components
   - Real-world constraints

2. Workflow/Process Flow (400-500 words)
   - Diagram Type B: Workflow diagram with timeline
   - Step-by-step execution
   - Decision points
   - Success vs failure paths

3. Integration Points (200-300 words)
   - How this section connects to previous chapter
   - What the next chapter builds on
   - Module 1 and Module 3 cross-links (3-5 references)

---

#### **Section 3: Real-World Applications (1,200-1,500 words)**

**Purpose**: Show practical humanoid robot scenarios that illustrate concepts

**Subsections**:
1. Application Scenario 1 (300-400 words)
   - Concrete humanoid robot example
   - Voice command or user interaction
   - VLA flow through this specific component
   - Challenges and edge cases

2. Application Scenario 2 (300-400 words)
   - Different humanoid robot context
   - More complex or constrained scenario
   - Illustrate limitations or tradeoffs

3. Edge Cases & Challenges (300-400 words)
   - Diagram Type D: Decision tree of failure modes
   - What goes wrong in real-world scenarios
   - How systems handle failures
   - Recovery strategies

4. Summary (200-300 words)
   - Key lessons from scenarios
   - Generalize to other robot systems

---

#### **Section 4: Module Integration (800-1,000 words)**

**Purpose**: Connect to Module 1 (ROS 2) and Module 3 (Perception) explicitly

**Subsections**:
1. Integration with Module 1: ROS 2 (400-500 words)
   - 3-5 explicit cross-links to Module 1 chapters
   - Example: "In Module 1, you learned about ROS 2 Topics. Here's how topics integrate with Whisper..."
   - Code examples (pseudocode only, no implementation)
   - Interface definitions (message types, service signatures)

2. Integration with Module 3: Perception (200-300 words)
   - References to VSLAM, Nav2, Isaac Sim where applicable
   - Feedback loops from perception to current component
   - Example: "In Chapter 3, you learned about VSLAM localization. Here's how robots use that location data..."
   - Not all chapters heavily reference Module 3 (Ch1-2 lighter, Ch3-4 heavier)

3. Integration with Other Chapters (200-300 words)
   - Forward references to upcoming chapters
   - Backward references to previous chapters
   - How all 4 chapters work together in VLA

---

#### **Section 5: Key Takeaways & Scope Boundary (600-800 words)**

**Purpose**: Summarize learning and clarify what's in/out of scope

**Subsections**:
1. Key Takeaway 1 (150-200 words)
   - One core insight from the chapter
   - Maps to learning objective

2. Key Takeaway 2 (150-200 words)
   - Second insight
   - Distinct from first takeaway

3. Key Takeaway 3 (150-200 words)
   - Third insight
   - Often about limitations or considerations

4. Scope Boundary: What's NOT in This Chapter (200-300 words)
   - Explicitly state what's out of scope
   - Example: "We don't train Whisper; we use it as a service"
   - Redirect to advanced resources if applicable
   - Prepare reader for what comes next

---

#### **Edge Cases & Limitations (300-500 words)**

**Purpose**: Address real-world complexity and failure modes

**Content**:
- Diagram Type D: Decision tree of failures
- Specific edge cases for this component
- How systems gracefully degrade
- Error handling strategies
- User impact of limitations

---

## Word Allocation Summary

| Section | Min | Max | Target |
|---------|-----|-----|--------|
| Section 1: Core Concepts | 1,000 | 1,200 | 1,100 |
| Section 2: Architecture & Workflow | 1,000 | 1,200 | 1,100 |
| Section 3: Real-World Applications | 1,200 | 1,500 | 1,350 |
| Section 4: Module Integration | 800 | 1,000 | 900 |
| Section 5: Key Takeaways | 600 | 800 | 700 |
| Edge Cases & Limitations | 300 | 500 | 400 |
| **TOTAL PER CHAPTER** | **4,500** | **5,500** | **5,150** |

---

## RAG Chunking Metadata Schema

### Purpose
Enable RAG chatbot to efficiently search, retrieve, and cite chapter content

### Chunking Strategy
- **Chunk Size**: 512 tokens ± 100 (acceptable range: 400-800)
- **Overlap**: 20% (102 tokens overlap between consecutive chunks)
- **Boundary Preference**: Section boundaries > subsection boundaries > arbitrary splits
- **Metadata Preservation**: Every chunk includes full metadata

### Metadata Per Chunk

```json
{
  "chunk_id": "ch1-whisper-001",
  "module": 4,
  "chapter": 1,
  "section": "Section 1: Core Concepts",
  "subsection": "What is Speech Recognition?",
  "token_count": 487,
  "keywords": ["speech recognition", "Whisper", "audio"],
  "learning_objectives": [
    "Explain Whisper's role in VLA pipeline",
    "Understand speech recognition capabilities"
  ],
  "related_modules": [
    "module1-chapter1",
    "module3-chapter1"
  ],
  "url": "/docs/module4/chapter1-whisper#what-is-speech-recognition",
  "content": "..."
}
```

### Field Definitions

| Field | Type | Example | Purpose |
|-------|------|---------|---------|
| `chunk_id` | String | "ch1-whisper-001" | Unique identifier (format: chX-topic-NNN) |
| `module` | Integer | 4 | Module number |
| `chapter` | Integer | 1-4 | Chapter number |
| `section` | String | "Section 1: Core Concepts" | Top-level section |
| `subsection` | String | "What is Speech Recognition?" | Subsection heading |
| `token_count` | Integer | 487 | Actual token count (400-800 range) |
| `keywords` | Array | ["speech", "Whisper", "audio"] | Searchable terms (3-5) |
| `learning_objectives` | Array | ["Explain Whisper role", ...] | Related learning outcomes |
| `related_modules` | Array | ["module1-chapter1"] | Cross-module references |
| `url` | String | "/docs/module4/chapter1-whisper#core-concepts" | Exact location in book |
| `content` | String | Full chunk text | Actual content to index |

### Expected Chunks Per Chapter
- **Chapter 1**: 10 chunks (5,000 words ÷ 500 avg tokens per chunk)
- **Chapter 2**: 10 chunks
- **Chapter 3**: 10 chunks
- **Chapter 4**: 10 chunks
- **Total**: ~40 chunks for Module 4

---

## URL Structure Pattern

### Format
```
/docs/module4/[chapter-slug]#[section-slug]
```

### Examples

**Chapter 1**:
- `/docs/module4/chapter1-whisper#core-concepts`
- `/docs/module4/chapter1-whisper#architecture-and-workflow`
- `/docs/module4/chapter1-whisper#real-world-applications`

**Chapter 2**:
- `/docs/module4/chapter2-llm-planning#core-concepts`
- `/docs/module4/chapter2-llm-planning#architecture-and-workflow`

**Chapter 3**:
- `/docs/module4/chapter3-ros2-actions#core-concepts`

**Chapter 4**:
- `/docs/module4/chapter4-complete-vla#core-concepts`

### Slug Generation Rules
- Convert to lowercase
- Replace spaces with hyphens
- Remove special characters
- Keep descriptive (not abbreviated)
- Match heading exactly

### Cross-Linking Format (Markdown)
```markdown
[Module 1: ROS 2 Nodes](/docs/module1/chapter1-ros2-fundamentals#ros-2-nodes)

[Chapter 2: LLM Cognitive Planning](/docs/module4/chapter2-llm-planning#core-concepts)

[Module 3: VSLAM Localization](/docs/module3/chapter3-vslam#vslam-localization)
```

---

## Cross-Linking Guidelines

### Module 1 References (ROS 2 Fundamentals)

**Chapter 1 (Whisper): 3-5 references**
- Nodes concept → voice input nodes
- Topics and Pub/Sub → audio topic, text topic
- Services → Whisper transcription service
- Example: "In Module 1, you learned about ROS 2 topics. Here's how the microphone publishes audio on a topic..."

**Chapter 2 (LLM Planning): 4-5 references**
- Python agents → orchestrating Whisper-to-LLM flow
- Action clients → sending action goals to robot
- Message types → JSON-formatted action goals
- URDF → understanding robot structure for planning
- Example: "Recall from Module 1 that action clients send goals to action servers. Here's how LLM planning outputs become action goals..."

**Chapter 3 (ROS 2 Actions): 5-7 references**
- Action server → core ROS 2 Actions concept
- Lifecycle → goal acceptance, feedback, result
- Topics and services → underlying pub/sub for actions
- Trajectory → kinematic models from URDF (Module 1)
- Example: "Module 1 introduced ROS 2 Actions conceptually. Let's deep dive into the action lifecycle..."

**Chapter 4 (Complete VLA): 5-6 references**
- All Module 1 chapters integrated
- Nodes, topics, services, actions working together
- Python agents coordinating subsystems
- URDF enabling trajectory planning
- Example: "The complete VLA pipeline uses every ROS 2 concept from Module 1..."

**Link Density**: 3-7 per chapter, naturally distributed

---

### Module 3 References (Perception/VSLAM/Nav2)

**Chapter 1 (Whisper): 1-2 optional references**
- Isaac Sim → audio simulation for testing Whisper
- Synthetic data → generating test audio
- Note: Chapter 1 is primarily about transcription; Module 3 integration is lighter

**Chapter 2 (LLM Planning): 2-3 references**
- VSLAM localization → robot knows its position for planning
- Scene understanding → what objects are visible to the robot
- Example: "VSLAM from Module 3 provides the robot's location. The LLM uses this context when planning: 'Navigate to the kitchen' makes sense only if the robot knows where the kitchen is..."

**Chapter 3 (ROS 2 Actions): 3-4 references**
- Nav2 path planning → motion planning with obstacle avoidance
- VSLAM feedback → confirming robot reached goal position
- Trajectory constraints → avoiding obstacles during execution
- Example: "While the action server executes, Module 3's Nav2 ensures the path is obstacle-free..."

**Chapter 4 (Complete VLA): 5-6 references**
- Perception feedback loop → robot sees, understands, acts, perceives result
- VSLAM informing LLM planning
- Nav2 enabling navigation actions
- Isaac Sim for VLA testing
- Example: "The perception feedback loop from Module 3 closes the loop in our VLA system. After the robot acts, VSLAM confirms success or triggers replanning..."

**Link Density**: 1-6 per chapter, naturally distributed

---

## Readability & Accessibility Standards

### Flesch-Kincaid Target: Grade 10-12

**Tools for Verification**:
- Hemingway Editor (https://www.hemingwayapp.com)
- Readability metrics in most word processors
- Manual check: Read aloud, ensure understanding

**Writing Rules**:
1. **Sentence Length**: 12-15 words average
   - Example ✓: "Whisper converts audio to text reliably, even in noisy environments."
   - Example ✗: "Due to the sophisticated nature of the underlying machine learning architecture that has been trained on a massive corpus of multilingual audio data, Whisper is able to achieve remarkably high accuracy in speech-to-text transcription tasks."

2. **Paragraph Length**: 100-300 words
   - Break longer concepts into multiple paragraphs
   - One idea per paragraph (topic sentence + supporting details)

3. **Technical Terms**: Define on first use with hyperlinks
   - Example: "Whisper, OpenAI's speech recognition model, converts..."
   - Hyperlink to glossary for detailed definition

4. **Acronyms**: Spell out on first use
   - Example: "VLA (Vision-Language-Action) is a complete system..."
   - Subsequent uses can be acronym only

5. **Active Voice**: Prefer active over passive
   - Example ✓: "Whisper converts audio to text."
   - Example ✗: "Audio is converted to text by Whisper."

6. **Examples**: Use concrete humanoid robot scenarios
   - Not: "Speech recognition has various applications."
   - Yes: "A household robot needs to understand voice commands like 'Pick up the blue ball' even with a TV running in the background. Whisper handles this."

7. **Visual Descriptions**: Prepare for accompanying diagrams
   - Example: "Think of Whisper's process like a three-stage pipeline: first, the audio is converted to spectrograms (visual representations of sound), then..."

---

## Summary Statistics

| Metric | Value |
|--------|-------|
| Chapters | 4 |
| Total Word Count | 20,000 (4,500-5,500 per chapter) |
| Sections per Chapter | 5 + edge cases |
| Diagrams per Chapter | 3-5 (Types A-E) |
| Cross-links per Chapter | 3-7 Module 1, 1-6 Module 3 |
| RAG Chunks Total | ~40 (512 ± 100 tokens) |
| YAML Metadata Fields | 14 |
| VLA Terminology Terms | 12 (consistent throughout) |
| Learning Objectives | 12 total (3 per chapter) |

---

## Phase 1 Completion Checklist

- [x] **P1.1**: YAML frontmatter schema (14 fields) defined ✓
- [x] **P1.2**: Chapter section structure (5 sections + edge cases) with word allocation defined ✓
- [x] **P1.3**: RAG chunking metadata schema defined ✓
- [x] **P1.4**: URL structure pattern and cross-linking guidelines defined ✓
- [ ] **P1.5**: Chapter structure contracts (20 ACs) - See `contracts/chapter-structure.md`
- [ ] **P1.6**: Learning path and navigation guide - See `quickstart.md`

---

**Created**: 2025-12-08 | **Feature**: 004-vla-pipeline | **Status**: Phase 1 Design (P1.1-P1.4) COMPLETE

**Next**: Create contracts and quickstart documents for Phase 1 completion
