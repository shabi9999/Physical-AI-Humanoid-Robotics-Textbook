# Module 4 Task Breakdown: Vision-Language-Action (VLA) Pipeline

**Feature**: 004-vla-pipeline | **Status**: Task Generation Complete | **Date**: 2025-12-08

**Specification**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md) | **Branch**: `004-vla-pipeline`

---

## Task Summary

**Total Tasks**: 77 executable tasks across 9 implementation phases

| Phase | Name | Tasks | Dependency | Status |
|-------|------|-------|-----------|--------|
| **0** | Research & Documentation | 8 | None | **Phase Gate** |
| **1** | Foundation Setup | 6 | Phase 0 | **Phase Gate** |
| **2** | Chapter 1 Structure & Core Content (Whisper - U1) | 11 | Phase 1 | **Phase Gate** |
| **3** | Chapter 1 Completion (Whisper Diagrams & Integration) | 6 | Phase 2 | Sequential |
| **4** | Chapter 2 Structure & Core Content (LLM Planning - U2) | 11 | Phase 1 | **Phase Gate** |
| **5** | Chapter 2 Completion (LLM Diagrams & Integration) | 6 | Phase 4 | Sequential |
| **6** | Chapter 3 Structure & Core Content (ROS 2 Actions - U3) | 11 | Phase 1 | **Phase Gate** |
| **7** | Chapter 3 Completion (ROS 2 Diagrams & Integration) | 6 | Phase 6 | Sequential |
| **8** | Chapter 4 & Docusaurus Integration (Complete VLA - U4) | 6 | Phases 3,5,7 | **Phase Gate** |
| **9** | Deployment & RAG Integration | 6 | Phase 8 | Final |

---

## Phase 0: Research & Documentation (8 tasks)

**Duration**: 1-2 days | **Deliverable**: research.md | **Gate**: All tasks must PASS

### Documentation Verification Tasks

- [ ] **[P0.1]** [U0] Verify OpenAI Whisper capabilities against official documentation
  - **File**: `specs/004-vla-pipeline/research.md` (section: Whisper Verification)
  - **Acceptance**: Document multilingual support, noise robustness, diarization; cite OpenAI docs URL
  - **Resources**: https://github.com/openai/whisper, https://openai.com/research/whisper/

- [ ] **[P0.2]** [U0] Verify LLM/GPT API documentation and prompting best practices
  - **File**: `specs/004-vla-pipeline/research.md` (section: LLM Verification)
  - **Acceptance**: Document API capabilities, structured outputs, context windows, token costs; cite OpenAI docs
  - **Resources**: https://platform.openai.com/docs, https://github.com/openai/gpt-best-practices

- [ ] **[P0.3]** [U0] Verify ROS 2 Action Server documentation and lifecycle
  - **File**: `specs/004-vla-pipeline/research.md` (section: ROS 2 Verification)
  - **Acceptance**: Document goal → execute → feedback → result flow; cite ROS 2 docs
  - **Resources**: https://docs.ros.org/en/humble/Concepts/Intermediate/Tutorials/Understanding-ROS2-Actions.html

- [ ] **[P0.4]** [U0] Map Module 1 integration points (nodes, topics, services, Python agents)
  - **File**: `specs/004-vla-pipeline/research.md` (section: Module 1 Integration)
  - **Acceptance**: List 5+ integration points (voice input nodes, ROS 2 Python agents, action clients); cite Module 1 chapters
  - **Resources**: `my-website/docs/module1/`, Module 1 spec

- [ ] **[P0.5]** [U0] Map Module 3 integration points (Isaac Sim, VSLAM, Nav2)
  - **File**: `specs/004-vla-pipeline/research.md` (section: Module 3 Integration)
  - **Acceptance**: List 5+ integration points (synthetic data, VSLAM localization, Nav2 path planning); cite Module 3 chapters
  - **Resources**: `my-website/docs/module3/`, Module 3 spec

- [ ] **[P0.6]** [U0] Identify 5 diagram types and create concrete examples per chapter
  - **File**: `specs/004-vla-pipeline/research.md` (section: Diagram Strategy)
  - **Acceptance**: Define Types A-E (Architecture, Workflow, Tables, Trees, Narrative); provide 2 examples each
  - **Examples**:
    - Type A: Audio→Whisper→Text flow diagram
    - Type B: Step-by-step Whisper processing workflow
    - Type C: Whisper capabilities comparison table
    - Type D: Speech recognition error decision tree
    - Type E: "Why Whisper matters" narrative explanation

- [ ] **[P0.7]** [U0] Verify 12 VLA terminology definitions against official sources
  - **File**: `specs/004-vla-pipeline/research.md` (section: Terminology Verification)
  - **Acceptance**: All 12 terms (Speech Recognition, Whisper, Intent, Entity, Semantic Understanding, LLM, Prompt, Structured Plan, Action Server, Trajectory, Feedback Loop, VLA) verified with citations
  - **Terms**: From spec.md lines 158-172

- [ ] **[P0.8]** [U0] Identify 4+ real-world humanoid robot scenarios (household, warehouse, interactive, search & rescue)
  - **File**: `specs/004-vla-pipeline/research.md` (section: Real-World Applications)
  - **Acceptance**: 4 detailed scenarios per chapter showing voice→action flow
  - **Examples**: "Pick up the blue object on the kitchen table", "Navigate to the conference room", "Respond to voice commands"

**Phase 0 Gate Criteria**:
- ✅ All documentation sources verified (Whisper, LLM, ROS 2)
- ✅ 3-5 cross-linking references per chapter identified
- ✅ 5 diagram types defined with examples
- ✅ All 12 VLA terms sourced and verified

---

## Phase 1: Foundation Setup (6 tasks)

**Duration**: 1-2 days | **Dependency**: Phase 0 PASS | **Deliverable**: data-model.md, quickstart.md, contracts/

### Structure & Metadata Design Tasks

- [ ] **[P1.1]** [U0] Design YAML frontmatter schema (14 fields) for chapter metadata
  - **File**: `specs/004-vla-pipeline/data-model.md` (section: Chapter Metadata Schema)
  - **Acceptance**: Document all 14 fields (title, module, chapter, id, learning_objectives, prerequisites, related_chapters, keywords, difficulty, estimated_reading_time, estimated_word_count, created_at, chunk_count, searchable_terms) with examples
  - **Example**:
    ```yaml
    title: "Speech Recognition with Whisper"
    module: 4
    chapter: 1
    id: "ch1-whisper"
    learning_objectives: ["Explain Whisper role", "Understand capabilities", "Recognize limitations"]
    prerequisites: ["Module 1 completed"]
    ```

- [ ] **[P1.2]** [U0] Define chapter section structure (5 sections + edge cases) with word allocation
  - **File**: `specs/004-vla-pipeline/data-model.md` (section: Content Section Structure)
  - **Acceptance**: Define all 6 sections with word targets:
    - Section 1: Core Concepts (1,000-1,200 words)
    - Section 2: Architecture & Workflow (1,000-1,200 words)
    - Section 3: Real-World Applications (1,200-1,500 words)
    - Section 4: Module Integration (800-1,000 words)
    - Section 5: Key Takeaways (600-800 words)
    - Edge Cases & Limitations (300-500 words)
    - **Total**: 4,500-5,500 words per chapter

- [ ] **[P1.3]** [U0] Define RAG chunking metadata schema (chunk boundaries, keywords, objectives)
  - **File**: `specs/004-vla-pipeline/data-model.md` (section: RAG Metadata Schema)
  - **Acceptance**: Define metadata per chunk (chunk_id, module, chapter, section, subsection, token_count, keywords, learning_objectives, related_modules, url, content)
  - **Chunk Strategy**: 512 ± 100 tokens, 20% overlap, prefer section boundaries for splits

- [ ] **[P1.4]** [U0] Design URL structure pattern and cross-linking guidelines
  - **File**: `specs/004-vla-pipeline/data-model.md` (section: URL Structure & Cross-Linking)
  - **Acceptance**:
    - URL Pattern: `/docs/module4/chapter-slug#section-slug` (examples: `/docs/module4/chapter1-whisper#core-concepts`)
    - Cross-linking: 3-5 Module 1 refs per chapter, 1-6 Module 3 refs per chapter
    - Intra-module links between chapters

- [ ] **[P1.5]** [U0] Create chapter structure contracts (20 acceptance criteria: 5 per chapter)
  - **File**: `specs/004-vla-pipeline/contracts/chapter-structure.md`
  - **Acceptance**:
    - **Chapter 1 Contract** (AC-1.1 to AC-1.5): Whisper definitions, diagrams, scenarios, scope
    - **Chapter 2 Contract** (AC-2.1 to AC-2.5): LLM definitions, diagrams, scenarios, scope
    - **Chapter 3 Contract** (AC-3.1 to AC-3.5): ROS 2 definitions, diagrams, scenarios, scope
    - **Chapter 4 Contract** (AC-4.1 to AC-4.5): Integration, diagrams, feedback loops, wrap-up

- [ ] **[P1.6]** [U0] Design learning path and navigation guide (quickstart.md)
  - **File**: `specs/004-vla-pipeline/quickstart.md`
  - **Acceptance**:
    - Welcome section with 4 key questions
    - Prerequisites review (6 Module 1 concepts + optional Module 3)
    - 4-chapter learning journey (60-70 min total)
    - 12-term glossary
    - 3 navigation options (sequential, topic-based, perception-focused)
    - FAQ section (5+ questions)
    - Active learning suggestions

**Phase 1 Gate Criteria**:
- ✅ YAML schema: 14 fields defined with examples
- ✅ Section structure: 5 sections + edge cases with word allocation
- ✅ RAG metadata: Chunk boundaries, keywords, objectives defined
- ✅ 20 acceptance criteria: Mapped to 4 user stories in contracts/
- ✅ Cross-linking strategy: 3-5 Module 1/3 refs per chapter documented

---

## Phase 2: Chapter 1 Structure & Core Content (Whisper - U1) (11 tasks)

**Duration**: 2-3 days | **Dependency**: Phase 1 PASS | **User Story**: U1 | **Priority**: P1

**Deliverable**: chapter1-whisper.md (~5,000 words) | **Acceptance Criteria**: AC-1.1 through AC-1.5

### Section 1: Core Concepts (1,000-1,200 words)

- [ ] **[P2.1]** [U1] Write Section 1.1: What is speech recognition? Why does it matter?
  - **File**: `my-website/docs/module4/chapter1-whisper.md` (lines 1-50)
  - **Acceptance**:
    - Define "Speech Recognition" clearly on first use (AC-1.1)
    - Explain why it's VLA pipeline entry point (200-300 words)
    - Real-world context: voice commands in humanoid robotics
    - Flesch-Kincaid target: 10-12 (verify with tool)

- [ ] **[P2.2]** [U1] Write Section 1.2: What is Whisper? How does it work?
  - **File**: `my-website/docs/module4/chapter1-whisper.md` (lines 51-150)
  - **Acceptance**:
    - Define "Whisper" (OpenAI model) clearly (AC-1.1)
    - Conceptual architecture (no training details)
    - Multilingual support, noise robustness, diarization (FR-003)
    - Distinguish from semantic understanding (FR-004)
    - 400-500 words

- [ ] **[P2.3]** [U1] Write Section 1.3: Why isn't transcription enough?
  - **File**: `my-website/docs/module4/chapter1-whisper.md` (lines 151-250)
  - **Acceptance**:
    - Explain why Whisper output needs further processing (AC-1.2)
    - Ambiguity examples: "book a table" (restaurant vs furniture)
    - Context examples: "move it" (move what?)
    - Connection to Chapter 2 (LLM) (300-400 words)

### Section 2: Architecture & Workflow (1,000-1,200 words)

- [ ] **[P2.4]** [U1] Create Section 2.1: Whisper architecture diagram (Type A: Architecture)
  - **File**: `my-website/docs/module4/chapter1-whisper.md` (lines 251-350)
  - **Acceptance**:
    - Mermaid diagram: Audio Input → Feature Extraction → Model Processing → Text Output (AC-1.3)
    - ASCII alternative description
    - Explanation of each component (200-300 words)
    - No training details (scope boundary)

- [ ] **[P2.5]** [U1] Create Section 2.2: Whisper workflow diagram (Type B: Workflow)
  - **File**: `my-website/docs/module4/chapter1-whisper.md` (lines 351-450)
  - **Acceptance**:
    - Step-by-step workflow: Audio File → Whisper → Transcription → Text (AC-1.3)
    - Include timing (real-time vs batch processing)
    - Confidence scores and multiple hypotheses
    - 250-350 words

- [ ] **[P2.6]** [U1] Write Section 2.3: Whisper role in VLA pipeline
  - **File**: `my-website/docs/module4/chapter1-whisper.md` (lines 451-550)
  - **Acceptance**:
    - Position Whisper in complete VLA flow (Voice → Text → Understanding → Planning → Action)
    - Connection to Module 1: voice input nodes in ROS 2
    - Next step preview: Chapter 2 (LLM Planning)
    - 300-400 words

### Section 3: Real-World Applications (1,200-1,500 words)

- [ ] **[P2.7]** [U1] Write Section 3.1: Voice commands in humanoid robotics
  - **File**: `my-website/docs/module4/chapter1-whisper.md` (lines 551-700)
  - **Acceptance**:
    - 4 scenarios minimum (household, warehouse, interactive, search & rescue) (AC-1.4)
    - Example 1: "Pick up the blue object" (kitchen robot)
    - Example 2: "Navigate to conference room" (navigation robot)
    - Example 3: "What's on the table?" (perception robot)
    - Example 4: "Help! Call emergency" (rescue scenario)
    - 400-500 words, real-world context

- [ ] **[P2.8]** [U1] Write Section 3.2: Handling difficult scenarios (noise, accents, languages)
  - **File**: `my-website/docs/module4/chapter1-whisper.md` (lines 701-850)
  - **Acceptance**:
    - Real-world challenges (AC-1.4): Background noise, accents, multiple languages, similar-sounding words (FR-006)
    - 5+ examples: "Pick it up" vs "Picket up" (noise), "aluminum" (regional accents)
    - Multilingual support: "Bonjour", "Hola", "你好" examples
    - Scope boundary: Whisper as service, not custom training (AC-1.5)
    - 400-500 words

- [ ] **[P2.9]** [U1] Write Section 3.3: Accessibility and transcription use cases
  - **File**: `my-website/docs/module4/chapter1-whisper.md` (lines 851-950)
  - **Acceptance**:
    - Real-world applications (FR-005): Voice commands, meeting transcription, accessibility
    - Accessibility: Users with mobility limitations
    - Transcription: Meeting notes, podcast subtitles
    - 300-400 words

### Section 4: Integration with ROS 2 (800-1,000 words)

- [ ] **[P2.10]** [U1] Write Section 4.1: Connecting Whisper to ROS 2 nodes
  - **File**: `my-website/docs/module4/chapter1-whisper.md` (lines 951-1050)
  - **Acceptance**:
    - Reference Module 1 concepts: nodes, topics, services (3-5 cross-links) (AC-1.3, AC-1.4)
    - Voice input node → Whisper service → Text output topic
    - ROS 2 Python example (pseudocode, not implementation):
      ```python
      # Pseudocode only - no real implementation
      # ros2 run audio_input whisper_node --input /microphone --output /transcription
      ```
    - Message types: audio bytes → String message
    - 300-400 words

- [ ] **[P2.11]** [U1] Write Section 5: Key Takeaways & Edge Cases
  - **File**: `my-website/docs/module4/chapter1-whisper.md` (lines 1051-1200)
  - **Acceptance**:
    - Key Takeaway 1: Whisper converts voice to text reliably (AC-1.1)
    - Key Takeaway 2: Transcription alone isn't understanding (AC-1.2)
    - Key Takeaway 3: It's the first step in VLA pipeline (AC-1.3)
    - Edge cases (FR-006): Noise, accents, multilingual, scope boundary (AC-1.5)
    - Looking ahead: Chapter 2 will add semantic understanding
    - Total: 600-800 words (AC-1.5: scope boundary explicitly stated)

**Phase 2 Gate Criteria**:
- ✅ Section 1 complete (1,000-1,200 words): Core concepts
- ✅ Section 2 complete (1,000-1,200 words): Architecture & workflow diagrams
- ✅ Section 3 complete (1,200-1,500 words): Real-world scenarios
- ✅ Section 4 complete (800-1,000 words): ROS 2 integration
- ✅ Section 5 complete (600-800 words): Takeaways & edge cases
- ✅ Total: 4,500-5,500 words
- ✅ All AC-1.1 through AC-1.5 satisfied

---

## Phase 3: Chapter 1 Completion (Whisper Diagrams & Integration) (6 tasks)

**Duration**: 1-2 days | **Dependency**: Phase 2 PASS | **User Story**: U1

### Diagrams & Cross-Linking Tasks

- [ ] **[P3.1]** [U1] Create 3-5 diagrams for Chapter 1 (Mermaid + ASCII)
  - **File**: `my-website/docs/module4/chapter1-whisper.md`
  - **Acceptance**:
    - Type A (Architecture): Audio processing pipeline (Mermaid)
    - Type B (Workflow): Whisper step-by-step (Mermaid)
    - Type C (Table): Whisper capabilities (languages, latency, accuracy)
    - Type D (Tree): Speech recognition error decision tree (when does Whisper fail?)
    - Type E (Narrative): "Why Whisper matters" story (text + ASCII)
    - All diagrams verified readable in markdown

- [ ] **[P3.2]** [U1] Add 3-5 cross-links to Module 1 (ROS 2 fundamentals)
  - **File**: `my-website/docs/module4/chapter1-whisper.md`
  - **Acceptance**:
    - Link 1: Module 1 Ch1 (nodes concept) → voice input nodes
    - Link 2: Module 1 Ch1 (topics) → audio topic, text topic
    - Link 3: Module 1 Ch2 (Python agents) → Whisper service integration
    - Link 4: Module 1 actions (optional) → action server for Whisper
    - Link 5: Module 1 URDF (optional) → robot configuration for audio I/O
    - URL pattern: `[Module 1: ROS 2 Nodes](/docs/module1/chapter1-ros2-fundamentals#nodes)`

- [ ] **[P3.3]** [U1] Add 1-2 optional cross-links to Module 3 (Isaac Sim audio simulation)
  - **File**: `my-website/docs/module4/chapter1-whisper.md`
  - **Acceptance**:
    - Link: Module 3 Ch1 (Isaac Sim) → audio simulation for testing Whisper
    - Notes: Optional for full VLA context but not required

- [ ] **[P3.4]** [U1] Verify Flesch-Kincaid readability (target 10-12)
  - **File**: `my-website/docs/module4/chapter1-whisper.md`
  - **Acceptance**:
    - Run readability tool (https://www.hemingwayapp.com or similar)
    - Target: Flesch-Kincaid Grade 10-12
    - Sentence avg: 12-15 words
    - Paragraph avg: 100-300 words
    - Document results in commit message

- [ ] **[P3.5]** [U1] Add YAML frontmatter with metadata (14 fields)
  - **File**: `my-website/docs/module4/chapter1-whisper.md` (top of file)
  - **Acceptance**:
    ```yaml
    ---
    title: "Speech Recognition with Whisper"
    module: 4
    chapter: 1
    id: "ch1-whisper"
    learning_objectives:
      - "Explain Whisper's role in VLA pipeline"
      - "Understand multilingual speech recognition capabilities"
      - "Recognize limitations of transcription-only approaches"
    prerequisites: ["Module 1 completed"]
    related_chapters: ["chapter2-llm-planning", "chapter4-complete-vla"]
    keywords: ["speech recognition", "Whisper", "audio transcription", "VLA"]
    difficulty: "Beginner"
    estimated_reading_time: "15 minutes"
    estimated_word_count: 5000
    created_at: "2025-12-08"
    chunk_count: 10
    searchable_terms: ["speech", "audio", "Whisper", "transcription", "VLA entry point"]
    ---
    ```

- [ ] **[P3.6]** [U1] Validate Chapter 1 against AC-1.1 through AC-1.5 (checklist)
  - **File**: `specs/004-vla-pipeline/contracts/chapter-structure.md` (add verification section)
  - **Acceptance**:
    - [ ] AC-1.1: "Speech Recognition" and "Whisper" defined clearly ✓
    - [ ] AC-1.2: Why Whisper output needs further processing explained ✓
    - [ ] AC-1.3: Workflow diagram (Audio → Whisper → Text) included ✓
    - [ ] AC-1.4: Real-world scenarios (noise, accents, languages) covered ✓
    - [ ] AC-1.5: Scope boundary (Whisper as service, not training) stated ✓

**Phase 3 Completion**: Chapter 1 fully written, diagrammed, cross-linked, and verified

---

## Phase 4: Chapter 2 Structure & Core Content (LLM Planning - U2) (11 tasks)

**Duration**: 2-3 days | **Dependency**: Phase 1 PASS | **User Story**: U2 | **Priority**: P1

**Deliverable**: chapter2-llm-planning.md (~5,000 words) | **Acceptance Criteria**: AC-2.1 through AC-2.5

### Section 1: Core Concepts (1,000-1,200 words)

- [ ] **[P4.1]** [U2] Write Section 1.1: What are LLMs? Why robotics?
  - **File**: `my-website/docs/module4/chapter2-llm-planning.md` (lines 1-50)
  - **Acceptance**:
    - Define "LLM" clearly without deep learning math (AC-2.2) (FR-007)
    - Explain robotics use case: text → structured robot plans
    - "Large Language Model" terminology verified against OpenAI
    - Flesch-Kincaid target: 10-12
    - 300-400 words

- [ ] **[P4.2]** [U2] Write Section 1.2: Intent, entities, semantic understanding
  - **File**: `my-website/docs/module4/chapter2-llm-planning.md` (lines 51-150)
  - **Acceptance**:
    - Define "Intent" (user's goal) (AC-2.1) (FR-009)
    - Define "Entity" (objects/targets) (AC-2.1) (FR-009)
    - Define "Semantic Understanding" (meaning extraction) (AC-2.1) (FR-011)
    - Examples: "Pick up the blue object" → Intent: pick_up, Entity: blue object
    - 300-400 words

- [ ] **[P4.3]** [U2] Write Section 1.3: Prompts and structured output
  - **File**: `my-website/docs/module4/chapter2-llm-planning.md` (lines 151-250)
  - **Acceptance**:
    - Define "Prompt" (input guiding LLM) (AC-2.1)
    - Define "Token" (smallest LLM unit) (AC-2.1)
    - Explain how prompts guide outputs toward robot-compatible structure (FR-008)
    - Example prompt: "Convert 'pick up the blue ball' to {action: ???, object: ???}"
    - 300-400 words

### Section 2: Architecture & Workflow (1,000-1,200 words)

- [ ] **[P4.4]** [U2] Create Section 2.1: LLM role in VLA pipeline (Type A: Architecture)
  - **File**: `my-website/docs/module4/chapter2-llm-planning.md` (lines 251-350)
  - **Acceptance**:
    - Mermaid diagram: Text Input → LLM → Intent/Entity/Plan Output (AC-2.3)
    - Position between Speech (Ch1) and Action (Ch3)
    - Explain how LLM receives Whisper output, produces structured plan
    - 250-350 words

- [ ] **[P4.5]** [U2] Create Section 2.2: Prompting workflow (Type B: Workflow)
  - **File**: `my-website/docs/module4/chapter2-llm-planning.md` (lines 351-450)
  - **Acceptance**:
    - Step-by-step: Transcribed Text → Prompt Assembly → LLM Processing → Structured Plan (AC-2.3)
    - Include: prompt engineering, few-shot examples, output validation
    - Real prompt example: "You are a robot command planner. Convert user input to action format..."
    - 300-400 words

- [ ] **[P4.6]** [U2] Write Section 2.3: Intent extraction and constraint handling
  - **File**: `my-website/docs/module4/chapter2-llm-planning.md` (lines 451-550)
  - **Acceptance**:
    - How LLM extracts intent from ambiguous language (FR-009)
    - Constraint examples: location, timing, prerequisites
    - Example: "Put the red ball on the table next to the window" → Intent: place, Object: red ball, Target: table (specific), Constraint: near window
    - Disambiguation: How LLM handles "Move it" (asks for clarification or uses context)
    - 300-400 words

### Section 3: Real-World Applications (1,200-1,500 words)

- [ ] **[P4.7]** [U2] Write Section 3.1: Handling ambiguous and multi-step commands
  - **File**: `my-website/docs/module4/chapter2-llm-planning.md` (lines 551-700)
  - **Acceptance**:
    - 4+ scenarios (AC-2.4): ambiguous, multi-step, constraints
    - Ambiguous: "Show me the red one" (which red object?)
    - Multi-step: "Pick up the blue ball, bring it to the kitchen, place it on the counter"
    - Constraints: "Move slowly to avoid breaking the vase"
    - Context: Robot memory and environment state
    - 400-500 words

- [ ] **[P4.8]** [U2] Write Section 3.2: LLM limitations and edge cases
  - **File**: `my-website/docs/module4/chapter2-llm-planning.md` (lines 701-850)
  - **Acceptance**:
    - LLM limitations (FR-012): Hallucinations, out-of-domain, ambiguity
    - Hallucination example: "Get the xyz object" (doesn't exist) → LLM invents properties
    - Out-of-domain: "Write me a poem" (robot can't do this)
    - Scope boundary: LLMs as APIs, not fine-tuned in-house (AC-2.5)
    - 400-500 words

- [ ] **[P4.9]** [U2] Write Section 3.3: Real-world humanoid scenarios
  - **File**: `my-website/docs/module4/chapter2-llm-planning.md` (lines 851-950)
  - **Acceptance**:
    - 4+ humanoid scenarios: "Organize the shelf", "Set the table", "Fetch items from multiple rooms"
    - Complex command: "Help me prepare for a meeting by setting up the conference room" (multi-step, hierarchical)
    - Reasoning: How LLM breaks down complex goals
    - 300-400 words

### Section 4: Integration with ROS 2 (800-1,000 words)

- [ ] **[P4.10]** [U2] Write Section 4.1: Mapping LLM outputs to ROS 2 actions
  - **File**: `my-website/docs/module4/chapter2-llm-planning.md` (lines 951-1050)
  - **Acceptance**:
    - Reference Module 1 concepts: Python agents, action clients (3-5 cross-links) (AC-2.1)
    - Example mapping: LLM output → ROS 2 Action Goal
    - Message types: Structured JSON → ROS 2 Custom Messages
    - Pseudocode (no real implementation):
      ```python
      # LLM output → ROS 2 Action Goal
      # "pick_up" intent → PickUpAction(object="blue_ball")
      ```
    - 300-400 words

- [ ] **[P4.11]** [U2] Write Section 5: Key Takeaways & Edge Cases
  - **File**: `my-website/docs/module4/chapter2-llm-planning.md` (lines 1051-1200)
  - **Acceptance**:
    - Key Takeaway 1: LLMs convert text to structured robot plans (AC-2.2)
    - Key Takeaway 2: Prompts guide LLM outputs (AC-2.2)
    - Key Takeaway 3: Semantic understanding requires intent + entities + constraints (AC-2.4)
    - Edge cases: Ambiguity, hallucinations, out-of-domain (FR-012)
    - Scope: API-based, not fine-tuned (AC-2.5)
    - Looking ahead: Chapter 3 executes these plans
    - Total: 600-800 words

**Phase 4 Gate Criteria**:
- ✅ Section 1 complete (1,000-1,200 words): Core concepts
- ✅ Section 2 complete (1,000-1,200 words): Architecture & workflow diagrams
- ✅ Section 3 complete (1,200-1,500 words): Real-world scenarios
- ✅ Section 4 complete (800-1,000 words): ROS 2 integration
- ✅ Section 5 complete (600-800 words): Takeaways & edge cases
- ✅ Total: 4,500-5,500 words
- ✅ All AC-2.1 through AC-2.5 satisfied

---

## Phase 5: Chapter 2 Completion (LLM Diagrams & Integration) (6 tasks)

**Duration**: 1-2 days | **Dependency**: Phase 4 PASS | **User Story**: U2

### Diagrams & Cross-Linking Tasks

- [ ] **[P5.1]** [U2] Create 3-5 diagrams for Chapter 2 (Mermaid + ASCII)
  - **File**: `my-website/docs/module4/chapter2-llm-planning.md`
  - **Acceptance**:
    - Type A (Architecture): Text → LLM → Structured Plan (Mermaid)
    - Type B (Workflow): Prompting step-by-step (Mermaid)
    - Type C (Table): Intent vs Entity vs Constraints comparison
    - Type D (Tree): LLM decision paths for ambiguous inputs
    - Type E (Narrative): "Why LLMs enable robot understanding" story

- [ ] **[P5.2]** [U2] Add 3-5 cross-links to Module 1 (Python agents, ROS 2 integration)
  - **File**: `my-website/docs/module4/chapter2-llm-planning.md`
  - **Acceptance**:
    - Link 1: Module 1 Ch2 (Python agents) → LLM as agent decision maker
    - Link 2: Module 1 Ch1 (topics) → LLM output as topic message
    - Link 3: Module 1 services → LLM planning service
    - Link 4: Module 1 actions → action goal from LLM output
    - Link 5: Module 1 URDF → understanding robot capabilities for planning

- [ ] **[P5.3]** [U2] Add 2-3 cross-links to Module 3 (VSLAM for context)
  - **File**: `my-website/docs/module4/chapter2-llm-planning.md`
  - **Acceptance**:
    - Link 1: Module 3 Ch3 (VSLAM) → robot localization context for planning
    - Link 2: Module 3 Ch4 (Nav2) → path constraints from navigation

- [ ] **[P5.4]** [U2] Verify Flesch-Kincaid readability (target 10-12)
  - **File**: `my-website/docs/module4/chapter2-llm-planning.md`
  - **Acceptance**:
    - Readability tool verification: Grade 10-12
    - Sentence avg: 12-15 words
    - Paragraph avg: 100-300 words
    - Document results in commit message

- [ ] **[P5.5]** [U2] Add YAML frontmatter with metadata (14 fields)
  - **File**: `my-website/docs/module4/chapter2-llm-planning.md` (top of file)
  - **Acceptance**:
    ```yaml
    ---
    title: "LLM Cognitive Planning"
    module: 4
    chapter: 2
    id: "ch2-llm-planning"
    learning_objectives:
      - "Understand how LLMs convert text to robot plans"
      - "Recognize intent, entity, and constraint extraction"
      - "Apply prompting techniques for robotics"
    prerequisites: ["Module 1 completed", "Chapter 1 completed"]
    related_chapters: ["chapter1-whisper", "chapter3-ros2-actions", "chapter4-complete-vla"]
    keywords: ["LLM", "intent recognition", "semantic understanding", "planning", "VLA"]
    difficulty: "Beginner"
    estimated_reading_time: "15 minutes"
    estimated_word_count: 5000
    created_at: "2025-12-08"
    chunk_count: 10
    searchable_terms: ["LLM", "intent", "entity", "semantic", "planning", "prompt"]
    ---
    ```

- [ ] **[P5.6]** [U2] Validate Chapter 2 against AC-2.1 through AC-2.5 (checklist)
  - **File**: `specs/004-vla-pipeline/contracts/chapter-structure.md` (update verification section)
  - **Acceptance**:
    - [ ] AC-2.1: Intent, Entity, Semantic Understanding, Prompt defined clearly ✓
    - [ ] AC-2.2: LLM role explained without ML math ✓
    - [ ] AC-2.3: Workflow diagram (Text → LLM → Plan) included ✓
    - [ ] AC-2.4: Real-world scenarios (ambiguous, multi-step, constraints) covered ✓
    - [ ] AC-2.5: Scope boundary (APIs, not fine-tuned) stated ✓

**Phase 5 Completion**: Chapter 2 fully written, diagrammed, cross-linked, and verified

---

## Phase 6: Chapter 3 Structure & Core Content (ROS 2 Actions - U3) (11 tasks)

**Duration**: 2-3 days | **Dependency**: Phase 1 PASS | **User Story**: U3 | **Priority**: P2

**Deliverable**: chapter3-ros2-actions.md (~5,000 words) | **Acceptance Criteria**: AC-3.1 through AC-3.5

### Section 1: Core Concepts (1,000-1,200 words)

- [ ] **[P6.1]** [U3] Write Section 1.1: What are ROS 2 Action Servers? Why essential?
  - **File**: `my-website/docs/module4/chapter3-ros2-actions.md` (lines 1-50)
  - **Acceptance**:
    - Define "Action Server" clearly (AC-3.1)
    - Why essential: Goal-oriented tasks with feedback
    - Distinguish from topics (streaming) and services (request-response) (FR-013)
    - Robotics motivation: Long-running tasks that can be monitored
    - 300-400 words

- [ ] **[P6.2]** [U3] Write Section 1.2: Action lifecycle and components
  - **File**: `my-website/docs/module4/chapter3-ros2-actions.md` (lines 51-150)
  - **Acceptance**:
    - Define "Goal" (what the action should do) (AC-3.1)
    - Define "Feedback" (progress updates) (AC-3.1)
    - Define "Result" (final outcome) (AC-3.1)
    - Define "Trajectory" (path/waypoints for motion) (AC-3.1) (FR-015)
    - Lifecycle: Goal Sent → Accepted → Executing → Feedback (periodic) → Result (final)
    - 300-400 words

- [ ] **[P6.3]** [U3] Write Section 1.3: Structured plans to action calls
  - **File**: `my-website/docs/module4/chapter3-ros2-actions.md` (lines 151-250)
  - **Acceptance**:
    - How structured plans (from Chapter 2) map to ROS 2 action calls (FR-016)
    - Example: {action: pick_up, object: blue_ball} → PickUpAction(target_object_id=123)
    - Message format translation
    - Reference Module 1 knowledge (AC-3.2)
    - 300-400 words

### Section 2: Architecture & Workflow (1,000-1,200 words)

- [ ] **[P6.4]** [U3] Create Section 2.1: Action Server architecture (Type A: Architecture)
  - **File**: `my-website/docs/module4/chapter3-ros2-actions.md` (lines 251-350)
  - **Acceptance**:
    - Mermaid diagram: Action Client → Action Server (Goal) → Executor → Result (AC-3.3)
    - Include feedback channel
    - Components: Client, Server, Goal, Feedback, Result, Executor
    - 250-350 words

- [ ] **[P6.5]** [U3] Create Section 2.2: Action execution workflow (Type B: Workflow)
  - **File**: `my-website/docs/module4/chapter3-ros2-actions.md` (lines 351-450)
  - **Acceptance**:
    - Step-by-step: Send Goal → Server Accepts → Execute → Periodic Feedback → Final Result (AC-3.3)
    - Timeline diagram showing async execution
    - Cancellation option
    - 300-400 words

- [ ] **[P6.6]** [U3] Write Section 2.3: Trajectory planning and execution
  - **File**: `my-website/docs/module4/chapter3-ros2-actions.md` (lines 451-550)
  - **Acceptance**:
    - Trajectory concept: sequence of waypoints, joint configurations, velocities (FR-015)
    - Example: Gripper movement trajectory: Home → Approach → Grasp → Retract
    - Real-time constraints: acceleration limits, collision avoidance
    - Inverse kinematics: Position goal → Joint angles
    - 300-400 words

### Section 3: Real-World Applications (1,200-1,500 words)

- [ ] **[P6.7]** [U3] Write Section 3.1: Multi-step actions and failure handling
  - **File**: `my-website/docs/module4/chapter3-ros2-actions.md` (lines 551-700)
  - **Acceptance**:
    - 4+ scenarios (AC-3.4): timeouts, partial failures, concurrent actions, obstacles
    - Timeout scenario: Gripper action takes too long (stalled)
    - Partial failure: Reached approach point, but grasp failed
    - Concurrent actions: Multiple robot joints moving simultaneously
    - Obstacle: Gripper detects collision during reach
    - 400-500 words

- [ ] **[P6.8]** [U3] Write Section 3.2: Real-world challenges and recovery
  - **File**: `my-website/docs/module4/chapter3-ros2-actions.md` (lines 701-850)
  - **Acceptance**:
    - Real-world challenges (FR-018): timeouts, failures, dynamic obstacles
    - Recovery strategies: Retry, alternative path, abort and recover
    - Force feedback: Gripper senses object vs empty
    - Dynamic environment: Obstacles appear during execution
    - Scope boundary: Standard ROS 2 Actions, not implementation (AC-3.5)
    - 400-500 words

- [ ] **[P6.9]** [U3] Write Section 3.3: Humanoid robot action examples
  - **File**: `my-website/docs/module4/chapter3-ros2-actions.md` (lines 851-950)
  - **Acceptance**:
    - Manipulation: Picking, placing, grasping with feedback
    - Navigation: Moving to location with obstacle avoidance
    - Interaction: Reaching human, responding to gestures
    - 300-400 words

### Section 4: Integration with Perception (800-1,000 words)

- [ ] **[P6.10]** [U3] Write Section 4.1: Perception feedback loops (VSLAM, Nav2)
  - **File**: `my-website/docs/module4/chapter3-ros2-actions.md` (lines 951-1050)
  - **Acceptance**:
    - Reference Module 3 concepts: VSLAM, Nav2 (3-4 cross-links) (AC-3.4)
    - How VSLAM feedback informs action execution: "Is robot at goal position?"
    - How Nav2 feedback provides collision avoidance during motion
    - Feedback loop: Action executing → VSLAM provides location → Adjust if off-course
    - 300-400 words

- [ ] **[P6.11]** [U3] Write Section 5: Key Takeaways & Edge Cases
  - **File**: `my-website/docs/module4/chapter3-ros2-actions.md` (lines 1051-1200)
  - **Acceptance**:
    - Key Takeaway 1: Action Servers enable long-running tasks with feedback (AC-3.2)
    - Key Takeaway 2: Trajectories sequence robot motion (AC-3.1)
    - Key Takeaway 3: Feedback loops monitor and adjust execution (AC-3.4)
    - Edge cases: Timeouts, failures, obstacles, concurrency (FR-018)
    - Scope: Standard ROS 2, usage not implementation (AC-3.5)
    - Looking ahead: Chapter 4 integrates all components
    - Total: 600-800 words

**Phase 6 Gate Criteria**:
- ✅ Section 1 complete (1,000-1,200 words): Core concepts
- ✅ Section 2 complete (1,000-1,200 words): Architecture & workflow diagrams
- ✅ Section 3 complete (1,200-1,500 words): Real-world scenarios
- ✅ Section 4 complete (800-1,000 words): Perception integration
- ✅ Section 5 complete (600-800 words): Takeaways & edge cases
- ✅ Total: 4,500-5,500 words
- ✅ All AC-3.1 through AC-3.5 satisfied

---

## Phase 7: Chapter 3 Completion (ROS 2 Diagrams & Integration) (6 tasks)

**Duration**: 1-2 days | **Dependency**: Phase 6 PASS | **User Story**: U3

### Diagrams & Cross-Linking Tasks

- [ ] **[P7.1]** [U3] Create 3-5 diagrams for Chapter 3 (Mermaid + ASCII)
  - **File**: `my-website/docs/module4/chapter3-ros2-actions.md`
  - **Acceptance**:
    - Type A (Architecture): Action Client → Server → Executor (Mermaid)
    - Type B (Workflow): Goal → Execute → Feedback → Result timeline (Mermaid)
    - Type C (Table): Action vs Topic vs Service comparison
    - Type D (Tree): Action decision paths (success, timeout, failure, cancel)
    - Type E (Narrative): "How robot arms execute plans" story

- [ ] **[P7.2]** [U3] Add 3-5 cross-links to Module 1 (ROS 2 core concepts)
  - **File**: `my-website/docs/module4/chapter3-ros2-actions.md`
  - **Acceptance**:
    - Link 1: Module 1 Ch1 (actions) → ROS 2 Action Server detail
    - Link 2: Module 1 Ch1 (topics) → feedback/result topics
    - Link 3: Module 1 Ch1 (services) → distinguish from actions
    - Link 4: Module 1 Ch2 (Python agents) → action clients
    - Link 5: Module 1 URDF → understanding robot joint structure

- [ ] **[P7.3]** [U3] Add 3-4 cross-links to Module 3 (VSLAM, Nav2 feedback)
  - **File**: `my-website/docs/module4/chapter3-ros2-actions.md`
  - **Acceptance**:
    - Link 1: Module 3 Ch3 (VSLAM) → localization feedback for verification
    - Link 2: Module 3 Ch4 (Nav2) → navigation planning before action
    - Link 3: Module 3 perception → collision detection during action

- [ ] **[P7.4]** [U3] Verify Flesch-Kincaid readability (target 10-12)
  - **File**: `my-website/docs/module4/chapter3-ros2-actions.md`
  - **Acceptance**:
    - Readability tool: Grade 10-12
    - Sentence avg: 12-15 words
    - Paragraph avg: 100-300 words

- [ ] **[P7.5]** [U3] Add YAML frontmatter with metadata (14 fields)
  - **File**: `my-website/docs/module4/chapter3-ros2-actions.md` (top of file)
  - **Acceptance**:
    ```yaml
    ---
    title: "ROS 2 Action Integration"
    module: 4
    chapter: 3
    id: "ch3-ros2-actions"
    learning_objectives:
      - "Understand ROS 2 Action Servers and lifecycle"
      - "Recognize trajectory planning and execution"
      - "Apply action feedback for robust robotics"
    prerequisites: ["Module 1 completed", "Chapter 1-2 completed"]
    related_chapters: ["chapter1-whisper", "chapter2-llm-planning", "chapter4-complete-vla"]
    keywords: ["ROS 2", "actions", "trajectory", "execution", "feedback"]
    difficulty: "Intermediate"
    estimated_reading_time: "18 minutes"
    estimated_word_count: 5000
    created_at: "2025-12-08"
    chunk_count: 10
    searchable_terms: ["action server", "goal", "feedback", "result", "trajectory"]
    ---
    ```

- [ ] **[P7.6]** [U3] Validate Chapter 3 against AC-3.1 through AC-3.5 (checklist)
  - **File**: `specs/004-vla-pipeline/contracts/chapter-structure.md` (update verification section)
  - **Acceptance**:
    - [ ] AC-3.1: Action Server, Trajectory, Goal, Feedback, Result defined ✓
    - [ ] AC-3.2: ROS 2 Actions explained with Module 1 knowledge ✓
    - [ ] AC-3.3: Workflow diagram (Plan → Action Server → Commands) included ✓
    - [ ] AC-3.4: Real-world scenarios (timeouts, failures, obstacles) covered ✓
    - [ ] AC-3.5: Scope boundary (standard usage, not implementation) stated ✓

**Phase 7 Completion**: Chapter 3 fully written, diagrammed, cross-linked, and verified

---

## Phase 8: Chapter 4 & Docusaurus Integration (Complete VLA - U4) (6 tasks)

**Duration**: 2-3 days | **Dependency**: Phases 3, 5, 7 PASS | **User Story**: U4 | **Priority**: P2

**Deliverable**: chapter4-complete-vla.md (~5,000 words) + integration | **Acceptance Criteria**: AC-4.1 through AC-4.5

### Chapter 4 Integration & Wrap-Up

- [ ] **[P8.1]** [U4] Write Chapter 4: Complete VLA Pipeline (5,000 words)
  - **File**: `my-website/docs/module4/chapter4-complete-vla.md`
  - **Acceptance**:
    - **Section 1** (1,000-1,200 words): What is complete VLA? How do parts work together? (AC-4.1)
      - Integrate Ch1-3 concepts: Voice → Text → Understanding → Planning → Action
      - Humanoid robot motivation
      - System-level view without repetition of chapters
    - **Section 2** (1,000-1,200 words): End-to-end data flow with feedback loops (FR-020) (AC-4.2)
      - Diagram: Audio → Whisper → LLM → Action Server → Motion → Perception feedback (AC-4.2)
      - Real-time constraints and latency budgets
      - Perception feedback (Module 3 VSLAM/Nav2) informing next steps (AC-4.3)
    - **Section 3** (1,200-1,500 words): Real-world complex scenarios and error recovery (AC-4.4)
      - Multi-step command with perception feedback: "Pick up the blue object on the table, bring it to the kitchen"
      - Error handling: Plan failed → Retry with adjusted approach
      - Real-time constraints: All subsystems must complete within deadline
      - 4+ complex scenarios (FR-021)
    - **Section 4** (800-1,000 words): Module 1 & 3 integration (AC-4.1)
      - How Modules 1, 3, and 4 work together: ROS 2 infrastructure, perception, VLA
      - Connecting references (3-5 cross-links per spec)
    - **Section 5** (600-800 words): Learning wrap-up and next steps (AC-4.5)
      - Summary: "You understand how humanoid robots understand and act on voice commands"
      - Advanced topics preview (multi-agent VLA, distributed systems)
      - Final diagram: Human voice → Intelligent humanoid action

- [ ] **[P8.2]** [U4] Add 5-6 diagrams to Chapter 4 (Mermaid + ASCII)
  - **File**: `my-website/docs/module4/chapter4-complete-vla.md`
  - **Acceptance**:
    - Type A (Architecture): Complete VLA pipeline end-to-end (AC-4.2)
    - Type B (Workflow): Voice command → Robot action full timeline (AC-4.2)
    - Type C (Table): Chapter 1-4 component summary
    - Type D (Tree): Error handling decision paths (AC-4.4)
    - Type E (Narrative): "The journey from voice to motion" story
    - Plus: Humanoid robot scenario diagram (AC-4.4)

- [ ] **[P8.3]** [U4] Add 5-6 cross-links to Modules 1 & 3
  - **File**: `my-website/docs/module4/chapter4-complete-vla.md`
  - **Acceptance**:
    - Module 1: ROS 2 nodes, topics, services, actions (2-3 links)
    - Module 3: Isaac Sim, VSLAM, Nav2 (2-3 links)
    - Perception feedback loop explicitly connected

- [ ] **[P8.4]** [U4] Add YAML frontmatter (14 fields) and verify readability
  - **File**: `my-website/docs/module4/chapter4-complete-vla.md` (top of file)
  - **Acceptance**:
    ```yaml
    ---
    title: "Complete VLA Pipeline"
    module: 4
    chapter: 4
    id: "ch4-complete-vla"
    learning_objectives:
      - "Trace voice commands through complete VLA system"
      - "Understand feedback loops and error recovery"
      - "Recognize VLA patterns in humanoid robotics"
    prerequisites: ["Module 1 completed", "Chapters 1-3 completed", "Module 3 optional"]
    related_chapters: ["chapter1-whisper", "chapter2-llm-planning", "chapter3-ros2-actions"]
    keywords: ["VLA", "complete pipeline", "integration", "feedback", "end-to-end"]
    difficulty: "Intermediate"
    estimated_reading_time: "20 minutes"
    estimated_word_count: 5000
    created_at: "2025-12-08"
    chunk_count: 10
    searchable_terms: ["VLA", "pipeline", "voice", "action", "perception", "feedback"]
    ---
    ```
    - Flesch-Kincaid readability: 10-12 (verified)

- [ ] **[P8.5]** [U4] Validate Chapter 4 against AC-4.1 through AC-4.5 (checklist)
  - **File**: `specs/004-vla-pipeline/contracts/chapter-structure.md` (final verification section)
  - **Acceptance**:
    - [ ] AC-4.1: Chapters 1-3 integrated into narrative ✓
    - [ ] AC-4.2: End-to-end diagram with perception loop ✓
    - [ ] AC-4.3: Perception feedback (VSLAM/Nav2) explained ✓
    - [ ] AC-4.4: Complex edge cases (partial successes, failures) covered ✓
    - [ ] AC-4.5: Module wrap-up included ✓

- [ ] **[P8.6]** [U4] Integrate chapters into Docusaurus (my-website/docs/module4/)
  - **File**: `my-website/docs/module4/`
  - **Acceptance**:
    - [ ] intro.md created (Module 4 overview, learning path)
    - [ ] chapter1-whisper.md copied to docs/
    - [ ] chapter2-llm-planning.md copied to docs/
    - [ ] chapter3-ros2-actions.md copied to docs/
    - [ ] chapter4-complete-vla.md copied to docs/
    - [ ] All files have correct YAML frontmatter
    - [ ] All cross-links formatted correctly: `[text](/docs/module1/chapter1-ros2-fundamentals#section)`

**Phase 8 Gate Criteria**:
- ✅ Chapter 4 written (5,000 words, AC-4.1 through AC-4.5)
- ✅ 5-6 diagrams created (all types represented, AC-4.2)
- ✅ 5-6 cross-links to Module 1 & 3 (AC-4.3)
- ✅ YAML frontmatter complete
- ✅ Readability verified
- ✅ All chapters integrated into Docusaurus
- ✅ All 20 acceptance criteria verified (AC-1.1 through AC-4.5)

---

## Phase 9: Deployment & RAG Integration (6 tasks)

**Duration**: 1-2 days | **Dependency**: Phase 8 PASS | **Deliverable**: Docusaurus deployment + RAG chunks

### Deployment & RAG Chunking Tasks

- [ ] **[P9.1]** [U0] Update Docusaurus sidebar navigation (sidebars.js)
  - **File**: `my-website/sidebars.js`
  - **Acceptance**:
    - Add Module 4 sidebar entry under main navigation
    - Structure:
      ```javascript
      {
        type: "doc",
        label: "Module 4: Vision-Language-Action",
        items: [
          { type: "doc", id: "module4/intro" },
          { type: "doc", id: "module4/chapter1-whisper" },
          { type: "doc", id: "module4/chapter2-llm-planning" },
          { type: "doc", id: "module4/chapter3-ros2-actions" },
          { type: "doc", id: "module4/chapter4-complete-vla" },
        ],
      }
      ```

- [ ] **[P9.2]** [U0] Verify Docusaurus rendering and cross-link functionality
  - **File**: `my-website/`
  - **Acceptance**:
    - Run: `npm run build` or `npm run serve`
    - Verify all chapters render correctly (headings, links, formatting)
    - Verify cross-links work: `/docs/module1/...`, `/docs/module3/...`
    - Check for broken link warnings
    - Screenshot/confirm module visible in sidebar

- [ ] **[P9.3]** [U0] Generate RAG chunks (512 ± 100 tokens, 20% overlap)
  - **File**: `specs/004-vla-pipeline/rag-chunks.json` (or RAG indexing script)
  - **Acceptance**:
    - Chunk each chapter into 512 ± 100 token segments
    - Use section boundaries where possible (prefer chapter/section breaks)
    - Add metadata to each chunk:
      - `chunk_id`: Unique identifier (e.g., "ch1-whisper-001")
      - `module`: 4
      - `chapter`: 1-4
      - `section`: Section name
      - `token_count`: Actual token count
      - `keywords`: Extracted or predefined
      - `learning_objectives`: Relevant objectives
      - `url`: `/docs/module4/chapter-name#section-slug`
    - Total chunks expected: ~20 (5 per chapter)

- [ ] **[P9.4]** [U0] Test semantic search with RAG chunks (Qdrant)
  - **File**: `specs/004-vla-pipeline/rag-test-results.md`
  - **Acceptance**:
    - Load chunks into Qdrant vector database
    - Test queries:
      - "How does Whisper recognize speech?" → Should find Ch1 content
      - "What are robot actions?" → Should find Ch3 content
      - "How do LLMs plan robot movements?" → Should find Ch2 content
      - "How does perception feedback work?" → Should find Ch4 content
    - Document results (search accuracy, latency)

- [ ] **[P9.5]** [U0] Deploy to GitHub Pages (or staging environment)
  - **File**: `my-website/`
  - **Acceptance**:
    - Run deployment command (specific to your setup)
    - Example: `npm run build && npm run gh-deploy` (for GitHub Pages)
    - Verify deployment: Chapters accessible at `https://<your-domain>/docs/module4/`
    - Confirm RAG chatbot can access published content (if applicable)

- [ ] **[P9.6]** [U0] Create final verification report
  - **File**: `specs/004-vla-pipeline/DEPLOYMENT_SUMMARY.md`
  - **Acceptance**:
    - [ ] All 4 chapters deployed and visible
    - [ ] Docusaurus navigation working
    - [ ] All cross-links functional (Module 1, Module 3, internal)
    - [ ] RAG chunking complete (20 chunks)
    - [ ] Semantic search tested (4+ queries)
    - [ ] 20 acceptance criteria verified: All AC-1.1 through AC-4.5 ✓
    - [ ] Readability verified: Flesch-Kincaid 10-12 per chapter ✓
    - [ ] 3-5 diagrams per chapter: All created ✓
    - [ ] RAG metadata complete: 14 fields per chapter, keywords, URLs ✓

**Phase 9 Gate Criteria**:
- ✅ Docusaurus integration complete
- ✅ Navigation working
- ✅ Cross-links functional
- ✅ RAG chunks generated and indexed
- ✅ Semantic search tested
- ✅ Deployment verified

---

## Task Dependency Graph

```
Phase 0 (Research) → Gate
    ↓
Phase 1 (Foundation) → Gate
    ↓
Phase 2 (Ch1 Core) → Phase 3 (Ch1 Complete) ─┐
Phase 4 (Ch2 Core) → Phase 5 (Ch2 Complete) ─┤
Phase 6 (Ch3 Core) → Phase 7 (Ch3 Complete) ─┤
                                              ├→ Phase 8 (Ch4 + Integration) → Gate
                                              ↓
                                         Phase 9 (Deployment) → Final
```

**Parallel Execution Opportunities**:
- Phases 2, 4, 6 can run in parallel (different user stories)
- Phases 3, 5, 7 can run in parallel after their respective phases
- Phase 8 requires completion of Phases 3, 5, 7
- Phase 9 requires Phase 8

**Critical Path**: Phase 0 → Phase 1 → Phase 2 → Phase 3 → Phase 8 → Phase 9 (or any chapter path, all converge at Phase 8)

---

## Quality Assurance Checklist

**Specification Alignment** (from spec.md):
- [ ] All 4 user stories addressed: U1 (Ph2-3), U2 (Ph4-5), U3 (Ph6-7), U4 (Ph8)
- [ ] All 23 functional requirements satisfied (FR-001 through FR-023)
- [ ] All 8 success criteria met (SC-001 through SC-008)
- [ ] All 20 acceptance criteria verified (AC-1.1 through AC-4.5)
- [ ] All 12 VLA terms used consistently
- [ ] All 7 edge cases addressed per chapter

**Content Quality**:
- [ ] 4 chapters × 5,000 words = 20,000 words total
- [ ] 5 sections per chapter + edge cases
- [ ] 3-5 diagrams per chapter (Types A-E represented)
- [ ] Flesch-Kincaid 10-12 readability per chapter
- [ ] 3-5 cross-links per chapter (Module 1 & Module 3)
- [ ] YAML frontmatter: 14 fields per chapter

**RAG Compatibility**:
- [ ] Chunking: 512 ± 100 tokens, 20% overlap
- [ ] Metadata: chapter, section, keywords, objectives, URL per chunk
- [ ] Searchability: All core VLA terms indexed
- [ ] Citation: URL and chapter location for each chunk

**Deployment**:
- [ ] Docusaurus sidebar updated
- [ ] All chapters deployed to GitHub Pages
- [ ] Cross-links verified functional
- [ ] RAG chatbot can access content
- [ ] Semantic search tested

---

## File Changes Summary

**New Files Created**:
- `my-website/docs/module4/intro.md`
- `my-website/docs/module4/chapter1-whisper.md`
- `my-website/docs/module4/chapter2-llm-planning.md`
- `my-website/docs/module4/chapter3-ros2-actions.md`
- `my-website/docs/module4/chapter4-complete-vla.md`
- `specs/004-vla-pipeline/research.md` (from Phase 0)
- `specs/004-vla-pipeline/data-model.md` (from Phase 1)
- `specs/004-vla-pipeline/contracts/chapter-structure.md` (from Phase 1)
- `specs/004-vla-pipeline/quickstart.md` (from Phase 1)
- `specs/004-vla-pipeline/rag-chunks.json` (from Phase 9)
- `specs/004-vla-pipeline/DEPLOYMENT_SUMMARY.md` (from Phase 9)

**Modified Files**:
- `my-website/sidebars.js` (add Module 4 navigation)

---

## Next Steps

### Immediate (After Task Breakdown)
1. Review tasks for completeness and clarity
2. Run `/sp.implement` to execute Phase 0-1 tasks (research, foundation setup)
3. Create PHR documenting task generation phase

### During Implementation
1. Execute tasks sequentially within phases, in parallel across phases where possible
2. Create git commits per phase milestone
3. Verify acceptance criteria at phase gates
4. Document any blockers or adaptations

### Final Verification
1. Run complete acceptance criteria check (all 20 ACs)
2. Verify RAG chunking and semantic search
3. Confirm Docusaurus deployment
4. Create final deployment summary

---

**Created**: 2025-12-08 | **Feature**: 004-vla-pipeline | **Status**: Task Breakdown Complete

**Total Tasks**: 77 | **Total Estimated Duration**: 8-12 days | **Critical Path**: Research → Foundation → Ch1 → Integration → Deployment

**Ready for Implementation**: `/sp.implement`
