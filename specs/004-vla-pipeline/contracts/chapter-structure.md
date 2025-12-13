# Module 4 Chapter Structure Contracts: Acceptance Criteria

**Feature**: 004-vla-pipeline | **Status**: Phase 1 Design Complete | **Date**: 2025-12-08

**Specification**: [spec.md](../spec.md) | **Data Model**: [data-model.md](../data-model.md) | **Branch**: `004-vla-pipeline`

---

## Overview

This document defines 20 testable acceptance criteria (5 per chapter × 4 chapters) that writers must satisfy when creating Module 4 chapters. Each criterion is specific, measurable, and verifiable.

---

## Contract 1: Chapter 1 — Speech Recognition with Whisper (U1/P1)

**User Story**: U1: "I want to understand how robots hear and recognize speech"
**Priority**: P1 (High)
**Expected Word Count**: 4,500-5,500 words
**Expected Reading Time**: 15 minutes
**Expected Diagrams**: 3-5 (Types A, B, C, D, E represented)

### Acceptance Criteria

#### **AC-1.1**: Defines "Speech Recognition" and "Whisper" clearly on first use
- **Definition**: Chapter must define both terms within first 300 words
- **Requirement**: Plain-language definition suitable for beginner learners
- **Verification**:
  - [ ] "Speech Recognition" defined in Section 1 (subsection 1)
  - [ ] "Whisper" defined in Section 1 (subsection 2)
  - [ ] Both definitions are beginner-friendly (no advanced ML terminology)
  - [ ] Definitions match terminology from research.md
- **Acceptance**: PASS if both terms clearly defined before technical depth

---

#### **AC-1.2**: Explains why Whisper output needs further processing
- **Definition**: Chapter must explicitly address the gap between transcription (Whisper) and understanding
- **Requirement**: Show why transcription alone is insufficient for robot action
- **Evidence**:
  - [ ] Example: "Move it" ambiguity (move what?)
  - [ ] Example: "Book a table" (restaurant vs furniture)
  - [ ] Explanation of semantic understanding need
  - [ ] Connection to Chapter 2 (LLM Planning)
- **Acceptance**: PASS if 2+ concrete ambiguity examples provided with explanation of why LLM is needed

---

#### **AC-1.3**: Includes workflow diagram (Audio → Whisper → Text)
- **Definition**: Chapter must include at least one Type A or Type B diagram showing Whisper's role
- **Requirement**: Visual representation of Whisper processing
- **Specifications**:
  - [ ] Mermaid diagram OR ASCII diagram provided
  - [ ] Shows input (audio), processing (Whisper), output (text)
  - [ ] Includes explanation (200-300 words)
  - [ ] Diagram placement in Section 2 (Architecture & Workflow)
  - [ ] Caption and reference in text
- **Acceptance**: PASS if diagram clearly shows Audio → Whisper → Text flow

---

#### **AC-1.4**: Covers real-world scenarios (noise, accents, languages)
- **Definition**: Chapter must demonstrate Whisper's practical capabilities through examples
- **Requirement**: 4+ specific scenarios showing how Whisper handles real-world challenges
- **Scenarios Required**:
  - [ ] Noisy environment (restaurant, factory)
  - [ ] Regional accent or non-native speaker
  - [ ] Multilingual scenario
  - [ ] Additional complex scenario (dual speakers, technical language, etc.)
- **Evidence**: Each scenario described with 100-200 words showing:
  - User voice command
  - Background conditions
  - How Whisper handles the challenge
  - Success/limitation
- **Acceptance**: PASS if 4+ distinct scenarios covered in Section 3

---

#### **AC-1.5**: Explicitly states scope boundary (Whisper as service, not training)
- **Definition**: Chapter must clarify what's IN scope (using Whisper) and OUT of scope (training/fine-tuning)
- **Requirement**: Clear scope statement in Section 5 (Key Takeaways)
- **Content Required**:
  - [ ] "We use Whisper as a service/API, not training custom models"
  - [ ] Reason: "Whisper is pre-trained on 680k hours of audio"
  - [ ] What readers WON'T learn: "How to fine-tune Whisper for specific accents"
  - [ ] Implication: "Accuracy varies by language and audio quality"
- **Acceptance**: PASS if scope explicitly stated (not just implied)

---

## Contract 2: Chapter 2 — LLM Cognitive Planning (U2/P1)

**User Story**: U2: "I want to understand how LLMs convert natural language into actionable robot plans"
**Priority**: P1 (High)
**Expected Word Count**: 4,500-5,500 words
**Expected Reading Time**: 15 minutes
**Expected Diagrams**: 3-5 (Types A, B, C, D, E represented)

### Acceptance Criteria

#### **AC-2.1**: Defines "Intent," "Entity," "Semantic Understanding," and "Prompt" clearly
- **Definition**: Chapter must define 4 key LLM planning terms in accessible language
- **Requirement**: Each term defined before deep technical explanation
- **Definitions**:
  - [ ] **Intent**: User's goal/action (e.g., "pick_up" from "Pick up the blue ball")
  - [ ] **Entity**: Objects/targets (e.g., "blue ball" from user command)
  - [ ] **Semantic Understanding**: Extracting meaning (intent + entities + constraints)
  - [ ] **Prompt**: Input text that guides LLM toward output format
- **Verification**: Each term appears in Section 1 with clear, standalone definition
- **Acceptance**: PASS if all 4 terms clearly defined, suitable for beginners

---

#### **AC-2.2**: Explains LLM role without requiring ML math or training knowledge
- **Definition**: Chapter must explain LLMs conceptually without deep learning mathematics
- **Requirement**: Accessible to someone with no neural network background
- **Content**:
  - [ ] What is an LLM in one sentence
  - [ ] What an LLM does (takes text input, produces structured output)
  - [ ] Why LLMs are useful for robots (flexible, understand language)
  - [ ] NO mention of: gradient descent, backpropagation, transformer architecture math, training procedures
- **Tone**: "The LLM is a sophisticated pattern-matching system trained on billions of text examples..."
- **Acceptance**: PASS if LLMs explained at system level, no ML math

---

#### **AC-2.3**: Includes workflow diagram (Text → LLM → Structured Plan)
- **Definition**: Chapter must include diagram showing LLM's planning transformation
- **Requirement**: Visual representation of text input → intent/entities → action plan output
- **Specifications**:
  - [ ] Mermaid diagram OR ASCII diagram provided
  - [ ] Shows: Text input → LLM processing → Structured output (JSON or table)
  - [ ] Example: "Pick up blue ball" → {action: pick_up, object: blue_ball}
  - [ ] Includes explanation (250-350 words)
  - [ ] Diagram placement in Section 2
- **Acceptance**: PASS if diagram clearly shows text-to-plan transformation

---

#### **AC-2.4**: Covers real-world scenarios (ambiguous, multi-step, constraints)
- **Definition**: Chapter must demonstrate LLM planning through 4+ practical robot scenarios
- **Requirement**: Show how LLMs handle complexity in real commands
- **Scenarios Required**:
  - [ ] Ambiguous command ("Move it" - move what? Where?)
  - [ ] Multi-step command (pick up → carry → place)
  - [ ] Constraint-heavy command ("Slowly move the vase")
  - [ ] Additional complex scenario (compound objects, conditional, etc.)
- **Evidence**: Each scenario (150-250 words):
  - User voice command
  - Whisper transcription
  - LLM parsing (intent, entities, constraints)
  - Action goal generated
  - Success or limitation
- **Acceptance**: PASS if 4+ distinct scenarios show LLM capabilities and limitations

---

#### **AC-2.5**: Explicitly states scope boundary (LLMs as APIs, not fine-tuned)
- **Definition**: Chapter must clarify what's IN scope (using LLM APIs) and OUT of scope (fine-tuning/training)
- **Requirement**: Clear scope statement in Section 5
- **Content Required**:
  - [ ] "We use LLMs via API (e.g., OpenAI's GPT), not fine-tuning"
  - [ ] Reason: "Proprietary models are too large and expensive for custom fine-tuning"
  - [ ] What readers WON'T learn: "How to fine-tune GPT for robot-specific language"
  - [ ] Implication: "LLM behavior and quality depend on API provider"
  - [ ] Mention cost: "Token-based pricing applies"
- **Acceptance**: PASS if scope explicitly stated, API-based approach justified

---

## Contract 3: Chapter 3 — ROS 2 Action Integration (U3/P2)

**User Story**: U3: "I want to understand how robot plans become actual movements in ROS 2"
**Priority**: P2 (Medium)
**Expected Word Count**: 4,500-5,500 words
**Expected Reading Time**: 18 minutes
**Expected Diagrams**: 3-5 (Types A, B, C, D, E represented)

### Acceptance Criteria

#### **AC-3.1**: Defines "Action Server," "Trajectory," "Goal," "Feedback," and "Result" clearly
- **Definition**: Chapter must define 5 core ROS 2 Action concepts in beginner-friendly language
- **Requirement**: Each term defined before architectural explanation
- **Definitions**:
  - [ ] **Action Server**: ROS 2 component that executes long-running tasks with feedback
  - [ ] **Trajectory**: Sequence of waypoints and velocities robot follows
  - [ ] **Goal**: What the action should accomplish (e.g., position to reach)
  - [ ] **Feedback**: Progress updates during execution (e.g., current position)
  - [ ] **Result**: Final outcome when action completes
- **Verification**: Each term appears in Section 1 with definition suitable for ROS 2 beginners
- **Acceptance**: PASS if all 5 terms clearly defined

---

#### **AC-3.2**: Explains ROS 2 Actions with reference to Module 1 knowledge
- **Definition**: Chapter must connect ROS 2 Actions to concepts learned in Module 1
- **Requirement**: 3-5 explicit cross-links to Module 1 chapters showing how Actions build on foundational concepts
- **Expected References**:
  - [ ] Module 1 Actions introduction: "Recall from Module 1 Chapter 1..."
  - [ ] Distinguish from topics: "Unlike topics (streaming data), actions are..."
  - [ ] Distinguish from services: "Unlike services (quick request-response), actions provide..."
  - [ ] Python clients: "You learned about rclpy in Module 1; here's how to write action clients..."
  - [ ] URDF application: "The URDF models from Module 1 define joint constraints for action execution..."
- **Link Format**: `[Module 1: ROS 2 Actions](/docs/module1/chapter1-ros2-fundamentals#ros-2-actions)`
- **Acceptance**: PASS if 3-5 Module 1 cross-links naturally integrated

---

#### **AC-3.3**: Includes workflow diagram (Plan → Action Server → Motor Commands)
- **Definition**: Chapter must show how structured plans from Chapter 2 become robot motion
- **Requirement**: Visual representation of action execution from goal to result
- **Specifications**:
  - [ ] Mermaid diagram OR ASCII diagram provided
  - [ ] Shows: Structured Plan → Action Goal → Action Server → Execution → Feedback → Result
  - [ ] Includes timeline/sequence aspect
  - [ ] Example: {action: pick_up, object: blue_ball} → Motor commands (gripper position, velocity)
  - [ ] Includes explanation (300-400 words)
  - [ ] Diagram placement in Section 2
- **Acceptance**: PASS if diagram clearly shows plan-to-motion transformation

---

#### **AC-3.4**: Covers real-world scenarios (timeouts, failures, obstacles)
- **Definition**: Chapter must demonstrate action execution through 4+ practical scenarios including failures
- **Requirement**: Show realistic challenges robots encounter during action execution
- **Scenarios Required**:
  - [ ] Success scenario (action completes as planned)
  - [ ] Timeout scenario (action takes longer than expected)
  - [ ] Failure scenario (action cannot achieve goal - e.g., gripper jam)
  - [ ] Obstacle scenario (dynamic obstacle appears during execution)
- **Evidence**: Each scenario (150-250 words):
  - Goal being executed
  - Execution challenge/event
  - How action server responds
  - Recovery strategy or failure handling
  - Feedback mechanism role
- **Acceptance**: PASS if 4+ distinct scenarios show both success and failure modes

---

#### **AC-3.5**: Explicitly states scope boundary (ROS 2 standard usage, not implementation)
- **Definition**: Chapter must clarify what's IN scope (using Actions) and OUT of scope (implementing Action Server)
- **Requirement**: Clear scope statement in Section 5
- **Content Required**:
  - [ ] "We learn to USE ROS 2 Actions, not implement them from scratch"
  - [ ] Reason: "ROS 2 provides standard action implementations"
  - [ ] What readers WON'T learn: "How to write a custom action server in C++"
  - [ ] Level of detail: "Conceptual understanding, not code-level implementation"
  - [ ] Why it matters: "Roboticists focus on task design, not ROS 2 internals"
- **Acceptance**: PASS if scope clearly stated (usage vs implementation distinction made)

---

## Contract 4: Chapter 4 — Complete VLA Pipeline (U4/P2)

**User Story**: U4: "I want to understand how perception, language, and action work together in a complete VLA system"
**Priority**: P2 (Medium)
**Expected Word Count**: 4,500-5,500 words
**Expected Reading Time**: 20 minutes
**Expected Diagrams**: 3-5 (Types A, B, C, D, E represented, with emphasis on Type A: Complete pipeline)

### Acceptance Criteria

#### **AC-4.1**: Integrates Chapters 1-3 into a cohesive pipeline narrative
- **Definition**: Chapter must weave Whisper, LLM, and ROS 2 Actions into a unified story
- **Requirement**: Avoid repeating Chapter 1-3 content; instead show how they work together
- **Content Structure**:
  - [ ] Section 1: "What is VLA?" (system overview, not repeated definitions)
  - [ ] Section 2: Complete VLA workflow (not separate chapters, but integrated flow)
  - [ ] Section 3: Real-world VLA scenarios (not individual component scenarios)
  - [ ] Section 4: How Modules 1 and 3 enable VLA (not repeated basics)
  - [ ] Section 5: System-level takeaways (not component-level)
- **Narrative Approach**: "Voice input flows through Whisper (Ch1) → LLM (Ch2) → Actions (Ch3) → Perception feedback (Ch4)"
- **Acceptance**: PASS if Chapter 4 reads as integrated system, not Chapter 1-3 summary

---

#### **AC-4.2**: Includes end-to-end workflow diagram with perception feedback loop
- **Definition**: Chapter must show complete VLA pipeline including perception feedback
- **Requirement**: Visual representation of voice-to-action-to-feedback cycle
- **Specifications**:
  - [ ] Mermaid diagram OR ASCII diagram provided
  - [ ] Shows: Voice Input → Whisper → Transcription → LLM → Plan → Actions → Motion → Perception Feedback
  - [ ] Includes feedback loop explicitly (arrow from result back to perception/understanding)
  - [ ] Shows time dimension (sequential vs parallel where applicable)
  - [ ] Includes explanation (400-500 words)
  - [ ] Diagram placement in Section 2
  - [ ] Type A (Architecture) diagram recommended for this critical diagram
- **Acceptance**: PASS if diagram shows complete cycle with feedback loop

---

#### **AC-4.3**: Explains how perception (Module 3 VSLAM) feeds back into language understanding
- **Definition**: Chapter must show the feedback loop from perception to cognition
- **Requirement**: Demonstrate how robot's perception informs next planning cycle
- **Content**:
  - [ ] Reference Module 3 VSLAM: "Localization from Module 3 tells the robot where it is"
  - [ ] Reference Module 3 Nav2: "Navigation planning from Module 3 guides safe motion"
  - [ ] Feedback mechanism: "After action executes, VSLAM confirms success or triggers replanning"
  - [ ] Example: "Command: 'Go to the kitchen.' VSLAM shows robot reached kitchen → action complete. If VSLAM shows robot still in living room → replan."
  - [ ] Error recovery: "If perception contradicts expectation, robot can request clarification or retry"
- **Cross-Links**: 2-3 explicit links to Module 3 chapters
- **Acceptance**: PASS if perception feedback loop is clearly explained with Module 3 integration

---

#### **AC-4.4**: Traces complete examples showing voice → perception → understanding → planning → action with feedback
- **Definition**: Chapter must walk through 4+ complete end-to-end scenarios
- **Requirement**: Each scenario shows complete VLA cycle from voice input to feedback
- **Scenarios Required**:
  - [ ] **Scenario 1**: Simple single-step (e.g., "Pick up the blue ball")
  - [ ] **Scenario 2**: Multi-step (e.g., "Pick up the blue ball and bring it to the kitchen")
  - [ ] **Scenario 3**: With unexpected event (e.g., obstacle appears during motion)
  - [ ] **Scenario 4**: With perception-based recovery (e.g., object moved, robot notices and re-plans)
- **Trace Structure** (each scenario 300-400 words):
  - Voice command (user speaks)
  - Whisper output (transcription with confidence)
  - LLM parsing (intent, entities, constraints extracted)
  - Action planning (what robot needs to do)
  - Action execution (motion commands sent)
  - Perception feedback (VSLAM/cameras confirm progress)
  - Result (success, failure, or adjustment)
- **Acceptance**: PASS if 4+ complete traces show voice-to-feedback cycles

---

#### **AC-4.5**: Includes module wrap-up message (Student understands voice-to-action pipeline)
- **Definition**: Chapter must conclude with a clear statement of learning achievement
- **Requirement**: Explicit summary of what student now understands
- **Content Required**:
  - [ ] "You now understand how humanoid robots understand and act on voice commands"
  - [ ] Specific outcomes:
    - "You can explain how Whisper converts voice to text"
    - "You can describe how LLMs extract intent and plan actions"
    - "You can trace how ROS 2 Actions execute plans as robot motion"
    - "You can identify the role of perception feedback in robot task success"
  - [ ] Forward reference: "These concepts form the foundation for advanced VLA systems (multi-agent coordination, distributed perception, real-time constraints)"
  - [ ] Call to action: "Next steps: [Build a VLA system yourself using ROS 2 and Whisper / Explore advanced topics in [next module]]"
- **Tone**: Celebratory but honest (students have learned significant concepts)
- **Acceptance**: PASS if wrap-up clearly states learning achievement

---

## Acceptance Criteria Summary Table

| Chapter | AC ID | Criterion | Status |
|---------|-------|-----------|--------|
| 1 | AC-1.1 | Defines "Speech Recognition" and "Whisper" | Pending |
| 1 | AC-1.2 | Explains why Whisper output needs further processing | Pending |
| 1 | AC-1.3 | Includes workflow diagram (Audio → Whisper → Text) | Pending |
| 1 | AC-1.4 | Covers real-world scenarios (noise, accents, languages) | Pending |
| 1 | AC-1.5 | States scope boundary (Whisper as service) | Pending |
| **2** | **AC-2.1** | **Defines Intent, Entity, Semantic Understanding, Prompt** | **Pending** |
| 2 | AC-2.2 | Explains LLM role without ML math | Pending |
| 2 | AC-2.3 | Includes workflow diagram (Text → LLM → Plan) | Pending |
| 2 | AC-2.4 | Covers real-world scenarios (ambiguous, multi-step) | Pending |
| 2 | AC-2.5 | States scope boundary (LLMs as APIs) | Pending |
| 3 | AC-3.1 | Defines Action Server, Trajectory, Goal, Feedback, Result | Pending |
| 3 | AC-3.2 | Explains with Module 1 knowledge (3-5 cross-links) | Pending |
| 3 | AC-3.3 | Includes workflow diagram (Plan → Actions → Commands) | Pending |
| 3 | AC-3.4 | Covers real-world scenarios (timeouts, failures, obstacles) | Pending |
| 3 | AC-3.5 | States scope boundary (usage, not implementation) | Pending |
| 4 | AC-4.1 | Integrates Ch1-3 into cohesive narrative | Pending |
| 4 | AC-4.2 | Includes end-to-end diagram with perception feedback | Pending |
| 4 | AC-4.3 | Explains perception feedback from Module 3 VSLAM | Pending |
| 4 | AC-4.4 | Traces 4+ complete voice-to-action examples | Pending |
| 4 | AC-4.5 | Includes module wrap-up message | Pending |

---

## Quality Checklist per Chapter

### All Chapters Must Include:

**Content Structure**:
- [ ] YAML frontmatter (14 fields per data-model.md)
- [ ] Section 1: Core Concepts (1,000-1,200 words)
- [ ] Section 2: Architecture & Workflow (1,000-1,200 words) + 2 diagrams minimum
- [ ] Section 3: Real-World Applications (1,200-1,500 words) + examples
- [ ] Section 4: Module Integration (800-1,000 words) + 3-5 cross-links
- [ ] Section 5: Key Takeaways & Scope (600-800 words)
- [ ] Edge Cases & Limitations (300-500 words)
- [ ] **TOTAL**: 4,500-5,500 words

**Diagrams**:
- [ ] Minimum 3 diagrams (Types A, B, C, D, or E)
- [ ] At least one Type B (Workflow) diagram
- [ ] All diagrams have captions and 100-300 word explanations
- [ ] Mermaid or ASCII format (no embedded images)

**Accessibility**:
- [ ] Flesch-Kincaid readability: Grade 10-12 (verified with tool)
- [ ] Sentence length: 12-15 words average
- [ ] Paragraph length: 100-300 words
- [ ] All technical terms defined on first use
- [ ] Acronyms spelled out on first use

**Cross-Linking**:
- [ ] 3-5 Module 1 cross-links per chapter (format: `[text](/docs/module1/...#anchor)`)
- [ ] 1-6 Module 3 cross-links per chapter (heavier in Ch3-4)
- [ ] 1-2 intra-module cross-links (to other chapters in Module 4)

**RAG Compatibility**:
- [ ] ~10 chunks expected per chapter (512 ± 100 tokens)
- [ ] Section boundaries preferred for chunk splits
- [ ] All chunks have associated metadata (from data-model.md)
- [ ] URLs follow pattern: `/docs/module4/chapter-slug#section-slug`

---

## Acceptance Verification Process

### Step 1: Self-Check (Writer)
Writer verifies their own chapter against all 5 ACs before submission:
```
Chapter 1 Checklist:
- [ ] AC-1.1: Whisper definition ✓
- [ ] AC-1.2: Why transcription insufficient ✓
- [ ] AC-1.3: Diagram included ✓
- [ ] AC-1.4: 4+ scenarios covered ✓
- [ ] AC-1.5: Scope boundary stated ✓
```

### Step 2: Technical Review
Verify word count, readability, formatting:
- [ ] Word count: 4,500-5,500 (not 4,200 or 5,800)
- [ ] Readability: Use Hemingway Editor or equivalent
- [ ] YAML frontmatter: All 14 fields present and valid
- [ ] Links: All cross-links use correct URL format

### Step 3: Acceptance Criteria Verification
For each AC, verify with specific evidence:
- [ ] **AC-1.1**: Search chapter for "Speech Recognition" definition location (line number)
- [ ] **AC-1.2**: Identify 2+ ambiguity examples and their location
- [ ] **AC-1.3**: Verify diagram exists, has caption, explanation present
- [ ] **AC-1.4**: Count scenarios (must be 4+), verify coverage (noise, accent, multilingual, other)
- [ ] **AC-1.5**: Locate scope boundary section, verify explicit statement

### Step 4: Final Gate
All 5 ACs for a chapter must show PASS status before moving to next phase:
```
CHAPTER 1 ACCEPTANCE:
- [x] AC-1.1: ✓ PASS
- [x] AC-1.2: ✓ PASS
- [x] AC-1.3: ✓ PASS
- [x] AC-1.4: ✓ PASS
- [x] AC-1.5: ✓ PASS
STATUS: ✅ CHAPTER 1 APPROVED FOR PHASE 3
```

---

**Created**: 2025-12-08 | **Feature**: 004-vla-pipeline | **Status**: Phase 1 Design Complete

**Next Phase**: Phase 2 (Chapter 1 Writing) - Writers use these contracts to create chapter1-whisper.md
