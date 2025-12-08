# RAG Chunking Strategy & Implementation Guide

**Purpose**: Define chunk boundaries, sizes, metadata preservation rules for Module 3 content
**Created**: 2025-12-08 | **Feature**: 003-isaac-brain

---

## Chunking Overview

**Chunk Size Target**: 512 tokens ± 100 (400-800 range)

**Overlap Strategy**: 20% overlap between chunks (preserve context across boundaries)

**Total Expected Chunks**: ~16 chunks across 4 chapters (~4 chunks per chapter)

---

## Token Estimation

- **1 chunk** = ~512 tokens
- **Words per chunk** = 512 ÷ 1.3 ≈ 394 words per chunk
- **Chapter 1** (5,000 words) = ~13 chunks; estimate 4 chunks after combining
- **Chapter 2** (4,500 words) = ~11 chunks; estimate 4 chunks after combining
- **Chapter 3** (5,500 words) = ~14 chunks; estimate 5 chunks after combining
- **Chapter 4** (5,000 words) = ~13 chunks; estimate 4 chunks after combining

---

## Metadata Preservation (Every Chunk MUST Include)

```json
{
  "id": "module3-chapter1-chunk-001",
  "text": "chunk content here (512 tokens)...",
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
    "keywords": ["photorealistic-simulation", "isaac-sim", "perception"],
    "learning_objectives": [
      "Explain what photorealistic simulation is",
      "Understand why it's used in robotics"
    ],
    "related_chapters": ["module3-chapter2", "module1-chapter3"]
  }
}
```

**Required Fields**:
- `module`, `chapter`, `chapter_title` - Content location
- `section`, `section_number`, `subsection` - Hierarchical position
- `chunk_index`, `chunk_total` - Position in chapter
- `token_count` - Verify 400-800 range
- `url` - Enable precise citations
- `keywords` - Support semantic search
- `learning_objectives` - Context for RAG filtering
- `related_chapters` - Enable cross-referencing

---

## Chunking Boundaries (Prefer section ends)

**Preferred**:
- After complete subsection (1.1, 1.2, 1.3)
- After complete example
- After complete workflow step
- After edge case description

**Acceptable**:
- Mid-subsection if next section starts
- Mid-paragraph if chunk boundary falls there

**Avoid**:
- Mid-sentence (always complete sentences)
- Mid-concept (incomplete ideas)
- Orphaned 1-2 sentence chunks (too small)

---

## Overlap Implementation

**Overlap Size**: ~102 tokens (20% of 512)

**Rule**: Last 102 tokens of Chunk N = First 102 tokens of Chunk N+1

**Example**:
```
Chunk 1 (512 tokens):
[...content...] [Conclusion paragraph with 102 tokens]

Chunk 2 (512 tokens):
[Conclusion paragraph with 102 tokens - COPY from Chunk 1]
[New content = 410 tokens]
```

**Benefit**: When searching, related chunks appear in results even if query matches only tail of one chunk.

---

## Per-Chapter Chunking Plan

### Chapter 1: Isaac Sim Fundamentals (5,000 words → 4 chunks)

**Chunk 1**: Learning Objectives + Section 1.1 + 1.2
- Content: "What is Photorealistic Simulation?" + "Physics Engine"
- Tokens: ~512 | Words: ~400
- Overlap tail: Last definition/concept from Section 1.2

**Chunk 2**: Section 1.3 + Section 2 (Architecture)
- Content: "Coordinate Frames, Cameras, Sensors" + "Isaac Sim Workflow"
- Tokens: ~512 | Words: ~400
- Overlap head: Frames definition
- Overlap tail: Workflow diagram or final process step

**Chunk 3**: Section 3 (Applications) + Start of Section 4
- Content: Two real-world examples + ROS 2 integration intro
- Tokens: ~512 | Words: ~400
- Overlap head: Second example intro
- Overlap tail: First cross-reference

**Chunk 4**: Section 4 + 5 + Edge Cases
- Content: ROS 2 integration + Key Takeaways + Simulator vs. Reality Gap
- Tokens: ~512 | Words: ~400
- Overlap head: Integration intro
- No overlap tail (last chunk)

### Chapter 2: Synthetic Data (4,500 words → 4 chunks)

**Chunk 1**: Objectives + Sections 1.1-1.2
- Content: "What is Synthetic Data?" + "Domain Adaptation"
- Tokens: ~512

**Chunk 2**: Section 1.3 + Section 2
- Content: "Data Diversity" + "Data Generation Workflow"
- Tokens: ~512

**Chunk 3**: Section 3 + Start of 4
- Content: "Real-World Applications" + "Integration with Agents"
- Tokens: ~512

**Chunk 4**: Section 4-5 + Edge Cases
- Content: ROS 2 context + Takeaways + Sim-to-Real Gap
- Tokens: ~512

### Chapter 3: VSLAM (5,500 words → 5 chunks)

**Chunk 1**: Objectives + Sections 1.1-1.2
- Content: "What is VSLAM?" + "Visual Odometry"
- Tokens: ~512

**Chunk 2**: Section 1.3-1.4 + Part of 2
- Content: "Loop Closure" + Start "Architecture"
- Tokens: ~512

**Chunk 3**: Section 2 continuation + Start of 3
- Content: "VSLAM Pipeline" + "Real-World Applications" intro
- Tokens: ~512

**Chunk 4**: Section 3 continuation + Section 4
- Content: Applications examples + "Integration with ROS 2"
- Tokens: ~512

**Chunk 5**: Section 5 + Edge Cases
- Content: "Key Takeaways" + "Edge Cases: Incomplete Maps, Featureless Walls"
- Tokens: ~512

### Chapter 4: Nav2 (5,000 words → 4 chunks)

**Chunk 1**: Objectives + Sections 1.1-1.2
- Content: "Costmaps" + "Path Planning Algorithms"
- Tokens: ~512

**Chunk 2**: Section 1.3 + Section 2
- Content: "Global vs. Local Planning" + "Nav2 Architecture"
- Tokens: ~512

**Chunk 3**: Section 3 + Start of 4
- Content: "Real-World Applications" + "ROS 2 Integration" intro
- Tokens: ~512

**Chunk 4**: Section 4-5 + Edge Cases
- Content: ROS connections + Takeaways + Path Planning Failures
- Tokens: ~512

---

## RAG Indexing Workflow

**Input**: 4 chapter files (chapter1-isaac-sim.md, etc.)

**Process**:
1. Parse each chapter into sections
2. Apply chunking boundaries (prefer section ends)
3. Ensure each chunk 400-800 tokens
4. Add 20% overlap between consecutive chunks
5. Extract/generate metadata for each chunk
6. Create index.json with all chunks
7. Upload to Qdrant vector database

**Output**:
- `index.json` (16 chunks with metadata)
- Qdrant collection "module3" with 16 vectors

**Validation**:
- [ ] All chunks 400-800 tokens
- [ ] All chunks have complete metadata
- [ ] 20% overlap implemented
- [ ] URLs match Docusaurus structure
- [ ] Keywords enable semantic search
- [ ] Chunk boundaries respect section hierarchies

---

## Search Query Examples

**Good RAG Query Results**:
- Query: "How does Isaac Sim help train perception?"
  - Should return: Chunk 1-1 (photorealistic simulation benefit)
  - Context from overlap: Physics engine details

- Query: "What is the sim-to-real gap?"
  - Should return: Chapter 2 Chunk 1 (domain adaptation section)
  - Context from overlap: Synthetic data definition

- Query: "How does VSLAM localize a robot?"
  - Should return: Chapter 3 Chunk 1-2 (visual odometry + loop closure)
  - Context from overlap: feature tracking steps

- Query: "How does Nav2 plan around obstacles?"
  - Should return: Chapter 4 Chunk 1-2 (costmaps + planning algorithms)
  - Context from overlap: costmap representation

---

**Implementation Responsibility**: T061 in tasks.md (Generate RAG chunk index)
**Testing Responsibility**: Post-deployment RAG retrieval validation

---

**Last Updated**: 2025-12-08 | **Feature**: 003-isaac-brain
