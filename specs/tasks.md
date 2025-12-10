# Complete Humanoid Robotics Textbook: Master Task List

**Project**: AI-Driven Docusaurus Book + RAG Chatbot + Humanoid Robotics Capstone
**Scope**: 4 Modules × 4 Phases | 15 Chapters | Full Pipeline
**Target Completion**: 13 Weeks
**Generated**: 2025-12-09

---

## Executive Summary

This document breaks down the entire **humanoid robotics textbook project** into 4 phases, each corresponding to one module. All tasks follow the strict checklist format with Task IDs, parallelization markers, and file paths.

- **Phase 0 (Week 1)**: Setup, Research, Validation
- **Phase 1 (Weeks 1-2)**: Module 1 — ROS 2 Fundamentals (Content Refinement)
- **Phase 2 (Weeks 2-3)**: Module 2 — Digital Twin (Content Refinement)
- **Phase 3 (Weeks 3-4)**: Module 3 — Isaac Sim & Perception (Content Refinement)
- **Phase 4 (Weeks 4-5)**: Module 4 — VLA Pipeline (Content Refinement)
- **Phase 5+ (Weeks 5+)**: Infrastructure, Deployment, Features

**Total Tasks**: ~120 individual, independently testable tasks
**MVP Scope**: Phase 0 (Research) + Phase 1 (Module 1 Refinement)

---

## Task Dependencies & Execution Order

```
Phase 0 (Research) → Foundation
   ↓
Phase 1-4 (Content Refinement) → Can run in parallel once Phase 0 completes
   ↓
Phase 5 (Data Model & Contracts) → Blocking for Phases 6-8
   ↓
Phase 6-8 (Infrastructure) → Can run in parallel
   ↓
Phase 9-10 (Deployment & Features)
```

**Parallel Opportunities**:
- All 4 content refinement phases (1-4) can run independently after Phase 0
- Data model + contracts (Phase 5) can be designed while content is finalized
- Frontend (Docusaurus) build and backend (FastAPI) can be built in parallel

---

# PHASE 0: Research & Validation (Week 1)

**Goal**: Verify all technical content, resolve unknowns, and prepare for implementation.

## Research Tasks

- [ ] T001 Research and document Whisper architecture in `specs/004-vla-pipeline/research.md`
- [ ] T002 [P] Verify LLM prompting techniques against OpenAI documentation
- [ ] T003 [P] Validate ROS 2 Action Server lifecycle and documentation
- [ ] T004 [P] Verify Isaac Sim capabilities (URDF, PhysX, RTX rendering)
- [ ] T005 [P] Validate Nav2 path planning algorithms (A*, Dijkstra, RRT)
- [ ] T006 [P] Research Gazebo physics engine specifications
- [ ] T007 [P] Research VSLAM and its role in robotics
- [ ] T008 [P] Research synthetic data generation with Isaac Sim

## Content Audit Tasks

- [ ] T009 Read all 15 chapters and measure Flesch-Kincaid readability scores
- [ ] T010 [P] Create content audit report in `CONTENT_AUDIT_2025-12-09.md`
- [ ] T011 [P] Identify cross-reference opportunities across modules
- [ ] T012 [P] Verify all external URLs (OpenAI docs, ROS 2 docs, Isaac docs)
- [ ] T013 [P] Check for broken anchor links in Markdown files

## Decision Documentation

- [ ] T014 Document technology stack decisions in `specs/004-vla-pipeline/research.md`
- [ ] T015 Create glossary foundation terms list in `docs/glossary.md` (50+ robotics terms)

**Phase 0 Acceptance Criteria**:
- ✅ All technical claims verified against official documentation
- ✅ 5 critical content issues identified
- ✅ Technology stack ratified with alternatives documented
- ✅ Glossary framework established

---

# PHASE 1: Module 1 — ROS 2 Fundamentals (Weeks 1-2)

**Goal**: Refine, polish, and prepare Module 1 content for RAG and deployment.

**Module Structure**:
- Module intro (new)
- Chapter 1: ROS 2 Core Concepts
- Chapter 2: Agent Bridge & Communication
- Chapter 3: URDF & Robot Modeling

## Content Refinement Tasks (Module 1)

### Readability & Consistency

- [ ] T016 Create `docs/module1/intro.md` with prerequisites and module overview (1500 words)
- [ ] T017 [P] Edit M1Ch1 (`docs/module1/chapter1-ros2-core.md`) for readability (target FK 10-12)
- [ ] T018 [P] Edit M1Ch2 (`docs/module1/chapter2-agent-bridge.md`) for readability
- [ ] T019 [P] Edit M1Ch3 (`docs/module1/chapter3-urdf-model.md`) for readability
- [ ] T020 [P] Add acronym definitions to all chapters (ROS 2, URDF, pub/sub, etc.)
- [ ] T021 [P] Standardize terminology across Module 1 chapters

### Cross-Linking & References

- [ ] T022 [P] Add 5-7 internal cross-links per Module 1 chapter (to other modules)
- [ ] T023 [P] Verify all Glossary references work correctly
- [ ] T024 [P] Update navigation breadcrumbs in chapter headers

### Diagram Creation & Validation

- [ ] T025 Create diagram: ROS 2 Architecture (Nodes, Topics, Services) in M1Ch1
- [ ] T026 [P] Create diagram: Pub/Sub communication pattern in M1Ch2
- [ ] T027 [P] Create diagram: URDF hierarchical structure in M1Ch3
- [ ] T028 [P] Validate all Mermaid syntax in module (no external images)

### RAG Preparation

- [ ] T029 Add YAML frontmatter to all M1 chapters with 14-field metadata schema
- [ ] T030 [P] Identify natural chunk boundaries in M1 chapters
- [ ] T031 [P] Add metadata headers (section, keywords, learning objectives) to chunk boundaries

### Quality Assurance

- [ ] T032 Run Flesch-Kincaid check on all M1 chapters (document in `QA_REPORT_M1.md`)
- [ ] T033 [P] Verify all external URL references work (Robots.ROS.org, ROS 2 docs)
- [ ] T034 [P] Test cross-module links (Module 1 → Module 3, Module 1 → Glossary)

**Phase 1 Acceptance Criteria**:
- ✅ All M1 chapters 10-12 FK readability
- ✅ Module 1 intro complete with 3+ learning paths
- ✅ 3-5 diagrams per chapter, text-based (Mermaid)
- ✅ YAML frontmatter on all chapters
- ✅ 95%+ cross-links working

---

# PHASE 2: Module 2 — Digital Twin (Weeks 2-3)

**Goal**: Refine, polish, and prepare Module 2 content for RAG and deployment.

**Module Structure**:
- Module intro (exists)
- Chapter 1: Digital Twin Concepts
- Chapter 2: Gazebo Physics
- Chapter 3: World Building
- Chapter 4: Sensor Simulation
- Chapter 5: Unity Visualization

## Content Refinement Tasks (Module 2)

### Readability & Consistency

- [ ] T035 [P] Edit M2Ch1 (`docs/module2/chapter1-digital-twin-concepts.md`) for readability
- [ ] T036 [P] Edit M2Ch2 (`docs/module2/chapter2-gazebo-physics.md`) for readability
- [ ] T037 [P] Edit M2Ch3 (`docs/module2/chapter3-world-building.md`) for readability
- [ ] T038 [P] Edit M2Ch4 (`docs/module2/chapter4-sensor-simulation.md`) for readability
- [ ] T039 [P] Edit M2Ch5 (`docs/module2/chapter5-unity-visualization.md`) for readability
- [ ] T040 [P] Add acronym definitions (LiDAR, IMU, CAD, etc.)
- [ ] T041 [P] Standardize terminology across Module 2 chapters

### Cross-Linking & References

- [ ] T042 [P] Add 5-7 internal cross-links per Module 2 chapter
- [ ] T043 [P] Cross-link to Module 1 (ROS 2 integration with Gazebo)
- [ ] T044 [P] Add forward references to Module 3 (perception post-processing)

### Diagram Creation & Validation

- [ ] T045 Create diagram: Physics engine pipeline in M2Ch2
- [ ] T046 [P] Create diagram: World structure (robots, objects, environment) in M2Ch3
- [ ] T047 [P] Create diagram: Sensor simulation workflow in M2Ch4
- [ ] T048 [P] Create diagram: Unity integration points in M2Ch5
- [ ] T049 [P] Validate all Mermaid syntax and ASCII diagrams

### RAG Preparation

- [ ] T050 [P] Add YAML frontmatter to all M2 chapters (14-field schema)
- [ ] T051 [P] Identify natural chunk boundaries in M2 chapters
- [ ] T052 [P] Add metadata headers to chunk boundaries

### Quality Assurance

- [ ] T053 Run Flesch-Kincaid check on all M2 chapters (document in `QA_REPORT_M2.md`)
- [ ] T054 [P] Verify Gazebo and Unity documentation references
- [ ] T055 [P] Test cross-module links (Module 2 → Module 1, Module 2 → Module 3)

**Phase 2 Acceptance Criteria**:
- ✅ All M2 chapters 10-12 FK readability
- ✅ 3-5 diagrams per chapter (text-based)
- ✅ YAML frontmatter on all chapters
- ✅ Cross-links to Module 1 and Module 3 established
- ✅ 95%+ cross-links working

---

# PHASE 3: Module 3 — Isaac Sim & Perception (Weeks 3-4)

**Goal**: Refine, polish, and prepare Module 3 content for RAG and deployment.

**Module Structure**:
- Module intro (exists)
- Chapter 1: Isaac Sim Introduction
- Chapter 2: Synthetic Data Generation
- Chapter 3: VSLAM Localization
- Chapter 4: Nav2 Path Planning

## Content Refinement Tasks (Module 3)

### Readability & Consistency

- [ ] T056 [P] Edit M3Ch1 (`docs/module3/chapter1-isaac-sim.md`) for readability
- [ ] T057 [P] Edit M3Ch2 (`docs/module3/chapter2-synthetic-data.md`) for readability
- [ ] T058 [P] Edit M3Ch3 (`docs/module3/chapter3-vslam.md`) for readability
- [ ] T059 [P] Edit M3Ch4 (`docs/module3/chapter4-nav2.md`) for readability
- [ ] T060 [P] Split M3Ch4 "Nav2 Global Planner" section into 4 subsections (Dijkstra, A*, RRT, Choice)
- [ ] T061 [P] Add acronym definitions (VSLAM, SLAM, LiDAR, RTX)
- [ ] T062 [P] Standardize terminology across Module 3 chapters

### Cross-Linking & References

- [ ] T063 [P] Add 5-7 internal cross-links per Module 3 chapter
- [ ] T064 [P] Cross-link to Module 1 (ROS 2 node structure for perception)
- [ ] T065 [P] Cross-link to Module 2 (Gazebo/Isaac Sim simulation for testing)
- [ ] T066 [P] Add forward references to Module 4 (perception feedback in VLA)

### Diagram Creation & Validation

- [ ] T067 Create diagram: Isaac Sim architecture in M3Ch1
- [ ] T068 [P] Create diagram: Synthetic data pipeline in M3Ch2
- [ ] T069 [P] Create diagram: VSLAM localization loop in M3Ch3
- [ ] T070 [P] Create diagram: Nav2 path planning (A*, RRT comparison) in M3Ch4
- [ ] T071 [P] Create diagram: Decision tree for algorithm selection in M3Ch4
- [ ] T072 [P] Validate all Mermaid syntax and ASCII diagrams

### RAG Preparation

- [ ] T073 [P] Add YAML frontmatter to all M3 chapters (14-field schema)
- [ ] T074 [P] Identify natural chunk boundaries in M3 chapters
- [ ] T075 [P] Add metadata headers to chunk boundaries

### Quality Assurance

- [ ] T076 Run Flesch-Kincaid check on all M3 chapters (document in `QA_REPORT_M3.md`)
- [ ] T077 [P] Verify Isaac Sim, VSLAM, and Nav2 documentation references
- [ ] T078 [P] Test cross-module links (Module 3 → all other modules)

**Phase 3 Acceptance Criteria**:
- ✅ All M3 chapters 10-12 FK readability
- ✅ M3Ch4 properly segmented (Dijkstra/A*/RRT/Choice)
- ✅ 3-5 diagrams per chapter (text-based)
- ✅ YAML frontmatter on all chapters
- ✅ Cross-links to Modules 1, 2, 4 established

---

# PHASE 4: Module 4 — VLA Pipeline (Weeks 4-5)

**Goal**: Refine, polish, and prepare Module 4 content for RAG and deployment.

**Module Structure**:
- Module intro (exists or new)
- Chapter 1: Whisper Speech Recognition
- Chapter 2: LLM Cognitive Planning
- Chapter 3: ROS 2 Action Integration
- Chapter 4: Complete VLA Pipeline

## Content Refinement Tasks (Module 4)

### Readability & Consistency

- [ ] T079 [P] Edit M4Ch1 (`docs/module4/chapter1-whisper-speech.md`) for readability
- [ ] T080 [P] Edit M4Ch2 (`docs/module4/chapter2-llm-planning.md`) for readability
- [ ] T081 [P] Edit M4Ch3 (`docs/module4/chapter3-ros2-actions.md`) for readability
- [ ] T082 [P] Edit M4Ch4 (`docs/module4/chapter4-complete-vla.md`) for readability
- [ ] T083 [P] Standardize "Action Server" capitalization across Module 4
- [ ] T084 [P] Add acronym definitions (VLA, LLM, NLP, API)
- [ ] T085 [P] Standardize terminology across Module 4 chapters

### Cross-Linking & References

- [ ] T086 [P] Add 5-7 internal cross-links per Module 4 chapter to Module 1
- [ ] T087 [P] Add 2-4 internal cross-links per Module 4 chapter to Module 3 (perception feedback)
- [ ] T088 [P] Cross-link between all Module 4 chapters (Ch1 → Ch2 → Ch3 → Ch4)

### Diagram Creation & Validation

- [ ] T089 Create diagram: Whisper pipeline (Audio → Spectrogram → Text) in M4Ch1
- [ ] T090 [P] Create diagram: LLM intent extraction (Text → Intent + Entities) in M4Ch2
- [ ] T091 [P] Create diagram: Action Server lifecycle in M4Ch3
- [ ] T092 [P] Create diagram: Complete VLA end-to-end flow in M4Ch4
- [ ] T093 [P] Create diagram: Perception feedback loop in M4Ch4
- [ ] T094 [P] Validate all Mermaid syntax and ASCII diagrams

### RAG Preparation

- [ ] T095 [P] Add YAML frontmatter to all M4 chapters (14-field schema)
- [ ] T096 [P] Identify natural chunk boundaries in M4 chapters
- [ ] T097 [P] Add metadata headers to chunk boundaries

### Quality Assurance

- [ ] T098 Run Flesch-Kincaid check on all M4 chapters (document in `QA_REPORT_M4.md`)
- [ ] T099 [P] Verify Whisper, LLM, and ROS 2 documentation references
- [ ] T100 [P] Test cross-module links (Module 4 → all other modules)

**Phase 4 Acceptance Criteria**:
- ✅ All M4 chapters 10-12 FK readability
- ✅ 3-5 diagrams per chapter (text-based)
- ✅ YAML frontmatter on all chapters
- ✅ Cross-links to Modules 1-3 established
- ✅ 95%+ cross-links working

---

# PHASE 5: Data Model & Contracts (Weeks 5-6)

**Goal**: Define RAG chunk structure, API contracts, and database schema.

## Data Model Tasks

- [ ] T101 Create `specs/004-vla-pipeline/data-model.md` with complete schema (Chunk, User, ChatHistory entities)
- [ ] T102 [P] Define Chunk metadata schema (id, chapter_id, section, content, embedding, url_anchor)
- [ ] T103 [P] Define User entity (id, email, preferences, created_at)
- [ ] T104 [P] Define ChatHistory entity (id, user_id, query, chunks, response, feedback)

## API Contract Tasks

- [ ] T105 Create `specs/004-vla-pipeline/contracts/api-rag-chatbot.yaml` for RAG endpoints
- [ ] T106 [P] Document `POST /api/chat` contract (query, user_id, context_override → response, chunks, citations)
- [ ] T107 [P] Document `GET /api/chunks` contract (chapter, keyword filters)
- [ ] T108 [P] Document `POST /api/auth/signup` contract (BetterAuth integration)
- [ ] T109 [P] Document `PUT /api/user/preferences` contract (difficulty, language, theme)
- [ ] T110 [P] Document error handling and status codes for all endpoints

## Database Schema Tasks

- [ ] T111 Create SQL schema for Neon Postgres in `backend/schema/schema.sql`
- [ ] T112 [P] Define `chunks` table (id, chapter_id, section, content, metadata JSONB, embedding VECTOR, url)
- [ ] T113 [P] Define `users` table (id, email, auth_method, preferences JSONB, created_at)
- [ ] T114 [P] Define `chat_history` table (id, user_id, query, chunks JSONB, response, feedback JSONB, created_at)

## Quickstart Documentation

- [ ] T115 Create `specs/004-vla-pipeline/quickstart.md` with setup instructions
- [ ] T116 [P] Document local Docusaurus build: `npm install && npm run start`
- [ ] T117 [P] Document local FastAPI: `pip install -r requirements.txt && uvicorn app.main:app --reload`
- [ ] T118 [P] Document test RAG query: `curl -X POST http://localhost:8000/api/chat -d '{"query":"What is ROS 2?"}'`

**Phase 5 Acceptance Criteria**:
- ✅ Complete data model with all entities and relationships
- ✅ All API contracts fully documented with examples
- ✅ Database schema ready for implementation
- ✅ Quickstart guide enables local development setup in <15 minutes

---

# PHASE 6: Content Validation & Tooling (Week 6)

**Goal**: Build automated quality checks and validation pipeline.

## Readability Checker Tasks

- [ ] T119 Create `tools/readability-check.py` that computes Flesch-Kincaid per chapter
- [ ] T120 [P] Implement jargon density analysis (acronym counting)
- [ ] T121 [P] Generate report flagging chapters >12 FK
- [ ] T122 [P] Test on all 15 chapters, generate `READABILITY_REPORT_2025-12-09.md`

## Link Validator Tasks

- [ ] T123 Create `tools/validate-links.py` that parses all cross-references
- [ ] T124 [P] Verify all internal links (Module 1 → Module 3, Chapter X → Chapter Y)
- [ ] T125 [P] Verify all external URLs (OpenAI, ROS 2, Isaac docs)
- [ ] T126 [P] Generate report for broken links, generate `BROKEN_LINKS_2025-12-09.md`

## Diagram Validator Tasks

- [ ] T127 Create `tools/validate-diagrams.py` that checks Mermaid/ASCII syntax
- [ ] T128 [P] Verify no external image references
- [ ] T129 [P] Generate report for broken diagrams

## GitHub Actions Workflow Tasks

- [ ] T130 Create `.github/workflows/lint-content.yml` GitHub Actions workflow
- [ ] T131 [P] Configure workflow to run on PR to `main` and `dev`
- [ ] T132 [P] Execute readability, link, and diagram checks
- [ ] T133 [P] Add PR comments with check results
- [ ] T134 [P] Test workflow on sample PR

**Phase 6 Acceptance Criteria**:
- ✅ All content validators working and tested
- ✅ GitHub Actions workflow integrated
- ✅ All 15 chapters pass readability check (10-12 FK)
- ✅ 95%+ cross-links validated
- ✅ Zero broken diagrams

---

# PHASE 7: RAG Chunking & Embeddings (Week 7)

**Goal**: Convert chapters into RAG chunks and generate embeddings.

## Chunking Script Tasks

- [ ] T135 Create `tools/chunk-content.py` that splits chapters into RAG chunks
- [ ] T136 [P] Implement 512 ± 100 token chunking logic
- [ ] T137 [P] Implement 20% overlap between consecutive chunks
- [ ] T138 [P] Extract and preserve metadata (chapter, section, keywords, learning objectives)
- [ ] T139 [P] Test on Module 1 chapters (generate sample chunks JSON)
- [ ] T140 [P] Test on all 15 chapters, generate complete `chunks/all-chapters-chunks.json`

## Embedding Generation Tasks

- [ ] T141 Create `tools/generate-embeddings.py` using OpenAI embedding API
- [ ] T142 [P] For each chunk, generate 1536-dim vector using `text-embedding-3-small`
- [ ] T143 [P] Store embeddings in `data/embeddings/chunk-embeddings.json`
- [ ] T144 [P] Track cost and token usage

## Chunk Registry Tasks

- [ ] T145 Create `tools/data/chunk-metadata.json` registry with all chunk IDs, vectors, URLs
- [ ] T146 [P] Validate chunk count: ~150-200 per chapter × 15 chapters = 2250-3000 total
- [ ] T147 [P] Generate summary statistics: chunks per chapter, token distribution

**Phase 7 Acceptance Criteria**:
- ✅ 2250-3000 chunks generated across all 15 chapters
- ✅ All chunks 400-800 tokens with 20% overlap
- ✅ All chunks have full metadata (chapter, section, keywords, URL)
- ✅ All embeddings generated and stored
- ✅ Chunk registry validated

---

# PHASE 8: Docusaurus Configuration & Site Build (Week 8)

**Goal**: Create production-ready Docusaurus site with all modules.

## Configuration Tasks

- [ ] T148 Update `my-website/docusaurus.config.ts` with correct project name and URLs
- [ ] T149 [P] Set title: "Humanoid Robotics Textbook"
- [ ] T150 [P] Set tagline: "Learn ROS 2, Simulation, Perception, and VLA"
- [ ] T151 [P] Enable Mermaid diagram support
- [ ] T152 [P] Configure search plugin (Docusaurus built-in)

## Sidebar & Navigation Tasks

- [ ] T153 Update `my-website/sidebars.ts` for 4-module structure
- [ ] T154 [P] Add navbar items: "Module 1 | Module 2 | Module 3 | Module 4"
- [ ] T155 [P] Add glossary entry to shared sidebar
- [ ] T156 [P] Test navigation across all modules and chapters

## Homepage Tasks

- [ ] T157 Create `my-website/src/pages/index.tsx` homepage component
- [ ] T158 [P] Design hero section: "Master Humanoid Robotics"
- [ ] T159 [P] Create 4 module cards with descriptions and learning hours
- [ ] T160 [P] Add "Start Reading" CTA button → `/docs/intro`
- [ ] T161 [P] Add links to glossary, GitHub repo, chatbot (when live)

## CSS & Theming Tasks

- [ ] T162 Update `my-website/src/css/custom.css` for 4 modules
- [ ] T163 [P] Define color scheme for each module (distinct but cohesive)
- [ ] T164 [P] Ensure responsive design (mobile, tablet, desktop)
- [ ] T165 [P] Style code blocks for pseudo-code examples

## Build & Validation Tasks

- [ ] T166 Run `npm run build` and validate successful build
- [ ] T167 [P] Run Lighthouse audit on homepage and sample chapters
- [ ] T168 [P] Verify Lighthouse >90 on performance, accessibility, best practices, SEO
- [ ] T169 [P] Test all cross-links work in built site
- [ ] T170 [P] Generate build report: `BUILD_REPORT_2025-12-09.md`

**Phase 8 Acceptance Criteria**:
- ✅ Docusaurus builds successfully with no errors
- ✅ All 15 chapters render correctly
- ✅ Navigation works across all modules
- ✅ Lighthouse >90 on all metrics
- ✅ <2s TTL on homepage and sample chapters
- ✅ 95%+ cross-links working

---

# PHASE 9: FastAPI Backend & RAG Core (Weeks 9-10)

**Goal**: Build RAG infrastructure and API server.

## Database Setup Tasks

- [ ] T171 Create Neon Postgres project (or use existing)
- [ ] T172 [P] Execute schema migration: `psql -f backend/schema/schema.sql`
- [ ] T173 [P] Verify tables created: `chunks`, `users`, `chat_history`
- [ ] T174 [P] Test connection from FastAPI

## Vector Store Setup Tasks

- [ ] T175 Create Qdrant collection: "chunks" (vector size 1536)
- [ ] T176 [P] Load all embeddings from Phase 7 into Qdrant
- [ ] T177 [P] Test similarity search: Query 5 random chunks, verify results

## FastAPI Server Tasks

- [ ] T178 Create `backend/app/main.py` FastAPI application
- [ ] T179 [P] Implement dependency injection (DB, Qdrant client)
- [ ] T180 [P] Implement CORS settings for frontend
- [ ] T181 [P] Add health check endpoint: `GET /health`

## RAG Chat Endpoint Tasks

- [ ] T182 Implement `POST /api/chat` endpoint
- [ ] T183 [P] Embed user query using OpenAI embedding API
- [ ] T184 [P] Search Qdrant for top-5 similar chunks
- [ ] T185 [P] Build LLM prompt with chunk context
- [ ] T186 [P] Call OpenAI LLM (GPT-3.5-turbo, temperature=0)
- [ ] T187 [P] Verify response cites chunks
- [ ] T188 [P] Return response + chunk citations

## Chunk Retrieval Endpoint Tasks

- [ ] T189 Implement `GET /api/chunks` endpoint
- [ ] T190 [P] Support chapter filter: `?chapter=module1/chapter1`
- [ ] T191 [P] Support keyword filter: `?keyword=nodes`
- [ ] T192 [P] Return chunks with URLs

## Auth Endpoint Tasks (Phase 9, Optional)

- [ ] T193 Implement `POST /api/auth/signup` endpoint (BetterAuth)
- [ ] T194 [P] Implement `PUT /api/user/preferences` endpoint

## Testing Tasks

- [ ] T195 Create `backend/tests/test_rag.py` for RAG flow tests
- [ ] T196 [P] Test embedding generation
- [ ] T197 [P] Test similarity search with sample queries
- [ ] T198 [P] Test full RAG pipeline (query → response)
- [ ] T199 Create `backend/tests/test_chunks.py` for chunk retrieval tests
- [ ] T200 [P] Test chunk filtering by chapter and keyword
- [ ] T201 Create `backend/tests/test_api.py` for endpoint tests
- [ ] T202 [P] Test health check, error handling
- [ ] T203 [P] Run full test suite: `pytest backend/tests/`

**Phase 9 Acceptance Criteria**:
- ✅ Neon Postgres schema created and tested
- ✅ Qdrant populated with all 2250-3000 embeddings
- ✅ FastAPI server starts and health check passes
- ✅ RAG chat endpoint returns accurate responses with citations
- ✅ All endpoints tested with >90% coverage

---

# PHASE 10: RAG Chatbot UI & Integration (Weeks 10-11)

**Goal**: Build frontend chatbot interface and integrate with backend.

## Chatbot Widget Tasks

- [ ] T204 Create React component `my-website/src/components/ChatbotWidget.tsx`
- [ ] T205 [P] Design floating button (bottom-right) with chat icon
- [ ] T206 [P] Create message UI (user query, AI response, citations)
- [ ] T207 [P] Implement message streaming (show response as it arrives)
- [ ] T208 [P] Add citation links (clickable to chapter/section)

## Chatbot Integration Tasks

- [ ] T209 Create `my-website/src/services/chatbotService.ts` to call `/api/chat`
- [ ] T210 [P] Pass current page context (module, chapter) for better relevance
- [ ] T211 [P] Implement response caching (cache 20 recent queries)
- [ ] T212 [P] Add error handling (network errors, timeout, irrelevant queries)

## Error Handling Tasks

- [ ] T213 Implement error message: "Sorry, can't reach the chatbot. Try again?"
- [ ] T214 [P] Implement OOB message: "I can only answer questions about the book content."
- [ ] T215 [P] Implement rate limiting: 10 queries/minute per user (free tier)

## User Feedback Tasks

- [ ] T216 Add "Was this helpful?" buttons (thumbs up/down)
- [ ] T217 [P] Send feedback to `/api/feedback` endpoint (Phase 11)
- [ ] T218 [P] Track feedback in database for analytics

## Testing Tasks

- [ ] T219 Create component tests for ChatbotWidget
- [ ] T220 [P] Test message rendering, streaming, citation links
- [ ] T221 [P] Test error scenarios (network, timeout, OOB)
- [ ] T222 [P] Manual E2E test: Ask 10 sample questions, verify responses

**Phase 10 Acceptance Criteria**:
- ✅ Chatbot widget renders on all pages
- ✅ Chat endpoint communicates with backend successfully
- ✅ Responses stream in real-time
- ✅ Citations link to correct chapters
- ✅ Error handling works correctly
- ✅ Rate limiting enforced

---

# PHASE 11: Deployment Pipeline (Week 11)

**Goal**: Set up GitHub Pages + CI/CD for Docusaurus and FastAPI.

## Docusaurus Deployment Tasks

- [ ] T223 Create `.github/workflows/build-docs.yml` GitHub Actions workflow
- [ ] T224 [P] Trigger on push to `main` branch
- [ ] T225 [P] Checkout repo, install dependencies
- [ ] T226 [P] Run lint checks (Phase 6 tools)
- [ ] T227 [P] Build Docusaurus (`npm run build`)
- [ ] T228 [P] Deploy to GitHub Pages (auto-create `gh-pages` branch)

## GitHub Pages Configuration Tasks

- [ ] T229 Configure GitHub Pages settings to use `gh-pages` branch
- [ ] T230 [P] Set base URL: `https://shahb.github.io/hackthon_humanoid_book/`
- [ ] T231 [P] Test live site loads, all links work

## FastAPI Deployment Tasks

- [ ] T232 Create `.github/workflows/deploy-api.yml` for FastAPI deployment
- [ ] T233 [P] Deploy FastAPI to cloud (AWS/GCP/Vercel) OR Docker
- [ ] T234 [P] Configure environment variables (OpenAI API key, Neon DB, Qdrant)
- [ ] T235 [P] Set up health checks and monitoring

## Documentation Tasks

- [ ] T236 Create `DEPLOYMENT.md` with deployment instructions
- [ ] T237 [P] Document Docusaurus deployment process
- [ ] T238 [P] Document FastAPI deployment process
- [ ] T239 [P] Create `CONTRIBUTING.md`: How to update chapters, add modules
- [ ] T240 [P] Create `ARCHITECTURE.md`: System design, RAG pipeline, API contracts

**Phase 11 Acceptance Criteria**:
- ✅ Docusaurus site live on GitHub Pages
- ✅ FastAPI deployed and accessible
- ✅ All CI/CD workflows passing
- ✅ Deployment documentation complete

---

# PHASE 12: Optional Features — Auth & Personalization (Week 12)

**Goal**: Add user accounts, preferences, and personalized content (optional for MVP).

## BetterAuth Integration Tasks

- [ ] T241 [P] Implement `POST /api/auth/signup` (email, social login)
- [ ] T242 [P] Implement JWT token generation and validation
- [ ] T243 [P] Add auth middleware to protected endpoints

## User Preferences Tasks

- [ ] T244 [P] Implement difficulty level selection (beginner, intermediate, advanced)
- [ ] T245 [P] Implement language selection (English, Urdu Phase 13)
- [ ] T246 [P] Implement theme selection (light, dark)
- [ ] T247 [P] Store preferences in `users.preferences JSONB` column

## Personalized Content Tasks

- [ ] T248 [P] Show dynamic hints: "Skip this if you've completed Module 1"
- [ ] T249 [P] Recommend next chapters based on progress
- [ ] T250 [P] Implement chapter bookmarking

## Chat History Tasks

- [ ] T251 [P] Save user queries & responses in `chat_history` table
- [ ] T252 [P] Allow replay/refinement of past questions
- [ ] T253 [P] Implement analytics: What questions users ask (aggregate, anonymized)

**Phase 12 Acceptance Criteria** (Optional):
- ✅ User authentication working (email signup/login)
- ✅ User preferences saved and applied
- ✅ Personalized content showing correctly
- ✅ Chat history preserved per user

---

# PHASE 13: Multi-Language Support (Week 13, Bonus)

**Goal**: Add Urdu translation for key sections (bonus feature).

## Translation Tasks

- [ ] T254 [P] Translate chapter introductions to Urdu
- [ ] T255 [P] Translate section headings to Urdu
- [ ] T256 [P] Translate glossary terms to Urdu

## Docusaurus i18n Tasks

- [ ] T257 [P] Enable i18n in `docusaurus.config.ts`: `i18n.locales: ['en', 'ur']`
- [ ] T258 [P] Add language toggle in navbar
- [ ] T259 [P] Configure fallback to English for untranslated sections

## Glossary Translation Tasks

- [ ] T260 [P] Create dual-language glossary table (English ↔ Urdu)

**Phase 13 Acceptance Criteria** (Bonus):
- ✅ Urdu translations complete for key sections
- ✅ Language toggle working in navbar
- ✅ Dual-language glossary live

---

# PHASE 14: Quality Assurance & Testing (Week 11-12)

**Goal**: Full QA before production launch.

## External Validation Tasks

- [ ] T261 [P] Robotics expert review: Verify technical accuracy
- [ ] T262 [P] Educator review: Verify pedagogical soundness
- [ ] T263 [P] Sample reader testing with beginner students
- [ ] T264 [P] Create feedback report: `QA_FEEDBACK_REPORT_2025-12-09.md`

## Link Verification Tasks

- [ ] T265 Re-run link validator on all chapters
- [ ] T266 [P] Fix any broken external URLs
- [ ] T267 [P] Verify 95%+ links working

## RAG Testing Tasks

- [ ] T268 Test 50+ sample queries across all modules
- [ ] T269 [P] Verify chatbot only answers from book content
- [ ] T270 [P] Verify citations are accurate
- [ ] T271 [P] Generate query test report: `RAG_TEST_REPORT_2025-12-09.md`

## Performance Testing Tasks

- [ ] T272 Load test FastAPI (100 concurrent users)
- [ ] T273 [P] Measure latency: Chat response <3s
- [ ] T274 [P] Run Lighthouse on all pages
- [ ] T275 [P] Verify Lighthouse >90 on all metrics
- [ ] T276 [P] Generate performance report: `PERFORMANCE_REPORT_2025-12-09.md`

**Phase 14 Acceptance Criteria**:
- ✅ All critical bugs fixed
- ✅ 95%+ cross-links working
- ✅ RAG accuracy verified (50+ queries tested)
- ✅ Performance targets met (<3s RAG, >90 Lighthouse)
- ✅ QA reports completed

---

# PHASE 15: Launch & Documentation (Week 13)

**Goal**: Go live and document the system.

## Documentation Tasks

- [ ] T277 Create root `README.md`: Project overview, links to live site & chatbot
- [ ] T278 [P] Create `DEPLOYMENT.md`: Deployment instructions
- [ ] T279 [P] Create `CONTRIBUTING.md`: How to update chapters
- [ ] T280 [P] Create `ARCHITECTURE.md`: System design, RAG pipeline

## Pre-Launch Checklist Tasks

- [ ] T281 Verify all 15 chapters polished & verified
- [ ] T282 [P] Docusaurus builds & deploys to GitHub Pages
- [ ] T283 [P] FastAPI running on cloud
- [ ] T284 [P] Chatbot fully functional
- [ ] T285 [P] Links working, readability passing
- [ ] T286 [P] Mobile responsive

## Soft Launch Tasks

- [ ] T287 Open to beta users (friends, classmates)
- [ ] T288 [P] Gather UX feedback
- [ ] T289 [P] Fix critical issues from beta feedback
- [ ] T290 [P] Create launch checklist: `LAUNCH_CHECKLIST_2025-12-09.md`

**Phase 15 Acceptance Criteria**:
- ✅ Live site accessible
- ✅ Live chatbot functional
- ✅ All documentation complete
- ✅ Beta user feedback incorporated

---

# PHASE 16: Iteration & Improvement (Week 13+)

**Goal**: Continuous improvement based on user feedback.

## Feedback Loop Tasks

- [ ] T291 Monitor chatbot analytics: What questions users ask
- [ ] T292 [P] Monitor site analytics: Which chapters most read, bounce rates
- [ ] T293 [P] Collect user feedback via surveys
- [ ] T294 [P] Create monthly improvement report

## Content Update Tasks

- [ ] T295 Fix typos and errors (reported by users)
- [ ] T296 [P] Clarify confusing sections (based on chatbot queries)
- [ ] T297 [P] Add new diagrams or examples if gaps found

## Feature Roadmap Tasks

- [ ] T298 Design interactive quizzes
- [ ] T299 [P] Design code sandbox (run examples in browser)
- [ ] T300 [P] Design video supplements
- [ ] T301 [P] Plan advanced topics module (Module 5)

---

## Summary Table

| Phase | Goal | Duration | Task Count | MVP? |
|-------|------|----------|-----------|------|
| 0 | Research & Validation | Week 1 | 15 | ✅ |
| 1 | Module 1 Refinement | Weeks 1-2 | 19 | ✅ |
| 2 | Module 2 Refinement | Weeks 2-3 | 21 | ❌ |
| 3 | Module 3 Refinement | Weeks 3-4 | 23 | ❌ |
| 4 | Module 4 Refinement | Weeks 4-5 | 22 | ❌ |
| 5 | Data Model & Contracts | Weeks 5-6 | 15 | ❌ |
| 6 | Content Validation & Tooling | Week 6 | 16 | ❌ |
| 7 | RAG Chunking & Embeddings | Week 7 | 13 | ❌ |
| 8 | Docusaurus Build | Week 8 | 23 | ❌ |
| 9 | FastAPI Backend | Weeks 9-10 | 32 | ❌ |
| 10 | Chatbot UI | Weeks 10-11 | 19 | ❌ |
| 11 | Deployment | Week 11 | 18 | ❌ |
| 12 | Auth & Personalization | Week 12 | 14 | ❌ |
| 13 | Multi-Language | Week 13 | 7 | ❌ |
| 14 | QA & Testing | Weeks 11-12 | 16 | ❌ |
| 15 | Launch & Docs | Week 13 | 14 | ❌ |
| 16 | Iteration | Week 13+ | 11 | ❌ |
| **TOTAL** | | **13 weeks** | **~310** | |

---

## MVP Scope (Weeks 1-2)

**Recommended MVP** includes:
- ✅ Phase 0 (Research) — T001-T015
- ✅ Phase 1 (Module 1 Refinement) — T016-T034

**After MVP Completion**:
- Run Phases 2-4 (other modules) in parallel
- Then execute Phases 5-11 (infrastructure, deployment)

---

## Parallel Execution Opportunities

### Batch 1 (After Phase 0):
- Phase 1, 2, 3, 4 content refinement (all modules) — **4 threads**
- Phase 5 (data model & contracts) — **1 thread**

### Batch 2 (After Batch 1):
- Phase 6 (validation tooling) — **1 thread**
- Phase 7 (chunking & embeddings) — **1 thread**

### Batch 3 (After Phase 7):
- Phase 8 (Docusaurus build) — **1 thread**
- Phase 9 (FastAPI backend) — **1 thread**
- **Can run in parallel**

### Batch 4 (After Phases 8-9):
- Phase 10 (Chatbot UI) — **1 thread**
- Phase 11 (Deployment) — **1 thread**
- **Can run in parallel**

---

## Task Dependencies Graph

```
T001-T015 (Phase 0 Research)
   ↓
   ├─ T016-T034 (Phase 1: Module 1)
   ├─ T035-T055 (Phase 2: Module 2) — Parallel
   ├─ T056-T078 (Phase 3: Module 3) — Parallel
   ├─ T079-T100 (Phase 4: Module 4) — Parallel
   └─ T101-T118 (Phase 5: Data Model) — Parallel
      ↓
      ├─ T119-T134 (Phase 6: Validation)
      └─ T135-T147 (Phase 7: Chunking)
         ↓
         ├─ T148-T170 (Phase 8: Docusaurus)
         └─ T171-T203 (Phase 9: FastAPI)
            ↓
            ├─ T204-T222 (Phase 10: Chatbot UI)
            └─ T223-T240 (Phase 11: Deployment)
               ↓
               ├─ T241-T253 (Phase 12: Auth/Personalization)
               ├─ T254-T260 (Phase 13: Multi-Language)
               ├─ T261-T276 (Phase 14: QA)
               └─ T277-T290 (Phase 15: Launch)
                  ↓
                  T291-T301 (Phase 16: Iteration)
```

---

**Generated**: 2025-12-09 | **Total Tasks**: ~310 | **Estimated Duration**: 13 weeks
