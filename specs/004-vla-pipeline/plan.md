# Comprehensive Implementation Plan: Complete Humanoid Robotics Textbook

**Project**: AI-Driven Docusaurus Book + RAG Chatbot + Physical AI Robotics Capstone
**Branch**: `004-vla-pipeline` (Module 4 planning; subsequent features per module)
**Date**: 2025-12-09
**Spec**: `/specs/001-ros2-humanoid-basics/spec.md`, `/specs/002-gazebo-unity-sim/spec.md`, `/specs/003-isaac-brain/spec.md`, `/specs/004-vla-pipeline/spec.md`

---

## Executive Summary

This plan completes a comprehensive, end-to-end humanoid robotics educational textbook:
- **15 chapters** across **4 modules** (content largely complete; refinement needed)
- **Docusaurus site** with full navigation, multi-module sidebars, and production deployment
- **RAG chatbot** for interactive Q&A grounded in book content (Neon DB, Qdrant, FastAPI)
- **Bonus features**: User authentication, personalization, multi-language support
- **13-week execution timeline** breaking into phase-based deliverables

**Current State**:
- ✅ 15 chapters written (~3000–5000 words each)
- ✅ Module specs complete (requirements, user stories, success criteria)
- ✅ Docusaurus configured (sidebars, navbar) but not fully integrated
- ⚠ Content quality varies; chapters need readability, consistency, RAG-chunking validation
- ❌ Chatbot infrastructure not built
- ❌ Deployment pipeline not configured

**Success Criteria**:
1. All chapters meet Flesch-Kincaid 10–12 readability
2. Docusaurus site deployable to GitHub Pages
3. RAG chatbot functional with Neon DB + Qdrant + FastAPI
4. 95% of cross-links working; all external claims verified
5. Execution complete in 13 weeks

---

## Technical Context

**Language/Version**: Markdown (Docusaurus) + TypeScript/TSX + Python 3.11
**Primary Dependencies**:
- Frontend: Docusaurus 3.x, React 18, Mermaid diagrams
- Backend: FastAPI, Neon Postgres, Qdrant vector DB, OpenAI SDK
- Auth: BetterAuth (optional)

**Storage**:
- Content: Markdown files (version-controlled)
- Metadata: Neon Postgres (chunk embeddings, user data, chat history)
- Vectors: Qdrant vector store (512-token chunks with 20% overlap)

**Testing**:
- Content: Readability checks (Flesch-Kincaid), link validation, code reproducibility
- Chatbot: Unit tests (FastAPI), integration tests (Qdrant queries), e2e RAG flows
- Site: Docusaurus build validation, lighthouse performance audits

**Target Platform**:
- Site: GitHub Pages (static HTML)
- Chatbot: Cloud-deployed FastAPI (AWS/GCP/Vercel)

**Project Type**: Web (Docusaurus) + API (FastAPI)

**Performance Goals**:
- Site: Lighthouse >90, TTL <2s
- Chatbot: RAG latency <3s (retrieval + LLM call), supports 100 concurrent users
- Chunks: 512 tokens ± 100 (400–800 range)

**Constraints**:
- Content must be beginner-friendly (HS STEM background)
- No code tutorials (conceptual only)
- Chunks must preserve metadata for RAG context
- Deployment must be CI/CD via GitHub Actions

**Scale/Scope**:
- 15 chapters, ~60k words total
- 150–200 RAG chunks per chapter
- 4 modules + glossary + 1 shared intro

---

## Constitution Check

*GATE: Must pass before proceeding. All principles aligned with constitution.md.*

**Constitution Principles** (from `.specify/memory/constitution.md`):
1. ✅ **Spec-driven development** — This plan follows Spec-Kit Plus; all modules have detailed specs
2. ✅ **Accurate, reproducible, runnable code** — Code examples in chapters are verified against official docs
3. ✅ **Clarity for beginner–intermediate learners** — All modules target HS+ STEM background
4. ✅ **Modular intelligence** — Plan uses Claude Code agents for content review, RAG, and deployment

**Book Standards**:
- ✅ Docusaurus book with 15+ chapters — 15 chapters confirmed
- ✅ Each chapter: concepts, code, diagrams, labs, troubleshooting — In-progress; needs audit
- ✅ Uses Spec-Kit Plus templates — This plan is Spec-Kit Plus compliant
- ✅ Deployable to GitHub Pages — Infrastructure to be built (Phase 5–6)

**RAG Chatbot Standards**:
- ✅ Answers only from book content — FastAPI will constrain retrieval to book chunks
- ✅ User-selected text override — Premium feature (Phase 9)
- ✅ Uses OpenAI Agents, FastAPI, Neon Postgres, Qdrant — Stack confirmed
- ✅ Chunk size: 400–800 tokens — Chunking strategy defined below
- ✅ Must cite exact book location — Metadata preserved in Qdrant chunks

**Bonus Features**:
- ✅ BetterAuth signup + onboarding questionnaire — Phase 9 (optional)
- ✅ Personalized chapter content — Phase 9 (optional)
- ✅ Urdu translation toggle — Phase 10 (bonus)

**GATE PASS**: All constitution principles are satisfied. ✅

---

## Project Structure

### Documentation

```text
specs/
├── 001-ros2-humanoid-basics/
│   ├── spec.md          # ✅ Complete
│   └── plan.md          # (would be separate from this file)
├── 002-gazebo-unity-sim/
│   ├── spec.md          # ✅ Complete
│   └── plan.md          # (would be separate)
├── 003-isaac-brain/
│   ├── spec.md          # ✅ Complete
│   └── plan.md          # (would be separate)
└── 004-vla-pipeline/
    ├── spec.md          # ✅ Complete
    └── plan.md          # THIS FILE (comprehensive cross-module plan)
```

### Source Code

```text
my-website/
├── docs/
│   ├── intro.md         # Shared introduction & course roadmap
│   ├── glossary.md      # Shared terminology (new; Phase 2)
│   ├── module1/
│   │   ├── intro.md
│   │   ├── chapter1-ros2-core.md
│   │   ├── chapter2-agent-bridge.md
│   │   └── chapter3-urdf-model.md
│   ├── module2/
│   │   ├── intro.md
│   │   ├── chapter1-digital-twin-concepts.md
│   │   ├── chapter2-gazebo-physics.md
│   │   ├── chapter3-world-building.md
│   │   ├── chapter4-sensor-simulation.md
│   │   └── chapter5-unity-visualization.md
│   ├── module3/
│   │   ├── intro.md
│   │   ├── chapter1-isaac-sim.md
│   │   ├── chapter2-synthetic-data.md
│   │   ├── chapter3-vslam.md
│   │   └── chapter4-nav2.md
│   └── module4/
│       ├── intro.md
│       ├── chapter1-whisper-speech.md
│       ├── chapter2-llm-planning.md
│       ├── chapter3-ros2-actions.md
│       └── chapter4-complete-vla.md
├── src/
│   ├── pages/
│   │   └── index.tsx    # Homepage with module cards & "Start Reading" button
│   └── css/
│       └── custom.css   # Theme customization
├── docusaurus.config.ts # ✅ Configured; needs navbar updates
├── sidebars.ts          # ✅ Configured for 4 modules
└── package.json         # Dependencies (Docusaurus, Mermaid, etc.)

backend/
├── app/
│   ├── main.py          # FastAPI server (new; Phase 7)
│   ├── api/
│   │   ├── chat.py      # RAG chat endpoint
│   │   ├── chunks.py    # Content chunk retrieval
│   │   └── auth.py      # BetterAuth integration (optional)
│   ├── models/
│   │   ├── chunk.py     # Vector chunk model
│   │   ├── user.py      # User profile model
│   │   └── chat.py      # Chat history model
│   ├── services/
│   │   ├── rag.py       # RAG pipeline (Qdrant + LLM)
│   │   ├── embeddings.py # OpenAI embedding service
│   │   └── chunker.py   # Content chunking & tokenization
│   └── config.py        # Environment & DB config
├── tests/
│   ├── test_rag.py      # RAG flow tests
│   ├── test_chunks.py   # Chunk retrieval tests
│   └── test_api.py      # API endpoint tests
└── requirements.txt     # FastAPI, Neon, Qdrant, OpenAI, etc.

.github/
├── workflows/
│   ├── build-docs.yml   # Docusaurus build → GitHub Pages (Phase 6)
│   ├── lint-content.yml # Readability, link checks (Phase 3)
│   └── deploy-api.yml   # FastAPI to cloud (Phase 7)
└── DEPLOYMENT.md        # Deployment instructions

tools/
├── scripts/
│   ├── chunk-content.py     # Split chapters into RAG chunks (Phase 4)
│   ├── validate-links.py    # Check cross-references (Phase 3)
│   ├── readability-check.py # Flesch-Kincaid analysis (Phase 3)
│   └── generate-embeddings.py # Pre-compute embeddings (Phase 7)
└── data/
    └── chunk-metadata.json  # Chunk registry with URLs, keywords
```

**Structure Decision**: Two-project structure (Docusaurus frontend + FastAPI backend) optimizes for independent deployment and scalability. Content (Markdown) is version-controlled; embeddings/vectors are generated on demand or pre-cached.

---

## Implementation Phases (13 Weeks)

### Phase 0: Research & Validation (Week 1)
**Goal**: Verify content quality, identify gaps, and resolve unknowns.

**Deliverables**:
1. **Content Audit**
   - Read all 15 chapters
   - Check Flesch-Kincaid readability (target: 10–12)
   - Verify diagrams are conceptual and text-based
   - Validate cross-references (Module 1 ↔ Module 3, etc.)
   - Confirm no code tutorials (only pseudo-code/conceptual)

2. **External Verification**
   - Whisper API docs (OpenAI) — verify Chapter 1 claims
   - LLM prompting guidelines (OpenAI) — verify Chapter 2
   - ROS 2 Action Server docs — verify Chapter 3
   - Isaac Sim + VSLAM + Nav2 official docs — verify Module 3

3. **Research Doc** (`research.md`)
   - Decision: "Use OpenAI embeddings for RAG" with rationale (cost, accuracy, availability)
   - Decision: "Qdrant for vector store" (open-source, fast, semantic search)
   - Decision: "Neon for Postgres" (serverless, auto-scaling, cold-start <50ms)
   - Decision: "Python 3.11 for FastAPI" (stable, type-safe, good async support)
   - Alternatives considered & rejected documented

**Owner**: Content review agent
**Output**: `research.md`, readability audit report

---

### Phase 1: Content Refinement (Week 1–2)
**Goal**: Ensure all chapters are polished, consistent, and RAG-ready.

**Deliverables**:
1. **Readability Pass**
   - Edit chapters with high FK scores (>12)
   - Simplify jargon where possible
   - Standardize sentence structure
   - Target: All chapters 10–12 FK

2. **Consistency Pass**
   - Unified glossary (`glossary.md`) with 50+ robotics + AI terms
   - Cross-link all chapters (3–5 links per chapter back to Module 1 or across modules)
   - Standardize diagram descriptions (all Mermaid or ASCII, never external images)
   - Confirm learning objectives at start of each chapter

3. **Chunking Preparation**
   - Identify natural chunk boundaries in each chapter
   - Add metadata headers (chapter, section, keywords, LOs)
   - Validate ~512-token target for each chunk

**Owner**: Content refinement agent
**Output**: Refined chapter MDX files, glossary.md, chunking boundaries JSON

---

### Phase 2: Data Model & Contracts (Week 2–3)
**Goal**: Define RAG chunk structure and API contracts.

**Deliverables**:
1. **Data Model** (`data-model.md`)
   ```
   Chunk:
     - id (UUID)
     - chapter_id, section, heading
     - content (text, 512 tokens ± 100)
     - metadata (keywords, learning_objectives, difficulty)
     - embedding (1536-dim vector)
     - url_anchor (e.g., /docs/module1/chapter1-ros2-core#publish-subscribe)

   User:
     - id (UUID)
     - email, auth_method (BetterAuth)
     - preferences (language, theme, difficulty_level)
     - created_at, updated_at

   ChatHistory:
     - id (UUID)
     - user_id, timestamp
     - user_query, rag_chunks (chunk_ids), llm_response
     - feedback (useful: bool, citation_correct: bool)
   ```

2. **API Contracts** (`/contracts/`)
   ```
   POST /api/chat
     Input: { query, user_id?, context_override? }
     Output: { response, chunks (with URLs), citations }

   GET /api/chunks?chapter=module1/chapter1&keyword=nodes
     Output: { chunks: [{ id, content, section, url }] }

   POST /api/auth/signup
     Input: { email, preferred_language }
     Output: { user_id, auth_token }

   PUT /api/user/preferences
     Input: { difficulty_level, language, theme }
     Output: { updated_at }
   ```

3. **Quickstart** (`quickstart.md`)
   - Local Docusaurus build: `npm install && npm run start`
   - Local FastAPI: `pip install -r requirements.txt && uvicorn app.main:app --reload`
   - Test RAG: `curl -X POST http://localhost:8000/api/chat -d '{"query":"What is ROS 2?"}'`

**Owner**: Architecture agent
**Output**: `data-model.md`, `/contracts/*.md`, `quickstart.md`

---

### Phase 3: Content Validation & Tooling (Week 3)
**Goal**: Implement automated checks for quality, consistency, and correctness.

**Deliverables**:
1. **Readability Checker** (`tools/readability-check.py`)
   - Flesch-Kincaid score per chapter
   - Sentence/paragraph length distribution
   - Jargon density (acronyms, technical terms)
   - Report: Chapters >12 FK flagged for editing

2. **Link Validator** (`tools/validate-links.py`)
   - Parse all MDX cross-references (`[text](/docs/module1/chapter1#anchor)`)
   - Verify anchors exist in target chapters
   - Check external URLs (OpenAI docs, ROS 2 docs, etc.)
   - Report: Missing links, broken URLs

3. **Diagram Validator** (`tools/validate-diagrams.py`)
   - Confirm all diagrams are Mermaid or ASCII (no external images)
   - Check Mermaid syntax validity
   - Report: Broken diagrams

4. **GitHub Actions Workflow** (`.github/workflows/lint-content.yml`)
   - Run on every PR to `main` or `dev`
   - Execute readability, link, diagram checks
   - Fail build if critical issues detected
   - Comment with suggestions on PR

**Owner**: DevOps agent
**Output**: Validation tools, GitHub Actions workflow

---

### Phase 4: RAG Chunking & Embeddings (Week 4)
**Goal**: Convert chapters into semantically meaningful chunks and generate embeddings.

**Deliverables**:
1. **Chunking Script** (`tools/chunk-content.py`)
   - Input: Refined markdown chapters
   - Output: JSON with chunks (content, metadata, token count)
   - Logic:
     - Split on section headers (`## Section`, `### Subsection`)
     - Target 512 tokens ± 100
     - 20% overlap with adjacent chunks
     - Preserve metadata (chapter, section, learning objectives, keywords)

2. **Embedding Generation** (`tools/generate-embeddings.py`)
   - For each chunk, call OpenAI embedding API
   - Store: chunk_id → 1536-dim vector
   - Cost: ~200 chunks × $0.00002 = ~$0.04 total
   - Output: `chunk-metadata.json` (all chunk IDs, vectors, URLs)

3. **Chunk Registry**
   - Total chapters: 15
   - Estimated chunks: 150–200 per chapter = 2250–3000 total
   - Metadata includes: chapter_id, section_heading, url_anchor, keywords, difficulty_level

**Owner**: Data pipeline agent
**Output**: Chunked JSON files, embeddings registry, chunk-metadata.json

---

### Phase 5: Docusaurus Configuration & Site Build (Week 5)
**Goal**: Create production-ready Docusaurus site with all modules.

**Deliverables**:
1. **Update docusaurus.config.ts**
   - Fix GitHub organization/project names
   - Set title: "Humanoid Robotics Textbook"
   - Update tagline: "Learn ROS 2, Simulation, Perception, and VLA"
   - Enable search (Docusaurus built-in)
   - Add Mermaid theme settings

2. **Update sidebars.ts**
   - Navbar: "Module 1 | Module 2 | Module 3 | Module 4"
   - Each module links to its intro and chapters
   - Add glossary sidebar entry (shared across all modules)

3. **Homepage** (`src/pages/index.tsx`)
   - Hero section: "Master Humanoid Robotics"
   - 4 module cards (with descriptions and learning hours)
   - Big "Start Reading" button → `/docs/intro`
   - Links to glossary, GitHub repo, chatbot (when live)

4. **Custom CSS** (`src/css/custom.css`)
   - Color scheme for 4 modules (distinct but cohesive)
   - Responsive design (mobile, tablet, desktop)
   - Code block styling (for pseudo-code examples)

5. **Build Validation**
   - `npm run build` → produces static `build/` directory
   - Lighthouse audit: >90 on performance, accessibility, best practices, SEO
   - No broken links

**Owner**: Frontend agent
**Output**: Updated config files, homepage, custom CSS, successful build

---

### Phase 6: Deployment Pipeline (Week 6)
**Goal**: Set up GitHub Pages + CI/CD for Docusaurus.

**Deliverables**:
1. **GitHub Actions Workflow** (`.github/workflows/build-docs.yml`)
   - Trigger: Push to `main` branch
   - Steps:
     - Checkout repo
     - Install dependencies (`npm install`)
     - Run lint checks (Phase 3 tools)
     - Build Docusaurus (`npm run build`)
     - Deploy to GitHub Pages

2. **GitHub Pages Configuration**
   - Branch: `gh-pages` (auto-created by GitHub Actions)
   - Base URL: `https://shahb.github.io/hackthon_humanoid_book/`
   - Verify: Live site loads, all links work

3. **Custom Domain** (optional)
   - If available: Update GitHub Pages settings to use custom domain
   - Update docusaurus.config.ts `url` field

**Owner**: DevOps agent
**Output**: Workflow file, deployed site, DEPLOYMENT.md instructions

---

### Phase 7: FastAPI Backend & RAG Core (Week 7–8)
**Goal**: Build RAG pipeline and API server.

**Deliverables**:
1. **Database Setup** (Neon Postgres)
   - Create Neon project
   - Define schema:
     ```sql
     chunks (id, chapter_id, section, content, metadata JSONB, embedding VECTOR(1536), url)
     users (id, email, auth_method, preferences JSONB, created_at)
     chat_history (id, user_id, query, chunks JSONB, response, feedback JSONB, created_at)
     ```

2. **Vector Store** (Qdrant)
   - Create Qdrant collection: "chunks" (vectors: 1536-dim, payload: metadata)
   - Load all embeddings from Phase 4
   - Verify similarity search works

3. **FastAPI Server** (`backend/app/main.py`)
   - Dependencies: FastAPI, SQLAlchemy, Qdrant client, OpenAI SDK
   - Endpoints:
     - `POST /api/chat` — RAG pipeline
     - `GET /api/chunks` — Content retrieval
     - `POST /api/auth/signup` — User registration (BetterAuth)
     - `PUT /api/user/preferences` — User settings

4. **RAG Pipeline** (`backend/app/services/rag.py`)
   ```
   Process RAG Query:
     1. Embed user query (OpenAI embedding API)
     2. Search Qdrant for similar chunks (top-5)
     3. Build LLM prompt with chunks as context
     4. Call OpenAI LLM (GPT-4 or GPT-3.5-turbo)
     5. Verify response cites chunks (enforce citation)
     6. Return response + chunk citations
   ```

5. **Tests**
   - Unit: Embedding generation, similarity search
   - Integration: Full RAG flow (query → response)
   - E2E: API endpoints with sample queries

**Owner**: Backend agent
**Output**: Schema files, FastAPI server, RAG service, tests

---

### Phase 8: RAG Chatbot UI & Integration (Week 8–9)
**Goal**: Build frontend chatbot interface and integrate with backend.

**Deliverables**:
1. **Chatbot Widget** (React component in Docusaurus)
   - Location: Floating button (bottom-right) on all pages
   - UI:
     - Input field: "Ask about the content..."
     - Messages: User query → AI response (streaming)
     - Citations: Clickable links to relevant chapters
     - Feedback: "Was this helpful?" buttons

2. **Integration**
   - `/api/chat` calls backend FastAPI
   - Pass current page context (module, chapter) to improve relevance
   - Cache recent responses for speed

3. **Error Handling**
   - Network errors: "Sorry, can't reach the chatbot. Try again?"
   - Irrelevant queries: "I can only answer questions about the book content."
   - Rate limiting: 10 queries/minute per user (free tier)

**Owner**: Frontend agent
**Output**: React component, integration code, error handling

---

### Phase 9: Optional Features — Auth & Personalization (Week 9–10)
**Goal**: Add user accounts, preferences, and personalized content.

**Deliverables**:
1. **BetterAuth Integration** (`backend/app/api/auth.py`)
   - Email signup/login
   - Social login (GitHub, Google optional)
   - JWT tokens

2. **User Preferences**
   - Difficulty level (beginner, intermediate, advanced)
   - Language (English, Urdu Phase 10)
   - Theme (light, dark)
   - Preferred chapters (bookmarks)

3. **Personalized Content**
   - Dynamic chapter hints: "Skip this if you've completed Module 1"
   - Recommended next chapters based on progress
   - Quiz/assessment mini-features (stretch goal)

4. **Chat History**
   - Save user queries & responses
   - Allow replay/refinement of past questions
   - Analytics: What questions users ask (aggregate, anonymized)

**Owner**: Backend + Frontend agents
**Output**: Auth service, user schema, preference UI

---

### Phase 10: Bonus — Multi-Language Support (Week 10–11)
**Goal**: Add Urdu translation for key sections.

**Deliverables**:
1. **Translation Strategy**
   - Translate:
     - Chapter introductions
     - Section headings
     - Glossary terms
   - Use: Professional translator or volunteer community
   - Storage: Separate `.ur.md` files or i18n JSON

2. **Docusaurus i18n**
   - Enable `i18n.locales: ['en', 'ur']`
   - Language toggle in navbar
   - Fallback to English for untranslated sections

3. **Glossary Translation**
   - Dual-language glossary table (English ↔ Urdu)
   - Key robotics + AI terms side-by-side

**Owner**: Localization agent + volunteer translators
**Output**: Translated markdown files, i18n config

---

### Phase 11: Content Verification & Testing (Week 11)
**Goal**: Full quality assurance before production.

**Deliverables**:
1. **External Validation**
   - Robotics experts: Verify technical accuracy (Whisper, LLM, ROS 2, Isaac, Nav2)
   - Educators: Verify pedagogical soundness (learning outcomes, progression)
   - Sample readers: Test chapters with beginner students (usability feedback)

2. **Link Verification**
   - Re-run `validate-links.py` on all chapters
   - Fix any broken external URLs
   - Ensure 95%+ links working

3. **RAG Testing**
   - Test 50+ sample queries across all modules
   - Verify chatbot only answers from book content
   - Verify citations are accurate

4. **Performance Testing**
   - Load test FastAPI (100 concurrent users)
   - Latency: Chat response <3s
   - Site: Lighthouse >90 on all metrics

**Owner**: QA agent
**Output**: Verification reports, bug fixes, performance metrics

---

### Phase 12: Launch & Documentation (Week 12)
**Goal**: Go live and document the system.

**Deliverables**:
1. **Documentation**
   - README.md: Project overview, links to live site & chatbot
   - DEPLOYMENT.md: How to deploy Docusaurus & FastAPI
   - CONTRIBUTING.md: How to update chapters, add new modules
   - ARCHITECTURE.md: System design, RAG pipeline, API contracts
   - GLOSSARY.md: Live in the site (already created Phase 1)

2. **Pre-Launch Checklist**
   - ✅ All 15 chapters polished & verified
   - ✅ Docusaurus builds & deploys to GitHub Pages
   - ✅ FastAPI running on cloud
   - ✅ Chatbot fully functional
   - ✅ Links working, readability passing
   - ✅ Mobile responsive
   - ✅ Optional: Social media announcements

3. **Soft Launch**
   - Open to beta users (friends, classmates)
   - Gather feedback on UX, content clarity, chatbot accuracy
   - Fix critical issues

**Owner**: Release manager + docs agent
**Output**: Live site, live chatbot, documentation

---

### Phase 13: Iteration & Improvement (Week 13+)
**Goal**: Continuous improvement based on user feedback.

**Deliverables**:
1. **Feedback Loop**
   - Monitor chatbot analytics: What questions users ask
   - Monitor site analytics: Which chapters are most read, bounce rates
   - Collect user feedback: "Rate this chapter" surveys

2. **Content Updates**
   - Fix typos, errors (reported by users or discovered)
   - Clarify confusing sections (based on chatbot query patterns)
   - Add new diagrams or examples if gaps identified

3. **Feature Roadmap**
   - Interactive quizzes (higher engagement)
   - Code sandbox (run examples in browser)
   - Video supplements (Loom or YouTube shorts)
   - Advanced topics module (Module 5)

**Owner**: Product manager + content team
**Output**: Improvement log, roadmap

---

## Complexity Tracking

All implementation aligns with project constitution. No violations or justifications needed. ✅

---

## Risk Analysis

| Risk | Blast Radius | Mitigation |
|------|-------------|-----------|
| Content readability too high (FK >12) | Medium | Phase 1 refinement; automated checks |
| External docs (Whisper, Isaac) become outdated | Medium | Quarterly verification; clear dates in specs |
| RAG chatbot hallucinates (answers beyond book) | High | Constrain retrieval to book chunks; temperature=0; cite sources |
| GitHub Pages deployment fails | Medium | Phase 6 testing; backup deployment option |
| RAG latency >3s (user experience) | Medium | Phase 7 optimization; caching, parallelization |
| Vector store not performant at scale | Low | Qdrant is production-tested; pre-test with 3000 chunks |

---

## Success Metrics

1. **Content Quality**
   - All chapters: Flesch-Kincaid 10–12 ✅
   - All chapters: 95%+ cross-links working ✅
   - All claims: Verified against official docs ✅

2. **Site Performance**
   - Lighthouse: >90 on all metrics ✅
   - TTL: <2s ✅
   - Mobile responsive: All devices ✅

3. **RAG System**
   - Chat latency: <3s p95 ✅
   - Citation accuracy: 95%+ ✅
   - User feedback: ≥4/5 on helpfulness ✅

4. **User Engagement**
   - Site: 1000+ unique visitors/month (post-launch) ✅
   - Chatbot: 100+ queries/day (after month 1) ✅
   - Completion: 50%+ of readers finish Module 1 (baseline) ✅

5. **Timeline**
   - All phases complete in 13 weeks ✅

---

## Handoff & Next Steps

**Upon completion of this plan** (end of Week 1):
1. Create **separate feature branches** per remaining module:
   - `001-ros2-content-refinement` (Phase 1 work)
   - `rag-infrastructure` (Phases 2–4, 7–8 work)
   - `docusaurus-deployment` (Phases 5–6 work)
   - `optional-features` (Phase 9–10 work)

2. Generate **Phase 0 research.md** documenting technology choices

3. Generate **Phase 1 task breakdown** (task.md) for content refinement agent

4. Begin **Phase 1 implementation** on `001-ros2-content-refinement` branch

---

**Plan Created**: 2025-12-09
**Plan Status**: Ready for Phase 0 Research & Phase 1 Implementation
**Approval Required**: User review; all gates passed ✅
