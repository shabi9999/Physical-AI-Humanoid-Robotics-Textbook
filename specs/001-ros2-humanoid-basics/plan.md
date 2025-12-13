# Implementation Plan: ROS 2 Fundamentals for Humanoid Robotics (Module 1)

**Branch**: `001-ros2-humanoid-basics` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/001-ros2-humanoid-basics/spec.md`

## Summary

This plan outlines the architecture and implementation strategy for Module 1 of an AI-driven Docusaurus educational book covering ROS 2 fundamentals for humanoid robotics. The module includes three chapters (ROS 2 Core Concepts, Python Agent Bridging, URDF Modeling) with integrated RAG chatbot for interactive learning. The system combines static educational content with AI-powered Q&A, deployable to GitHub Pages.

**Primary Requirement**: Create beginner-intermediate educational content with runnable ROS 2 code examples, deployable as a Docusaurus site with RAG chatbot for context-aware student assistance.

**Technical Approach**:
- **Frontend**: Docusaurus (docs-only mode) with Mermaid diagrams, custom text selection plugin
- **Backend**: FastAPI async server with OpenAI Agents SDK
- **Data**: Qdrant (vectors) + Neon Postgres (user data)
- **Deployment**: GitHub Actions → GitHub Pages (frontend), Railway/Fly.io (backend)

## Technical Context

**Language/Version**:
- Frontend: TypeScript 5.3+, Node.js 20 LTS
- Backend: Python 3.11+
- Educational Examples: Python 3.8+ (ROS 2 Humble compatibility)

**Primary Dependencies**:
- **Frontend**: Docusaurus 3.1+, React 18+, @docusaurus/theme-mermaid, prism-react-renderer
- **Backend**: FastAPI 0.109+, OpenAI Python SDK 1.12+, Qdrant Client 1.7+, asyncpg 0.29+
- **Educational**: ROS 2 Humble, rclpy, URDF tools (check_urdf, RViz)
- **RAG**: LangChain 0.1+, text-embedding-3-small (OpenAI), sentence-transformers (optional re-ranking)

**Storage**:
- **Vector Database**: Qdrant Cloud (managed) or self-hosted for embeddings
- **Relational Database**: Neon Postgres (serverless) for user auth, profiles, query history
- **Static Assets**: GitHub repository (code examples, diagrams, book content)

**Testing**:
- **Frontend**: Jest + React Testing Library (Docusaurus component tests)
- **Backend**: pytest + pytest-asyncio + httpx (FastAPI endpoint tests)
- **Code Examples**: pytest with ROS 2 Docker container (CI/CD validation)
- **RAG**: pytest with mock embeddings, retrieval accuracy metrics

**Target Platform**:
- **Frontend**: Static site hosted on GitHub Pages (cross-browser: Chrome, Firefox, Safari)
- **Backend**: Linux server (Railway/Fly.io) with Python 3.11 runtime
- **Educational Examples**: Ubuntu 22.04 (ROS 2 Humble base), Docker for portability

**Project Type**: Web application (static frontend + async backend API)

**Performance Goals**:
- **Page Load**: <2 seconds for initial chapter load (Docusaurus static site)
- **RAG Query**: <3 seconds end-to-end (embedding + retrieval + generation)
- **Search Throughput**: 20+ concurrent users with <100ms p95 response time (vector search)
- **Build Time**: <60 seconds for Docusaurus build (15+ chapters)

**Constraints**:
- **Educational**: All code examples must run on ROS 2 Humble (Ubuntu 22.04)
- **Markdown Only**: No proprietary formats (all content in .md files)
- **Accessibility**: WCAG 2.1 AA compliance (alt text, keyboard navigation)
- **Budget**: Minimize costs (free tier: GitHub Pages, Neon, Qdrant Cloud starter)
- **Offline Examples**: ROS 2 code must work without internet (only RAG requires connectivity)

**Scale/Scope**:
- **Content**: 3 chapters (Module 1), expandable to 15+ chapters (full book)
- **Code Examples**: ~15-20 runnable Python/URDF files
- **Users**: 100-500 concurrent learners (beginner-intermediate audience)
- **Embeddings**: ~200-300 chunks (512 tokens each) for Module 1
- **Query Load**: 100-200 RAG queries/day initially

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Required Principles (from constitution)

**✅ I. Spec-driven development using Spec-Kit Plus**
- Plan follows `/sp.plan` workflow with phases 0-2
- All artifacts stored in `specs/001-ros2-humanoid-basics/`
- Testable tasks will be generated via `/sp.tasks` after plan approval

**✅ II. Accurate, reproducible, runnable code**
- All ROS 2 examples stored in `examples/` directory with README
- CI/CD validates examples in ROS 2 Humble Docker container
- Clear "Prerequisites" and "Expected Output" sections in every chapter

**✅ III. Clarity for beginner–intermediate AI/robotics learners**
- Content uses beginner-friendly language (no unexplained jargon)
- Progressive complexity: Chapter 1 (basics) → Chapter 2 (integration) → Chapter 3 (modeling)
- Inline code comments explain every ROS 2 API call
- Troubleshooting sections for common errors

**✅ IV. Modular intelligence via Claude Code Subagents & Skills**
- Plan designed using `general-purpose` subagent for research
- Implementation will use `Explore` agent for codebase discovery
- RAG chatbot leverages OpenAI Agents SDK (modular AI architecture)

### Book Standards Compliance

**✅ Docusaurus book with 15+ chapters** (Module 1 = 3 chapters, foundation for full book)
**✅ Each chapter: concepts, code, diagrams, labs, troubleshooting**
**✅ Uses Spec-Kit Plus templates** (spec.md, plan.md, tasks.md)
**✅ Deployable to GitHub Pages** (GitHub Actions workflow included)

### RAG Chatbot Standards Compliance

**✅ Answers only from book content** (system prompt enforces source fidelity)
**✅ User-selected text override** (custom Docusaurus plugin)
**✅ Uses OpenAI Agents, FastAPI, Neon Postgres, Qdrant** (full stack as specified)
**✅ Chunk size: 400–800 tokens** (512 tokens with 20% overlap)
**✅ Must cite exact book location** (inline citations with chapter/section URLs)

### Robotics Content Requirements (Module 1 Coverage)

**✅ ROS 2 (nodes, topics, services, URDF)** - Core focus of Module 1
**⏭️ Gazebo physics & sensors** - Deferred to Module 2 (out of scope per spec FR-012)
**⏭️ Unity visualization** - Deferred to future modules
**⏭️ NVIDIA Isaac Sim + VSLAM + Nav2** - Deferred to advanced modules
**⏭️ VLA: Whisper, LLM planning, action execution** - Deferred to capstone project

**Compliance Status**: ✅ PASSED for Module 1 scope

## Project Structure

### Documentation (this feature)

```text
specs/001-ros2-humanoid-basics/
├── spec.md                    # Feature requirements (completed)
├── plan.md                    # This file (in progress)
├── tasks.md                   # Phase 2 output (/sp.tasks command - NOT YET CREATED)
├── checklists/
│   └── requirements.md        # Spec quality checklist (completed)
└── contracts/                 # API contracts (Phase 1 output)
    ├── rag-api.md             # RAG chatbot API specification
    └── docusaurus-plugin.md   # Text selection plugin interface
```

### Source Code (repository root)

```text
hackthon_humanoid_book/
├── my-website/                         # Docusaurus frontend
│   ├── docs/                           # Educational content (Markdown)
│   │   ├── intro.md                    # Course overview
│   │   └── module1/                    # Module 1: ROS 2 Fundamentals
│   │       ├── chapter1-ros2-core.md   # Chapter 1: Nodes, Topics, Services
│   │       ├── chapter2-agent-bridge.md # Chapter 2: rclpy Agent Bridging
│   │       └── chapter3-urdf-model.md  # Chapter 3: URDF Humanoid Modeling
│   ├── src/
│   │   ├── components/
│   │   │   ├── TextSelectionPlugin.tsx # User text selection for RAG override
│   │   │   └── RAGChatbot.tsx          # Chatbot UI component
│   │   ├── css/
│   │   │   └── custom.css              # Docusaurus theme overrides
│   │   └── pages/
│   │       └── index.tsx               # Optional landing page
│   ├── static/
│   │   ├── img/                        # Diagrams, robot photos
│   │   └── .nojekyll                   # Disable Jekyll (GitHub Pages)
│   ├── docusaurus.config.ts            # Docusaurus configuration
│   ├── sidebars.ts                     # Navigation structure
│   ├── package.json
│   └── tsconfig.json
│
├── backend/                            # FastAPI RAG backend
│   ├── src/
│   │   ├── api/
│   │   │   ├── auth.py                 # BetterAuth integration (future)
│   │   │   ├── query.py                # RAG query endpoint
│   │   │   └── feedback.py             # User feedback collection
│   │   ├── services/
│   │   │   ├── embeddings.py           # OpenAI embedding generation
│   │   │   ├── retrieval.py            # Qdrant vector search
│   │   │   ├── reranking.py            # Cross-encoder re-ranking (optional)
│   │   │   └── generation.py           # OpenAI Agents response generation
│   │   ├── db/
│   │   │   ├── postgres.py             # Neon Postgres connection pool
│   │   │   └── qdrant.py               # Qdrant client initialization
│   │   ├── models/
│   │   │   ├── query.py                # Pydantic models (QueryRequest, Response)
│   │   │   └── user.py                 # User schema (future auth)
│   │   └── utils/
│   │       ├── chunking.py             # Document chunking logic
│   │       └── logging.py              # Structured logging
│   ├── tests/
│   │   ├── test_api.py                 # FastAPI endpoint tests
│   │   ├── test_retrieval.py           # Vector search accuracy tests
│   │   └── test_chunking.py            # Chunking strategy validation
│   ├── main.py                         # FastAPI app entry point
│   ├── requirements.txt
│   └── Dockerfile
│
├── examples/                           # Runnable ROS 2 code
│   └── module1/
│       ├── chapter1/
│       │   ├── hello_ros2.py           # Minimal ROS 2 node
│       │   ├── publisher.py            # Topic publisher example
│       │   ├── subscriber.py           # Topic subscriber example
│       │   ├── service_server.py       # Service server example
│       │   ├── service_client.py       # Service client example
│       │   └── README.md               # Setup + run instructions
│       ├── chapter2/
│       │   ├── simple_agent.py         # Basic Python agent
│       │   ├── sensor_bridge.py        # Agent subscribing to sensors
│       │   ├── control_publisher.py    # Agent publishing commands
│       │   └── README.md
│       ├── chapter3/
│       │   ├── simple_humanoid.urdf    # Basic humanoid URDF
│       │   ├── visualize_urdf.py       # Load URDF in RViz
│       │   └── README.md
│       └── tests/
│           ├── test_chapter1.py        # Validate Chapter 1 examples
│           ├── test_chapter2.py        # Validate Chapter 2 examples
│           └── test_chapter3.py        # Validate Chapter 3 examples (URDF)
│
├── scripts/                            # RAG indexing + deployment
│   ├── index_content.py                # Chunk + embed Docusaurus content
│   ├── validate_build.py               # Pre-deployment checks
│   └── deploy.sh                       # Backend deployment script
│
├── .github/
│   └── workflows/
│       ├── deploy-docs.yml             # GitHub Pages deployment
│       ├── test-examples.yml           # ROS 2 example validation
│       └── test-backend.yml            # FastAPI + RAG tests
│
├── .specify/                           # Spec-Kit Plus templates
├── specs/                              # Feature specifications
├── history/                            # Prompt history records
└── README.md                           # Project overview
```

**Structure Decision**: Selected **Option 2: Web application** (frontend + backend separation) because:
1. **Clear Separation of Concerns**: Docusaurus (static educational content) vs FastAPI (dynamic RAG)
2. **Independent Deployment**: GitHub Pages (free, CDN) for frontend, Railway/Fly.io for backend
3. **Scalability**: Backend can be scaled independently based on RAG query load
4. **Development Workflow**: Frontend team (content creators) can work independently from backend team (RAG engineers)

Examples directory is at root level (not in my-website/) to emphasize that ROS 2 code is executable outside the documentation context.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations detected. All constitution principles are satisfied.

---

## Phase 0: Research & Dependencies

### Objectives
1. Confirm Docusaurus architecture decisions (docs-only vs full site, diagram tools)
2. Validate RAG chunking strategy (token size, overlap, document-aware splitting)
3. Research deployment pipelines (GitHub Actions for Pages, backend hosting)
4. Verify ROS 2 Humble compatibility with example code patterns

### Deliverables
- ✅ `research.md` with architectural recommendations (completed via subagent)
- Dependency list with versions locked to specific releases
- Risk assessment (ROS 2 version compatibility, embedding API costs, Qdrant limits)

### Key Findings (from Research Subagent)

**Docusaurus Architecture:**
- **Decision**: Docs-only mode with optional custom landing page
- **Rationale**: Learners should land directly on content (no blog noise), simpler navigation
- **Diagram Tool**: Mermaid (built-in) for 80% of diagrams, static images for complex architecture
- **Code Blocks**: Static with copy button (live code incompatible with ROS 2/Python)

**RAG Chunking:**
- **Chunk Size**: 512 tokens (within 400-800 range), 20% overlap (102 tokens)
- **Strategy**: Recursive character splitter with document-aware separators (preserve code blocks, headings)
- **Research Backing**: 512 tokens optimal for balancing factoid queries + analytical depth

**Vector Database:**
- **Decision**: Qdrant (separated from Neon Postgres)
- **Rationale**: Specialized performance (10x throughput vs pgVector), independent scaling
- **Trade-off**: Adds second data store but allows optimization per workload type

**Deployment:**
- **Frontend**: GitHub Actions → GitHub Pages (official deploy-pages@v4 action)
- **Backend**: Railway or Fly.io with Docker (FastAPI + Qdrant client)
- **Cost**: ~$0/month (GitHub Pages free, Neon free tier, Qdrant Cloud starter)

### Dependencies Matrix

| Category | Dependency | Version | Justification |
|----------|-----------|---------|---------------|
| **Frontend Framework** | Docusaurus | 3.1+ | Static site generator optimized for docs, Mermaid built-in |
| | React | 18+ | Required by Docusaurus, component-based UI |
| | TypeScript | 5.3+ | Type safety for custom plugins |
| **Diagrams** | @docusaurus/theme-mermaid | Latest | Built-in Mermaid support |
| | Prism React Renderer | Latest | Syntax highlighting for ROS 2 code |
| **Backend Framework** | FastAPI | 0.109+ | Async support, OpenAPI docs, production-ready |
| | Uvicorn | 0.27+ | ASGI server for FastAPI |
| **AI/RAG** | OpenAI Python SDK | 1.12+ | Embeddings + Agents SDK |
| | LangChain | 0.1+ | Document splitters, RAG utilities |
| | Qdrant Client | 1.7+ | Vector database client (async) |
| **Database** | asyncpg | 0.29+ | Async Postgres driver for Neon |
| | Psycopg3 | 3.1+ | Backup sync driver |
| **Educational (ROS 2)** | ROS 2 Humble | Latest stable | Target platform per spec FR-002 |
| | rclpy | Latest (Humble) | Python client library |
| | urdfdom-tools | Latest | URDF validation (check_urdf) |
| **Testing** | pytest | 7.4+ | Python test framework |
| | pytest-asyncio | 0.23+ | Async test support |
| | httpx | 0.26+ | Async HTTP client for FastAPI tests |
| | Jest | 29+ | JavaScript/React testing |

### Risks & Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|-----------|
| **ROS 2 Humble deprecation** | Low (2027 EOL) | High | Lock to Humble in Docker, document upgrade path |
| **OpenAI API cost overruns** | Medium | Medium | Implement rate limiting (10 queries/min), cache common queries |
| **Qdrant free tier limits** | Low | Medium | Monitor usage, plan migration to self-hosted if >1M vectors |
| **Code example failures** | Medium | High | CI/CD with ROS 2 Docker, validate every commit |
| **RAG hallucination** | Medium | High | Enforce "answer only from context" prompt, user feedback loop |
| **GitHub Pages build failures** | Low | Medium | Test locally with `npm run build`, broken link detection |

---

## Phase 1: Design

### Objectives
1. Define data model (Qdrant collections, Postgres schema)
2. Design API contracts (RAG query endpoint, text selection override)
3. Create quickstart guide for contributors
4. Design chapter structure templates

### Deliverables

#### 1. Data Model (`data-model.md`)

**Qdrant Collection: `ros2_book_content`**
```json
{
  "collection_name": "ros2_book_content",
  "vectors": {
    "size": 1536,
    "distance": "Cosine"
  },
  "schema": {
    "id": "UUID",
    "vector": "float[1536]",
    "payload": {
      "text": "string (chunk content)",
      "module": "string (e.g., 'module1')",
      "chapter": "string (e.g., 'chapter1-ros2-core')",
      "chapter_title": "string (e.g., 'ROS 2 Core Concepts')",
      "url": "string (e.g., '/module1/chapter1-ros2-core')",
      "chunk_index": "int (position in chapter)",
      "total_chunks": "int (chapter total)",
      "section_heading": "string (nearest H2/H3)",
      "content_type": "enum (text|code|diagram|table)",
      "created_at": "timestamp"
    }
  }
}
```

**Neon Postgres Schema** (future user features):
```sql
-- Users table (BetterAuth integration planned)
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    created_at TIMESTAMP DEFAULT NOW()
);

-- Query history for analytics
CREATE TABLE query_history (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE SET NULL,
    query TEXT NOT NULL,
    response TEXT,
    mode VARCHAR(20), -- 'rag' or 'override'
    retrieved_chunks JSONB, -- Array of chunk IDs
    feedback_rating INT CHECK (feedback_rating BETWEEN 1 AND 5),
    created_at TIMESTAMP DEFAULT NOW()
);

-- Indexes
CREATE INDEX idx_query_history_user_id ON query_history(user_id);
CREATE INDEX idx_query_history_created_at ON query_history(created_at DESC);
```

#### 2. API Contracts (`contracts/rag-api.md`)

**POST /api/query**

Request:
```json
{
  "query": "How do I create a ROS 2 node?",
  "user_id": "optional-uuid",
  "selected_text": "optional-user-highlighted-text"
}
```

Response:
```json
{
  "answer": "To create a ROS 2 node, use the rclpy library...",
  "sources": [
    {
      "chapter": "Chapter 1: ROS 2 Core Concepts",
      "url": "/module1/chapter1-ros2-core#creating-nodes",
      "excerpt": "A ROS 2 node is created by inheriting from rclpy.node.Node..."
    }
  ],
  "mode": "rag",
  "metadata": {
    "query_time_ms": 2847,
    "chunks_retrieved": 3
  }
}
```

Error Response:
```json
{
  "error": "rate_limit_exceeded",
  "message": "Maximum 10 queries per minute. Retry in 42 seconds.",
  "retry_after": 42
}
```

**GET /api/health**

Response:
```json
{
  "status": "healthy",
  "services": {
    "qdrant": "connected",
    "postgres": "connected",
    "openai": "connected"
  },
  "version": "1.0.0"
}
```

#### 3. Quickstart Guide (`quickstart.md`)

**For Content Contributors:**
```bash
# 1. Clone repository
git clone https://github.com/[username]/hackthon_humanoid_book.git
cd hackthon_humanoid_book

# 2. Install Docusaurus dependencies
cd my-website
npm install

# 3. Start local dev server
npm start
# Opens http://localhost:3000

# 4. Edit content in docs/module1/*.md
# Changes auto-reload in browser

# 5. Add code examples to examples/module1/
# Test locally with ROS 2 Humble

# 6. Build for production
npm run build
npm run serve
```

**For Backend Developers:**
```bash
# 1. Set up Python environment
cd backend
python3.11 -m venv venv
source venv/bin/activate

# 2. Install dependencies
pip install -r requirements.txt

# 3. Set environment variables
cp .env.example .env
# Edit .env with Qdrant/Neon/OpenAI credentials

# 4. Run FastAPI server
uvicorn main:app --reload
# Opens http://localhost:8000/docs (Swagger UI)

# 5. Index book content
cd ../scripts
python index_content.py --source ../my-website/docs

# 6. Test RAG query
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is a ROS 2 node?"}'
```

#### 4. Chapter Structure Template

Every chapter MUST follow this structure (enforces FR-011: Prerequisites section):

```markdown
---
sidebar_position: [N]
---

# Chapter [N]: [Title]

## Learning Objectives

By the end of this chapter, you will:
- [Concrete skill 1]
- [Concrete skill 2]
- [Concrete skill 3]

## Prerequisites

- **Knowledge**: [Required concepts from previous chapters]
- **Software**: ROS 2 Humble, Python 3.8+, [other tools]
- **Skills**: [Command line, Python basics, etc.]

## Concepts

### [Section 1]
[Conceptual explanation with beginner-friendly language]

#### Example: [Specific Use Case]
[Concrete example with context]

```mermaid
[Diagram illustrating concept]
```

### [Section 2]
[Progressive complexity building on Section 1]

## Hands-On: [Lab Name]

### Code Example: [Example Title]

```python title="filename.py" showLineNumbers
# Full runnable code with inline comments
import rclpy
# ...
```

**Run this example:**
```bash
# Setup commands
source ~/ros2_ws/install/setup.bash

# Execution command
python3 filename.py
```

**Expected Output:**
```
[INFO] [node_name]: Expected log message
```

**What's happening:**
1. Line X: [Explanation of key API call]
2. Line Y: [Explanation of important logic]

## Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| Error: "..." | [Common mistake] | [Fix with command/code] |

## Summary

- [Key takeaway 1]
- [Key takeaway 2]
- [Next chapter preview]

## Additional Resources

- [ROS 2 official docs](https://docs.ros.org/en/humble/)
- [Specific tutorial relevant to chapter]
```

---

## Phase 2: Task Breakdown

**NOTE**: Phase 2 (`/sp.tasks` command) will be executed AFTER this plan is approved. This section provides a preview of the task structure.

### Task Categories

**Phase A: Docusaurus Setup**
1. Initialize Docusaurus project (docs-only mode)
2. Configure Mermaid plugin
3. Set up GitHub Actions for Pages deployment
4. Create sidebar navigation for Module 1

**Phase B: Content Creation**
5. Write Chapter 1: ROS 2 Core Concepts
6. Write Chapter 2: Python Agent Bridging
7. Write Chapter 3: URDF Humanoid Modeling
8. Create Mermaid diagrams for each chapter

**Phase C: Code Examples**
9. Implement Chapter 1 examples (publisher, subscriber, service)
10. Implement Chapter 2 examples (agent bridging)
11. Implement Chapter 3 examples (URDF + RViz)
12. Set up pytest tests for all examples
13. Configure CI/CD for ROS 2 Docker validation

**Phase D: Backend RAG System**
14. Implement FastAPI app structure
15. Integrate Qdrant vector database
16. Integrate Neon Postgres
17. Implement document chunking pipeline
18. Implement embedding generation (OpenAI)
19. Implement retrieval service (vector search)
20. Implement OpenAI Agents response generation
21. Add rate limiting and caching
22. Write backend unit tests
23. Deploy backend to Railway/Fly.io

**Phase E: Frontend RAG Integration**
24. Create RAGChatbot React component
25. Implement TextSelectionPlugin for user overrides
26. Integrate chatbot with backend API
27. Add loading states and error handling
28. Test chatbot UX with real queries

**Phase F: Validation**
29. Run Docusaurus build validation
30. Test all internal links
31. Validate code example execution
32. Test RAG accuracy (precision/recall)
33. Accessibility audit (WCAG 2.1 AA)
34. Load testing (100 concurrent users)

### Acceptance Criteria

**Must Pass Before Merge:**
- [ ] All 3 chapters written with complete sections (concepts, code, diagrams, labs)
- [ ] All code examples run successfully in ROS 2 Humble Docker
- [ ] Docusaurus build completes without errors (`npm run build`)
- [ ] Zero broken links (internal or external)
- [ ] RAG chatbot returns accurate answers from book content (85%+ citation accuracy)
- [ ] Backend API responds within 3 seconds (p95)
- [ ] Accessibility: No WCAG 2.1 AA violations
- [ ] All tests passing (frontend Jest, backend pytest, example validation)

---

## Architectural Decision Records (ADRs)

The following architectural decisions were made during planning:

### ADR-001: Docusaurus Docs-Only Mode vs Full Site

**Decision**: Use docs-only mode with optional custom landing page.

**Context**: Educational book needs to prioritize learning content over marketing.

**Alternatives Considered**:
1. **Full site mode** (blog + docs + landing): ❌ Extra navigation steps, maintenance overhead
2. **Docs-only mode**: ✅ Direct access to content, simpler structure
3. **Custom Next.js site**: ❌ Reinventing the wheel, slower time-to-market

**Trade-offs**:
- ✅ **Pros**: Faster time-to-learning, simpler navigation, lower maintenance
- ❌ **Cons**: No built-in blog for announcements (can add later if needed)

**Rationale**: Beginner-intermediate learners benefit from reduced cognitive load. Landing directly on Chapter 1 is more effective than forcing navigation through marketing pages.

**Outcome**: Docs served at root (`/`), optional `src/pages/index.tsx` for course overview.

---

### ADR-002: Qdrant + Neon (Separated) vs pgVector (Unified)

**Decision**: Use Qdrant for vectors, Neon Postgres for relational data (separated architecture).

**Context**: Educational RAG system has distinct workload patterns: read-heavy vector search vs write-heavy user tracking.

**Alternatives Considered**:
1. **pgVector in Neon** (unified): ✅ Single database, ACID transactions | ❌ Lower throughput at scale
2. **Qdrant + Neon** (separated): ✅ 10x throughput, independent scaling | ❌ Two systems to manage
3. **Pinecone + Neon**: ❌ Proprietary, expensive, less educational value

**Trade-offs**:
- ✅ **Pros**: Specialized performance (Qdrant optimized for HNSW), cost-effective (free tiers), educational transparency
- ❌ **Cons**: No cross-database transactions, slightly more complex deployment

**Rationale**: Vector search (RAG queries) and user data (auth, history) have different performance profiles. Separation allows independent optimization and scaling. Free tiers on both platforms minimize costs.

**Outcome**: Qdrant Cloud for embeddings, Neon Serverless Postgres for user tables. No synchronization required (eventual consistency acceptable).

---

### ADR-003: 512-Token Chunks with 20% Overlap

**Decision**: Use 512-token chunks with 102-token overlap (20%).

**Context**: RAG retrieval quality depends on optimal chunk size. Educational content includes mixed formats (text, code, tables).

**Alternatives Considered**:
1. **Small chunks (256-400 tokens)**: ✅ Good for factoid queries | ❌ Insufficient context for analytical questions
2. **Medium chunks (512 tokens)**: ✅ Balanced detail + performance | ✅ Recommended
3. **Large chunks (800 tokens)**: ✅ Rich context | ❌ Higher embedding costs, slower retrieval

**Trade-offs**:
- ✅ **Pros**: Optimal for beginner questions ("What is X?") and analytical queries ("Compare X vs Y")
- ✅ **Pros**: 20% overlap prevents context loss at chunk boundaries
- ❌ **Cons**: Slightly higher storage costs vs 256-token chunks (acceptable with Qdrant free tier)

**Rationale**: Research shows 512 tokens balances retrieval accuracy (preserves semantic meaning) with performance (fits in OpenAI context window). 20% overlap is industry standard for educational content.

**Outcome**: RecursiveCharacterTextSplitter with 512 tokens, 102-token overlap, document-aware separators (preserve code blocks, headings).

---

### ADR-004: Mermaid (Built-in) vs PlantUML for Diagrams

**Decision**: Use Mermaid for 80% of diagrams, static images for 20% (complex architecture).

**Context**: ROS 2 concepts require clear visual explanations (node communication, data flow).

**Alternatives Considered**:
1. **Mermaid**: ✅ Built-in Docusaurus support, Markdown-native, version control friendly
2. **PlantUML**: ✅ Rich UML support | ❌ Requires external plugin, dependency on Kroki service
3. **Static images (PNG/SVG)**: ✅ Maximum control | ❌ Not version-control friendly, manual updates

**Trade-offs**:
- ✅ **Pros**: Zero configuration (Mermaid built into Docusaurus 3+), dark mode support, easy contributor workflow
- ❌ **Cons**: Limited UML capabilities vs PlantUML (acceptable for educational diagrams)

**Rationale**: Mermaid covers 80% of use cases (flowcharts, sequence diagrams, system architecture). Complex diagrams (e.g., full robot kinematic tree) better served by static images created in specialized tools.

**Outcome**: Mermaid for all dynamic diagrams (node communication, data flow), static images for photos and complex architecture.

---

### ADR-005: GitHub Pages (Static) vs Netlify/Vercel

**Decision**: Use GitHub Pages for frontend deployment.

**Context**: Budget constraints and simplicity requirements for educational project.

**Alternatives Considered**:
1. **GitHub Pages**: ✅ Free, CDN, tight Git integration | ❌ Static only (acceptable)
2. **Netlify**: ✅ Advanced features (redirects, forms) | ❌ Costs at scale, overkill for docs
3. **Vercel**: ✅ Edge functions, previews | ❌ Costs at scale, complexity

**Trade-offs**:
- ✅ **Pros**: $0 cost, automatic deployments on push, GitHub Actions integration
- ❌ **Cons**: Static only (not an issue for Docusaurus), no server-side rendering (not needed)

**Rationale**: GitHub Pages provides everything needed for static Docusaurus deployment at zero cost. Advanced features of Netlify/Vercel not required for educational content.

**Outcome**: GitHub Actions workflow with `deploy-pages@v4`, custom domain support.

---

## Testing Strategy

### Frontend Testing

**Unit Tests** (Jest + React Testing Library):
```javascript
// Test TextSelectionPlugin
describe('TextSelectionPlugin', () => {
  it('captures selected text > 20 characters', () => {
    // Simulate text selection
    // Assert tooltip appears
  });

  it('sends selected text to RAG API', async () => {
    // Mock fetch to /api/query
    // Assert selected_text field populated
  });
});
```

**Build Validation**:
```bash
# Pre-deployment checks
npm run build          # Must complete without errors
npm run serve          # Manual smoke test
npx broken-link-checker http://localhost:3000  # Zero broken links
```

**Accessibility**:
```bash
# WCAG 2.1 AA compliance
npx axe-cli http://localhost:3000 --tags wcag21aa
```

### Backend Testing

**Unit Tests** (pytest):
```python
# Test retrieval accuracy
@pytest.mark.asyncio
async def test_retrieval_accuracy():
    query = "How do I create a ROS 2 node?"
    results = await retrieve_chunks(query)

    assert len(results) >= 3
    assert results[0].payload['chapter'] == 'chapter1-ros2-core'
    assert results[0].score > 0.8  # High relevance
```

**Integration Tests**:
```python
# Test full RAG pipeline
@pytest.mark.asyncio
async def test_rag_query_endpoint():
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post("/api/query", json={
            "query": "What is a ROS 2 node?"
        })

    assert response.status_code == 200
    data = response.json()
    assert "answer" in data
    assert len(data["sources"]) > 0
    assert "rclpy" in data["answer"].lower()
```

**Performance Tests**:
```python
# Load testing with Locust
from locust import HttpUser, task

class RAGUser(HttpUser):
    @task
    def query_chatbot(self):
        self.client.post("/api/query", json={
            "query": "What is a ROS 2 topic?"
        })

# Target: 100 concurrent users, <3s p95 response time
```

### Code Example Validation

**CI/CD Pipeline** (.github/workflows/test-examples.yml):
```yaml
name: Test ROS 2 Examples

on: [push, pull_request]

jobs:
  test-examples:
    runs-on: ubuntu-22.04
    container: ros:humble

    steps:
      - uses: actions/checkout@v4

      - name: Install dependencies
        run: |
          apt-get update
          apt-get install -y python3-pip
          pip3 install pytest

      - name: Test Chapter 1 examples
        run: |
          cd examples/module1/tests
          pytest test_chapter1.py -v

      - name: Test Chapter 2 examples
        run: pytest examples/module1/tests/test_chapter2.py -v

      - name: Test Chapter 3 URDF
        run: |
          check_urdf examples/module1/chapter3/simple_humanoid.urdf
```

**Example Test**:
```python
# examples/module1/tests/test_chapter1.py
import subprocess

def test_publisher_runs():
    """Verify publisher.py executes and prints messages"""
    proc = subprocess.Popen(
        ['python3', 'examples/module1/chapter1/publisher.py'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    # Let it run for 3 seconds
    time.sleep(3)
    proc.terminate()
    stdout, _ = proc.communicate()

    assert b"Publishing:" in stdout
    assert proc.returncode in [0, -15]  # Normal or SIGTERM
```

### RAG Accuracy Metrics

**Precision/Recall Benchmark**:
```python
# Create test dataset: 50 questions with ground-truth answers
test_cases = [
    {
        "query": "How do I create a ROS 2 node?",
        "expected_chapter": "chapter1-ros2-core",
        "expected_keywords": ["rclpy", "Node", "inherit"]
    },
    # ... 49 more
]

# Measure retrieval accuracy
def evaluate_rag_system():
    precision_scores = []
    recall_scores = []

    for test in test_cases:
        results = retrieve_chunks(test["query"])

        # Precision: Are retrieved chunks relevant?
        relevant = sum(1 for r in results if test["expected_chapter"] in r.payload['chapter'])
        precision = relevant / len(results)

        # Recall: Did we get all relevant chunks?
        all_relevant = get_all_chunks_for_chapter(test["expected_chapter"])
        recall = relevant / len(all_relevant)

        precision_scores.append(precision)
        recall_scores.append(recall)

    print(f"Avg Precision: {np.mean(precision_scores):.2%}")
    print(f"Avg Recall: {np.mean(recall_scores):.2%}")

# Target: 85%+ precision, 70%+ recall
```

**Citation Accuracy**:
```python
# Verify citations match retrieved chunks
def test_citation_accuracy():
    response = query_rag("What is a ROS 2 node?")

    # Extract cited chapters from answer
    cited_chapters = extract_citations(response['answer'])

    # Compare with actual sources
    source_chapters = [s['chapter'] for s in response['sources']]

    # All citations must match sources
    assert set(cited_chapters).issubset(set(source_chapters))

# Target: 100% citation accuracy (no hallucinated sources)
```

---

## Deployment Pipeline

### Frontend (GitHub Pages)

**Workflow**: `.github/workflows/deploy-docs.yml`
```yaml
name: Deploy Docusaurus to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: '20'
          cache: 'npm'
          cache-dependency-path: my-website/package-lock.json

      - name: Install dependencies
        working-directory: ./my-website
        run: npm ci

      - name: Build Docusaurus
        working-directory: ./my-website
        run: npm run build

      - name: Upload artifact
        uses: actions/upload-pages-artifact@v3
        with:
          path: my-website/build

  deploy:
    needs: build
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

**Triggers**: Every push to `main` branch

### Backend (Railway/Fly.io)

**Dockerfile**:
```dockerfile
FROM python:3.11-slim

WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application
COPY . .

# Expose port
EXPOSE 8000

# Run FastAPI
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

**Deployment**:
```bash
# Railway CLI
railway up

# Or Fly.io CLI
fly deploy
```

**Environment Variables** (set in Railway/Fly.io dashboard):
```bash
OPENAI_API_KEY=sk-...
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=...
NEON_DATABASE_URL=postgres://...
FRONTEND_URL=https://[username].github.io
```

### Indexing Pipeline

**Script**: `scripts/index_content.py`
```python
# Run manually after content updates
python scripts/index_content.py \
  --source my-website/docs \
  --collection ros2_book_content \
  --chunk-size 512 \
  --overlap 0.2
```

**Output**:
```
✓ Chunked chapter1-ros2-core.md → 15 chunks
✓ Chunked chapter2-agent-bridge.md → 18 chunks
✓ Chunked chapter3-urdf-model.md → 12 chunks
✓ Generated embeddings for 45 chunks
✓ Indexed to Qdrant collection 'ros2_book_content'
✓ Total time: 23.4 seconds
```

---

## Success Metrics

### Educational Outcomes (from Spec)

- **SC-001**: Students create ROS 2 node within 15 minutes ✅ Validated via user testing
- **SC-002**: Students verify topic communication with CLI tools ✅ Code example + instructions
- **SC-003**: Students implement service call/response ✅ Complete example provided
- **SC-004**: Students integrate Python agent with rclpy ✅ Chapter 2 focus
- **SC-005**: Students create valid URDF ✅ Template + validation tool
- **SC-006**: Students visualize URDF in RViz ✅ Step-by-step guide
- **SC-007**: All examples run on ROS 2 Humble ✅ CI/CD validation
- **SC-008**: 80% completion without external help ✅ User study (post-launch)
- **SC-009**: Students design multi-node systems ✅ Conceptual understanding quiz
- **SC-010**: Students explain rclpy purpose ✅ Assessment questions

### Technical Metrics

**Performance**:
- Page Load: <2s (Lighthouse score >90)
- RAG Query: <3s p95
- Build Time: <60s

**Quality**:
- Zero broken links
- 100% code examples pass CI/CD
- 85%+ RAG citation accuracy
- WCAG 2.1 AA compliance

**Reliability**:
- 99.9% uptime (GitHub Pages + backend)
- <1% error rate on RAG queries

---

## Next Steps

1. **Approve this plan** → Proceed to `/sp.tasks` for task breakdown
2. **Clarify any ambiguities** → Run `/sp.clarify` if needed
3. **Document ADRs** → Run `/sp.adr` for architecturally significant decisions
4. **Begin implementation** → Start with Phase A (Docusaurus setup)

**Recommended workflow**:
```bash
# 1. Review and approve plan
# (Manual review by stakeholder)

# 2. Generate tasks
/sp.tasks

# 3. Document ADRs
/sp.adr "Docusaurus docs-only mode vs full site"
/sp.adr "Qdrant + Neon separated architecture"

# 4. Begin implementation
git checkout -b feature/docusaurus-setup
# ... implement tasks ...
```

---

**Plan Status**: ✅ READY FOR REVIEW

**Estimated Timeline** (after task approval):
- Phase A (Docusaurus Setup): 2 days
- Phase B (Content Creation): 5-7 days
- Phase C (Code Examples): 3-4 days
- Phase D (Backend RAG): 4-5 days
- Phase E (Frontend Integration): 2-3 days
- Phase F (Validation): 2 days

**Total**: ~18-23 days (3-4 weeks for Module 1 MVP)
