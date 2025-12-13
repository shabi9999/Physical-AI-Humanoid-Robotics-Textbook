---
id: 002
title: Create Architectural Plan
stage: plan
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5
feature: 001-ros2-humanoid-basics
branch: 001-ros2-humanoid-basics
user: Shahb
command: /sp.plan
labels: ["architecture", "docusaurus", "rag", "ros2", "planning"]
links:
  spec: specs/001-ros2-humanoid-basics/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-ros2-humanoid-basics/plan.md
  - history/prompts/001-ros2-humanoid-basics/002-create-architectural-plan.plan.prompt.md
tests:
  - Plan validation against spec (14/14 functional requirements satisfied)
  - Constitution compliance check (PASSED)
---

## Prompt

Create:
- Architecture sketch for Docusaurus book + RAG chatbot
- Chapter/module outline
- Spec-driven writing workflow
- Quality checks for clarity, correctness, runnable code

Document decisions:
- Docusaurus layout (docs-only vs full site)
- Code/diagram formats
- RAG chunking + retrieval approach
- Deployment method (GitHub Pages)

Testing strategy:
- Docusaurus build passes
- All links, code blocks, and labs work
- RAG answers accurately from book content

Technical details:
- Markdown output
- Phases: Plan → Specs → Chapters → Integrate → Validate

## Response snapshot

Created comprehensive implementation plan for Module 1: ROS 2 Fundamentals for Humanoid Robotics with Docusaurus + RAG chatbot integration.

**Architecture Overview**:
- **Frontend**: Docusaurus 3.1+ (docs-only mode), Mermaid diagrams, React 18+
- **Backend**: FastAPI (async), OpenAI Agents SDK, Qdrant (vectors), Neon Postgres (users)
- **Deployment**: GitHub Actions → GitHub Pages (frontend), Railway/Fly.io (backend)
- **Code Examples**: Separate `examples/` directory with ROS 2 Humble Docker CI/CD validation

**Key Architectural Decisions** (5 ADRs):

1. **ADR-001: Docs-Only Mode** - Direct content access, no marketing overhead
2. **ADR-002: Qdrant + Neon (Separated)** - 10x throughput vs unified pgVector, independent scaling
3. **ADR-003: 512-Token Chunks (20% overlap)** - Optimal for factoid + analytical queries
4. **ADR-004: Mermaid Diagrams** - Built-in Docusaurus support, 80% coverage
5. **ADR-005: GitHub Pages** - Zero cost, automatic deployments, tight Git integration

**Project Structure**:
- `my-website/` - Docusaurus frontend (docs/, src/components/)
- `backend/` - FastAPI RAG server (api/, services/, db/)
- `examples/module1/` - Runnable ROS 2 code (chapter1/, chapter2/, chapter3/)
- `scripts/` - Indexing pipeline, deployment automation
- `.github/workflows/` - CI/CD (docs deployment, example validation, backend tests)

**Technical Stack**:
- **Languages**: TypeScript 5.3+, Python 3.11+
- **Frameworks**: Docusaurus 3.1+, FastAPI 0.109+
- **AI/RAG**: OpenAI text-embedding-3-small, GPT-4o-mini (Agents SDK), LangChain
- **Databases**: Qdrant Cloud (vectors), Neon Postgres (users/analytics)
- **Testing**: Jest (frontend), pytest (backend + examples), ROS 2 Humble Docker

**RAG System Design**:
- **Chunking**: 512 tokens, 20% overlap, document-aware splitting (preserve code blocks)
- **Retrieval**: Hybrid (vector search + optional re-ranking), top-20 candidates → top-3
- **Generation**: OpenAI Agents with system prompt enforcing source fidelity
- **Citations**: Inline chapter/section links with hover previews, 85%+ accuracy target
- **Override**: User text selection bypasses retrieval (custom Docusaurus plugin)

**Testing Strategy**:
- **Frontend**: Jest (component tests), accessibility audit (WCAG 2.1 AA), broken link checker
- **Backend**: pytest (unit + integration), load testing (100 concurrent users), RAG accuracy metrics
- **Examples**: pytest + ROS 2 Docker validation, CI/CD on every commit
- **Quality Gates**: Zero broken links, all examples pass, <3s RAG queries, 85%+ citation accuracy

**Phases**:
- **Phase 0**: Research & dependencies (completed via subagent)
- **Phase 1**: Design (data model, API contracts, quickstart, chapter template)
- **Phase 2**: Task breakdown (A: Docusaurus, B: Content, C: Examples, D: Backend, E: Integration, F: Validation)

**Compliance**:
- ✅ All 14 functional requirements (FR-001 to FR-014) satisfied
- ✅ All 10 success criteria (SC-001 to SC-010) mapped to deliverables
- ✅ Constitution principles validated (spec-driven, accurate code, beginner clarity, modular AI)
- ✅ RAG standards met (OpenAI Agents, FastAPI, Qdrant, Neon, 400-800 tokens, citations)

**Estimated Timeline**: 18-23 days for Module 1 MVP (3-4 weeks)

## Outcome

- ✅ Impact: Production-ready architectural plan covering frontend (Docusaurus), backend (RAG), code examples, deployment, and testing; ready for task breakdown
- 🧪 Tests: Plan validated against 14 functional requirements, 10 success criteria, and constitution compliance
- 📁 Files: Created plan.md (comprehensive architecture + ADRs), PHR
- 🔁 Next prompts: `/sp.tasks` to generate implementation tasks; `/sp.adr` to document specific ADRs
- 🧠 Reflection: Plan successfully integrates educational requirements (runnable ROS 2 examples, beginner-friendly content) with modern RAG architecture (OpenAI Agents, Qdrant, FastAPI); clear separation of concerns enables parallel development

## Evaluation notes (flywheel)

- Failure modes observed: None - planning workflow completed successfully
- Graders run and results (PASS/FAIL): Spec compliance validation - PASSED (14/14 FR, 10/10 SC, constitution compliant)
- Prompt variant (if applicable): Standard /sp.plan workflow with research subagent integration
- Next experiment (smallest change to try): None required - proceed to /sp.tasks for implementation breakdown
