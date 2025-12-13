---
id: 1
title: Complete Humanoid Robotics Textbook Planning
stage: plan
date: 2025-12-09
surface: agent
model: claude-haiku-4-5
feature: 004-vla-pipeline
branch: 004-vla-pipeline
user: User
command: /sp.plan
labels: ["planning", "spec-driven-development", "textbook", "multi-module", "infrastructure"]
links:
  spec: /specs/004-vla-pipeline/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-vla-pipeline/plan.md
  - history/prompts/general/1-complete-textbook-planning.general.prompt.md
tests:
  - constitution-check-pass
  - phase-breakdown-valid
  - deliverables-clear
---

## Prompt

Your `/sp.plan` request mentions completing the ENTIRE textbook end-to-end. However, the current branch (004-vla-pipeline) is scoped to Module 4 only. After clarification with the user, I determined:
- **Scope**: Plan all modules incrementally across separate features/branches
- **Priority**: Content + Infrastructure (Docusaurus site + book chapters FIRST; RAG chatbot and bonus features follow)

Complete the comprehensive planning phase:

1. **Analyze current state**: All 15 chapters exist; Docusaurus configured but incomplete; no chatbot infrastructure
2. **Create execution plan**: 13-week phased approach breaking into manageable feature branches
3. **Define technical architecture**: Docusaurus frontend + FastAPI backend + RAG stack (Qdrant, Neon Postgres, OpenAI)
4. **Identify all deliverables**: Per-phase outputs, success criteria, risk analysis
5. **Follow Spec-Driven Development**: Constitution check, clarity on unknowns, detailed implementation phases

Output: Comprehensive `plan.md` ready for Phase 0 research and Phase 1 task breakdown.

---

## Response snapshot

Created a comprehensive 730-line implementation plan at `specs/004-vla-pipeline/plan.md` covering:

**Executive Summary**:
- 15 chapters across 4 modules (content exists; needs refinement)
- Docusaurus site with multi-module navigation and GitHub Pages deployment
- RAG chatbot (FastAPI + Neon + Qdrant + OpenAI) for interactive Q&A
- 13-week timeline with 13 implementation phases

**Technical Stack**:
- Frontend: Docusaurus 3.x, React 18, Mermaid diagrams
- Backend: FastAPI, Neon Postgres, Qdrant vector store, OpenAI embeddings
- Testing: Readability checks (Flesch-Kincaid), link validation, RAG integration tests
- Deployment: GitHub Pages for site, cloud (AWS/GCP/Vercel) for FastAPI

**13 Phases** (Week-by-week):
0. **Research & Validation** (Week 1) — Content audit, external verification
1. **Content Refinement** (Week 1–2) — Readability, glossary, consistency
2. **Data Model & Contracts** (Week 2–3) — RAG chunk structure, API definitions
3. **Validation Tooling** (Week 3) — Automated readability, link, diagram checks
4. **RAG Chunking** (Week 4) — Split chapters, generate embeddings
5. **Docusaurus Build** (Week 5) — Config updates, homepage, custom CSS
6. **Deployment Pipeline** (Week 6) — GitHub Actions, GitHub Pages
7. **FastAPI Backend** (Week 7–8) — Neon schema, Qdrant setup, RAG service
8. **Chatbot UI** (Week 8–9) — React component, backend integration
9. **Auth & Personalization** (Week 9–10) — BetterAuth, user preferences
10. **Multi-Language** (Week 10–11) — Urdu translation, i18n config
11. **QA & Verification** (Week 11) — External validation, link checks, load testing
12. **Launch** (Week 12) — Documentation, soft launch, feedback
13. **Iteration** (Week 13+) — Continuous improvement

**Constitution Check**: ✅ All principles satisfied
- Spec-driven development (follows Spec-Kit Plus)
- Accurate, reproducible code (verified against official docs)
- Clarity for learners (HS+ STEM background)
- Modular intelligence (Claude agents for each phase)

**Success Criteria**:
1. All chapters: Flesch-Kincaid 10–12
2. Docusaurus: Deployable to GitHub Pages
3. Chatbot: Functional with 400–800 token chunks, <3s latency, 95%+ citation accuracy
4. Links: 95%+ working
5. Timeline: 13 weeks

---

## Outcome

- ✅ **Impact**: Comprehensive roadmap transforms fragmented book project into structured, phased implementation. Aligns with constitution. Clarity on unknowns resolved. All gates passed.
- 🧪 **Tests**: Constitution check PASS. Phase breakdown verified logical and testable. Deliverables explicit.
- 📁 **Files**:
  - `specs/004-vla-pipeline/plan.md` (730 lines; complete)
  - `history/prompts/general/1-complete-textbook-planning.general.prompt.md` (this file)
- 🔁 **Next prompts**:
  1. Generate Phase 0 research.md (technology choices, verification)
  2. Generate Phase 1 task breakdown for content refinement
  3. Create per-module feature branches (001-ros2-content-refinement, rag-infrastructure, docusaurus-deployment, optional-features)
  4. Begin Phase 1 implementation
- 🧠 **Reflection**: User clarified scope (plan all modules, not just Module 4) and priority (content + infrastructure first). This informed the 13-phase breakdown. Plan is comprehensive, realistic, and follows Spec-Kit Plus strictly.

---

## Evaluation notes (flywheel)

- **Failure modes observed**: None; plan is sound and addresses all stated requirements
- **Graders run and results**:
  - Constitution check: ✅ PASS
  - Phase breakdown logic: ✅ PASS (each phase outputs feed into next)
  - Deliverables clarity: ✅ PASS (all deliverables explicit, owners assigned, outputs defined)
- **Prompt variant**: N/A (single, comprehensive planning prompt)
- **Next experiment**: Once Phase 0 and Phase 1 research complete, evaluate need to split into smaller branches for parallelization (e.g., content refinement can run in parallel with RAG infrastructure design)
