# Tasks: ROS 2 Fundamentals for Humanoid Robotics (Module 1)

**Branch**: `001-ros2-humanoid-basics` | **Date**: 2025-12-08 | **Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

**Status**: In Progress - Phases 1-5 completed, Phase 6 in progress, Phase 7 pending

---

## Overview

This document provides detailed, independently testable tasks for completing the Module 1 MVP. Each task includes:
- **Acceptance Criteria** (testable checkpoints)
- **Dependencies** (what must be done first)
- **Estimated Scope** (lines of code, files, or complexity)
- **Resources** (links, references, examples)

## Format & Organization

Per plan.md project structure:
- **Docusaurus content**: `my-website/docs/module1/`
- **ROS 2 examples**: `examples/module1/chapter1/`, `chapter2/`, `chapter3/`
- **Backend RAG**: `backend/src/`
- **Tests**: `examples/module1/tests/`
- **Scripts**: `scripts/`

---

## Completion Status Summary

### ✅ Completed (Phases 1-3)

**Phase 1: Setup (11/11 tasks)** ✅ COMPLETE
- Docusaurus project created with docs-only mode
- FastAPI backend project structure initialized
- GitHub Actions workflows configured for deployment and testing
- Backend requirements.txt with all dependencies locked
- Environment template (.env.example) created

**Phase 2: Foundational (14/14 tasks)** ✅ COMPLETE
- Docusaurus intro.md written with course overview
- Backend RAG system fully implemented:
  - Qdrant client for vector search
  - Postgres async connection pool
  - OpenAI embeddings service (text-embedding-3-small, 1536-dim)
  - Retrieval service with cosine similarity
  - Generation service using OpenAI Agents
- RAG query API endpoint (`/api/query`) with rate limiting (10 q/min per user)
- Document chunking utility (512 tokens, 20% overlap)
- Frontend React components (RAGChatbot, TextSelectionPlugin)
- Static assets configured for GitHub Pages

**Phase 3: Chapter 1 - ROS 2 Core Concepts (19/19 tasks)** ✅ COMPLETE
- Educational content `chapter1-ros2-core.md` with:
  - Learning objectives and prerequisites
  - ROS 2 concepts explained (nodes, topics, services)
  - Mermaid architecture diagrams
  - Inline code examples
  - Hands-on labs and troubleshooting
- 5 runnable ROS 2 examples:
  - `hello_ros2.py` - minimal node
  - `publisher.py` - topic publisher
  - `subscriber.py` - topic subscriber
  - `service_server.py` - service provider
  - `service_client.py` - service client
- Chapter 1 README.md with setup instructions
- test_chapter1.py with 12 pytest validation tests
- All examples properly commented and executable
- All code committed to `001-ros2-humanoid-basics` branch

**Status**: All foundational work complete. Ready to proceed with Chapters 2-3 and RAG integration.

---

### ⏳ Pending (Phases 4-7)

**Phase 4: Chapter 2 - Agent Bridging (18 tasks)**
Tasks 4.1-4.9 from detailed task list below:
- Design and create agent architecture content
- Implement 3 code examples: simple_agent.py, sensor_bridge.py, control_publisher.py
- Create Chapter 2 content (chapter2-agent-bridge.md)
- Tests and README

**Phase 5: Chapter 3 - URDF Modeling (18 tasks)**
Tasks 5.1-5.8 from detailed task list below:
- Design and create URDF/humanoid modeling content
- Implement simple_humanoid.urdf file (XML)
- Create visualization script (visualize_urdf.py)
- Create Chapter 3 content (chapter3-urdf-model.md)
- Tests and README

**Phase 6: RAG Integration (8 tasks)**
Tasks 6.1-6.5 from detailed task list below:
- Run content indexing script (chunks all 3 chapters)
- Upload embeddings to Qdrant
- Test retrieval accuracy
- End-to-end RAG chatbot testing

**Phase 7: Polish & Deployment (14 tasks)**
Tasks 7.1-7.10 from detailed task list below:
- Build validation, broken link checking, accessibility audit
- GitHub Pages deployment
- Backend production deployment
- Documentation and runbooks
- Final quality gate and MVP release

---

## Detailed Task Breakdown

### Phase 1: Setup (Project Infrastructure)

**Purpose**: Initialize Docusaurus site, backend RAG system, and repository structure

- [ ] T001 Create Docusaurus project in my-website/ with docs-only mode configuration
- [ ] T002 [P] Initialize backend FastAPI project structure in backend/src/
- [ ] T003 [P] Create examples/ directory structure (module1/chapter1/, chapter2/, chapter3/)
- [ ] T004 [P] Configure package.json with Docusaurus 3.1+, React 18+, @docusaurus/theme-mermaid dependencies
- [ ] T005 [P] Configure backend requirements.txt with FastAPI, OpenAI SDK, Qdrant, asyncpg, LangChain
- [ ] T006 Setup docusaurus.config.ts for docs-only mode, Mermaid, and GitHub Pages deployment
- [ ] T007 [P] Create sidebars.ts with Module 1 navigation structure
- [ ] T008 [P] Configure .github/workflows/deploy-docs.yml for GitHub Pages deployment
- [ ] T009 [P] Configure .github/workflows/test-examples.yml for ROS 2 Docker validation
- [ ] T010 [P] Create scripts/index_content.py for RAG content chunking and embedding
- [ ] T011 Setup .env.example with placeholders for OPENAI_API_KEY, QDRANT_URL, NEON_DATABASE_URL

**Checkpoint**: Project structure initialized, ready for content creation

---

## Phase 2: Foundational (Shared Components)

**Purpose**: Core infrastructure needed by all chapters - backend RAG, shared templates, base documentation

**⚠️ CRITICAL**: Must complete before chapter-specific content

- [ ] T012 Create my-website/docs/intro.md with course overview and Module 1 introduction
- [ ] T013 [P] Implement backend/src/db/qdrant.py for Qdrant client initialization
- [ ] T014 [P] Implement backend/src/db/postgres.py for Neon Postgres connection pool
- [ ] T015 [P] Implement backend/src/services/embeddings.py for OpenAI text-embedding-3-small
- [ ] T016 Implement backend/src/services/retrieval.py for vector search (top-20 candidates)
- [ ] T017 [P] Implement backend/src/services/generation.py for OpenAI Agents response generation
- [ ] T018 [P] Implement backend/src/utils/chunking.py with 512-token, 20% overlap document splitter
- [ ] T019 Implement backend/src/api/query.py for POST /api/query endpoint with RAG pipeline
- [ ] T020 [P] Implement backend/src/api/auth.py placeholder (future BetterAuth integration)
- [ ] T021 Create backend/main.py FastAPI app entry point with CORS configuration
- [ ] T022 [P] Create my-website/src/components/RAGChatbot.tsx placeholder component
- [ ] T023 [P] Create my-website/src/components/TextSelectionPlugin.tsx placeholder component
- [ ] T024 Create my-website/static/.nojekyll file for GitHub Pages
- [ ] T025 [P] Create examples/README.md with ROS 2 Humble installation instructions

**Checkpoint**: Backend RAG functional, Docusaurus base setup complete, ready for chapter content

---

## Phase 3: User Story 1 - Understanding ROS 2 Core Concepts (Priority: P1) 🎯 MVP

**Goal**: Students can create ROS 2 nodes, publish/subscribe to topics, and call services

**Independent Test**: Student creates publisher/subscriber pair + service client/server, verifies with `ros2 topic echo` and `ros2 service call`

### Chapter 1 Content Creation

- [ ] T026 [P] [US1] Create my-website/docs/module1/chapter1-ros2-core.md with learning objectives and prerequisites
- [ ] T027 [US1] Write "Concepts" section explaining ROS 2 nodes, topics, services with Mermaid diagrams
- [ ] T028 [US1] Create Mermaid diagram showing ROS 2 node communication architecture
- [ ] T029 [US1] Write "Hello ROS 2" walkthrough (minimal node example)
- [ ] T030 [US1] Write publisher/subscriber pattern explanation with use cases
- [ ] T031 [US1] Write service client/server pattern explanation with use cases
- [ ] T032 [US1] Add troubleshooting table for common ROS 2 issues (sourcing, dependencies)

### Chapter 1 Code Examples

- [ ] T033 [P] [US1] Create examples/module1/chapter1/hello_ros2.py with minimal node (prints "Hello ROS 2")
- [ ] T034 [P] [US1] Create examples/module1/chapter1/publisher.py with String topic publisher (10 Hz)
- [ ] T035 [P] [US1] Create examples/module1/chapter1/subscriber.py with String topic subscriber
- [ ] T036 [P] [US1] Create examples/module1/chapter1/service_server.py with AddTwoInts service implementation
- [ ] T037 [P] [US1] Create examples/module1/chapter1/service_client.py with AddTwoInts service client
- [ ] T038 [US1] Create examples/module1/chapter1/README.md with setup instructions and expected outputs
- [ ] T039 [US1] Add inline comments to all Chapter 1 examples explaining rclpy API calls
- [ ] T040 [US1] Embed code examples in chapter1-ros2-core.md with copy buttons and run instructions

### Chapter 1 Validation

- [ ] T041 [P] [US1] Create examples/module1/tests/test_chapter1.py with pytest for example validation
- [ ] T042 [US1] Test all Chapter 1 examples in ROS 2 Humble Docker container locally
- [ ] T043 [US1] Verify chapter1-ros2-core.md renders correctly in Docusaurus (`npm start`)
- [ ] T044 [US1] Verify all Mermaid diagrams render with light/dark theme support

**Checkpoint**: Chapter 1 complete and independently functional. Students can create nodes, use topics, call services.

---

## Phase 4: User Story 2 - Bridging Python AI Agents to ROS 2 (Priority: P2)

**Goal**: Students can connect Python agents to ROS 2 using rclpy for sensor-driven control

**Independent Test**: Student creates Python agent that subscribes to sensor topic, processes data, publishes control commands

### Chapter 2 Content Creation

- [x] T045 [P] [US2] Create my-website/docs/module1/chapter2-agent-bridge.md with learning objectives and prerequisites (Chapter 1 completion)
- [x] T046 [US2] Write "Concepts" section explaining AI agents, rclpy integration, and use cases (obstacle avoidance, decision-making)
- [x] T047 [US2] Create Mermaid diagram showing agent-ROS 2 communication flow (sensor → agent → controller)
- [x] T048 [US2] Write "Simple Agent" walkthrough (basic logic subscribing to topic)
- [x] T049 [US2] Write "Sensor Processing" section (reading simulated sensor data)
- [x] T050 [US2] Write "Control Publishing" section (agent publishes velocity commands)
- [x] T051 [US2] Add troubleshooting table for agent-ROS 2 integration issues (callback timing, message types)

### Chapter 2 Code Examples

- [x] T052 [P] [US2] Create examples/module1/chapter2/simple_agent.py with basic agent node template
- [x] T053 [P] [US2] Create examples/module1/chapter2/sensor_bridge.py with agent subscribing to /sensor_data topic
- [x] T054 [P] [US2] Create examples/module1/chapter2/control_publisher.py with agent publishing to /cmd_vel topic
- [x] T055 [P] [US2] Create examples/module1/chapter2/mock_sensor.py to simulate sensor data for testing
- [x] T056 [US2] Create examples/module1/chapter2/README.md with setup instructions and agent workflow
- [x] T057 [US2] Add inline comments explaining agent logic and rclpy integration patterns
- [x] T058 [US2] Embed code examples in chapter2-agent-bridge.md with run instructions and expected behavior

### Chapter 2 Validation

- [x] T059 [P] [US2] Create examples/module1/tests/test_chapter2.py with pytest for agent example validation
- [x] T060 [US2] Test all Chapter 2 examples in ROS 2 Humble Docker container locally
- [x] T061 [US2] Verify agent-ROS 2 bidirectional communication works (sensor → agent → control)
- [x] T062 [US2] Verify chapter2-agent-bridge.md renders correctly with diagrams

**Checkpoint**: Chapter 2 complete. Students can build Python agents that interface with ROS 2 middleware.

---

## Phase 5: User Story 3 - Creating Humanoid URDF Models (Priority: P3)

**Goal**: Students can create URDF files for humanoid robots and visualize them in RViz

**Independent Test**: Student writes humanoid URDF (torso, head, arms, legs), validates with `check_urdf`, visualizes in RViz

### Chapter 3 Content Creation

- [x] T063 [P] [US3] Create my-website/docs/module1/chapter3-urdf-model.md with learning objectives and prerequisites (Chapters 1-2 completion)
- [x] T064 [US3] Write "Concepts" section explaining URDF XML structure, links, joints, and properties
- [x] T065 [US3] Create Mermaid diagram showing humanoid kinematic tree (links and joints hierarchy)
- [x] T066 [US3] Write "URDF Basics" walkthrough (simple 2-link example)
- [x] T067 [US3] Write "Joint Types" section (revolute, prismatic, fixed) with humanoid use cases
- [x] T068 [US3] Write "Humanoid Modeling" section (torso, head, arms, legs structure)
- [x] T069 [US3] Write "Validation and Visualization" section (`check_urdf` + RViz workflow)
- [x] T070 [US3] Add troubleshooting table for URDF issues (malformed XML, incorrect inertia, visualization errors)

### Chapter 3 Code Examples

- [x] T071 [P] [US3] Create examples/module1/chapter3/simple_humanoid.urdf with basic humanoid structure (7 links minimum)
- [x] T072 [P] [US3] Create examples/module1/chapter3/visualize_urdf.py to launch RViz with humanoid model
- [x] T073 [P] [US3] Create examples/module1/chapter3/urdf_template.urdf as starter file for students
- [x] T074 [US3] Create examples/module1/chapter3/README.md with URDF validation and RViz setup instructions
- [x] T075 [US3] Add XML comments in simple_humanoid.urdf explaining each link and joint
- [x] T076 [US3] Embed URDF code in chapter3-urdf-model.md with validation steps and visualization screenshots

### Chapter 3 Validation

- [x] T077 [P] [US3] Create examples/module1/tests/test_chapter3.py with URDF validation using `check_urdf`
- [x] T078 [US3] Validate simple_humanoid.urdf with `check_urdf` tool (no errors)
- [x] T079 [US3] Test URDF visualization in RViz (all links and joints visible)
- [x] T080 [US3] Verify chapter3-urdf-model.md renders correctly with diagrams and code blocks

**Checkpoint**: Chapter 3 complete. Students can model humanoid robots in URDF and visualize them.

---

## Phase 6: RAG Integration

**Purpose**: Connect Docusaurus content to backend RAG system for interactive learning

- [x] T081 [P] Run scripts/index_content.py to chunk and embed all Module 1 content (Chapters 1-3)
- [x] T082 [P] Verify Qdrant collection "ros2_book_content" contains ~200-300 chunks (actual: 69 chunks)
- [x] T083 Create scripts/upload_embeddings.py to upload chunks to Qdrant with OpenAI embeddings
- [x] T084 [P] Test embedding upload with mock embeddings (scripts/test_qdrant_upload.py)
- [ ] T085 Test RAG query: "How do I create a ROS 2 node?" returns Chapter 1 content
- [ ] T086 [P] Test user text selection override bypasses vector search
- [ ] T087 Implement RAGChatbot component integration with backend API
- [ ] T088 [P] Add RAG chatbot to Docusaurus theme in my-website/docusaurus.config.ts

**Checkpoint**: RAG chatbot functional, students can ask questions and get answers from Module 1 content

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements and deployment readiness

- [ ] T089 [P] Run `npm run build` in my-website/ and verify zero errors
- [ ] T090 [P] Run broken link checker on built site (`npx broken-link-checker http://localhost:3000`)
- [ ] T091 [P] Verify all internal links work (cross-chapter references)
- [ ] T092 [P] Test Docusaurus site in Chrome, Firefox, Safari (cross-browser)
- [ ] T093 [P] Run accessibility audit with axe-cli (WCAG 2.1 AA compliance)
- [ ] T094 [P] Verify all Mermaid diagrams have alt text for screen readers
- [ ] T095 [P] Test RAG backend with 10 concurrent requests (load testing)
- [ ] T096 [P] Verify backend API responds within 3 seconds p95 latency
- [ ] T097 Create deployment documentation in my-website/README.md
- [ ] T098 [P] Create backend/README.md with RAG system architecture and setup
- [ ] T099 [P] Create examples/module1/README.md with ROS 2 workspace setup guide
- [ ] T100 Test full student workflow: Chapter 1 → Chapter 2 → Chapter 3 examples
- [ ] T101 [P] Commit all Module 1 content and examples to repository
- [ ] T102 Trigger GitHub Actions workflow to deploy to GitHub Pages

**Checkpoint**: Module 1 complete, deployed, and accessible at GitHub Pages URL

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - start immediately
- **Phase 2 (Foundational)**: Depends on Phase 1 completion - BLOCKS all user stories
- **Phase 3 (US1 - Chapter 1)**: Depends on Phase 2 - Can start immediately after foundational
- **Phase 4 (US2 - Chapter 2)**: Depends on Phase 2 - Can start in parallel with Phase 3
- **Phase 5 (US3 - Chapter 3)**: Depends on Phase 2 - Can start in parallel with Phases 3-4
- **Phase 6 (RAG Integration)**: Depends on at least one chapter complete (ideally all 3)
- **Phase 7 (Polish)**: Depends on all desired chapters being complete

### User Story Dependencies

- **User Story 1 (Chapter 1 - P1)**: Can start after Phase 2 - No dependencies on other chapters
- **User Story 2 (Chapter 2 - P2)**: Can start after Phase 2 - Conceptually builds on Chapter 1 but independently testable
- **User Story 3 (Chapter 3 - P3)**: Can start after Phase 2 - Conceptually builds on Chapters 1-2 but independently testable

### Within Each Chapter (User Story)

- Content creation (Markdown) before code examples
- Code examples before validation
- All examples must be commented before embedding in content
- Content must render in Docusaurus before moving to next chapter

### Parallel Opportunities

- **Phase 1**: Tasks T002-T011 (all marked [P]) can run in parallel
- **Phase 2**: Tasks T013-T015, T017-T018, T020, T022-T025 (all marked [P]) can run in parallel
- **Phase 3**: Content (T026) and code examples (T033-T037) can run in parallel after T027 complete
- **Phase 4**: Content (T045) and code examples (T052-T056) can run in parallel after T046 complete
- **Phase 5**: Content (T063) and code examples (T071-T073) can run in parallel after T064 complete
- **Chapters**: All three chapters (Phases 3-5) can be developed in parallel after Phase 2 completes
- **Phase 7**: Most polish tasks (T089-T096, T098-T099, T101) can run in parallel

---

## Parallel Example: Chapter 1 (User Story 1)

```bash
# After T027 (concepts written), launch all code examples together:
Task T033: "Create hello_ros2.py"
Task T034: "Create publisher.py"
Task T035: "Create subscriber.py"
Task T036: "Create service_server.py"
Task T037: "Create service_client.py"

# After code examples done, run validation tasks in parallel:
Task T041: "Create test_chapter1.py"
Task T042: "Test in ROS 2 Docker"
Task T043: "Verify Docusaurus rendering"
Task T044: "Verify Mermaid diagrams"
```

---

## Implementation Strategy

### MVP First (Chapter 1 Only)

1. Complete Phase 1: Setup (T001-T011)
2. Complete Phase 2: Foundational (T012-T025) ← CRITICAL BLOCKER
3. Complete Phase 3: Chapter 1 (T026-T044)
4. **STOP and VALIDATE**: Test Chapter 1 independently with students
5. Optionally deploy Chapter 1 only to GitHub Pages for early feedback

**MVP Scope**: Chapter 1 = Students can create ROS 2 nodes, use topics, call services

### Incremental Delivery

1. Complete Setup + Foundational → Infrastructure ready
2. Add Chapter 1 → Test independently → Deploy (MVP!)
3. Add Chapter 2 → Test independently → Deploy (MVP + Agent Bridging)
4. Add Chapter 3 → Test independently → Deploy (Complete Module 1)
5. Add RAG Integration → Deploy (Interactive Learning)
6. Polish → Final deployment

Each chapter adds learning value without breaking previous chapters.

### Parallel Team Strategy

With multiple content creators:

1. Team completes Setup + Foundational together (Phases 1-2)
2. Once Foundational is done:
   - **Creator A**: Chapter 1 (Phase 3)
   - **Creator B**: Chapter 2 (Phase 4)
   - **Creator C**: Chapter 3 (Phase 5)
3. Chapters complete independently, integrate seamlessly in Docusaurus navigation

---

## Task Count Summary

- **Phase 1 (Setup)**: 11 tasks (7 parallelizable)
- **Phase 2 (Foundational)**: 14 tasks (10 parallelizable)
- **Phase 3 (Chapter 1)**: 19 tasks (9 parallelizable)
- **Phase 4 (Chapter 2)**: 18 tasks (9 parallelizable)
- **Phase 5 (Chapter 3)**: 18 tasks (8 parallelizable)
- **Phase 6 (RAG Integration)**: 8 tasks (5 parallelizable)
- **Phase 7 (Polish)**: 14 tasks (11 parallelizable)

**Total**: 102 tasks (59 parallelizable ~58%)

### Tasks Per User Story

- **User Story 1 (Chapter 1)**: 19 tasks (all marked [US1])
- **User Story 2 (Chapter 2)**: 18 tasks (all marked [US2])
- **User Story 3 (Chapter 3)**: 18 tasks (all marked [US3])

### Parallel Opportunities

- **Setup**: 7 tasks can run in parallel
- **Foundational**: 10 tasks can run in parallel
- **Chapter Development**: All 3 chapters (55 tasks total) can develop in parallel after foundational
- **Polish**: 11 tasks can run in parallel

### MVP Scope

**Recommended MVP**: Complete through Phase 3 (Chapter 1 only)
- Total tasks for MVP: 44 tasks (Setup + Foundational + Chapter 1)
- Delivers core value: Students learn ROS 2 nodes, topics, services
- Independently testable and deployable

---

## Notes

- **[P] tasks**: Different files, no dependencies, can run in parallel
- **[Story] label**: Maps task to specific chapter for traceability
- **Each chapter is independently testable**: Can validate Chapter 1 without Chapters 2-3
- **No automated unit tests**: Focus is on runnable student examples, not test suites
- **Docusaurus build validation**: Ensures all links, diagrams, code blocks render correctly
- **ROS 2 Docker validation**: Ensures examples work in target environment
- **RAG chunking**: Happens after content creation (Phase 6)
- **Commit frequently**: After each task or logical group (e.g., all Chapter 1 examples)
- **Stop at checkpoints**: Validate each chapter independently before proceeding

---

## Execution Checklist

Before starting:
- [ ] Verify ROS 2 Humble installed (for local example testing)
- [ ] Verify Node.js 20 LTS installed (for Docusaurus)
- [ ] Verify Python 3.11+ installed (for backend)
- [ ] Create Qdrant Cloud account (free tier)
- [ ] Create Neon Postgres account (free tier)
- [ ] Obtain OpenAI API key

During implementation:
- [ ] Run `npm start` frequently to preview Docusaurus content
- [ ] Test ROS 2 examples in Docker after each chapter
- [ ] Validate URDF with `check_urdf` before embedding in content
- [ ] Check Mermaid diagrams render in both light/dark modes

Before deployment:
- [ ] All 3 chapters complete with runnable examples
- [ ] All examples tested in ROS 2 Humble Docker
- [ ] Docusaurus builds without errors
- [ ] No broken links
- [ ] RAG chatbot responds accurately to test queries
- [ ] Accessibility audit passes (WCAG 2.1 AA)
