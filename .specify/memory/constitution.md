<!-- Sync Impact Report:
Version change: 1.0.0 → 1.1.0 (MINOR: New principle added for QA)
List of modified principles:
  - Added: V. Quality Assurance & Link Integrity (2025-12-10)
Added sections:
  - Quality Assurance Standards (2025-12-10)
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md ⚠ pending
  - .specify/templates/spec-template.md ⚠ pending
  - .specify/templates/tasks-template.md ⚠ pending
  - .specify/templates/commands/*.md ⚠ pending
Follow-up TODOs: None
Rationale:
  - New Principle V enforces QA standards from daily homepage fixes
  - Zero broken links, correct routing paths are non-negotiable
  - Formal Lighthouse score requirements added (≥90, ≥95, ≥95)
  - MINOR bump justified: new principle adds to existing framework without removing or conflicting
-->
# AI-Driven Docusaurus Book + RAG Chatbot + Physical AI Robotics Capstone Constitution

## Core Principles

### I. Spec-driven development using Spec-Kit Plus
All development adheres to Spec-Kit Plus methodologies, ensuring a structured approach from requirements to implementation.

### II. Accurate, reproducible, runnable code
Code must be precise, easily reproducible in various environments, and executable as intended, with clear instructions and minimal setup.

### III. Clarity for beginner–intermediate AI/robotics learners
All content and code must be accessible and understandable for learners at beginner to intermediate levels in AI and robotics, emphasizing clear explanations and practical examples.

### IV. Modular intelligence via Claude Code Subagents & Skills
The system leverages Claude Code Subagents and Skills for modular intelligence, promoting reusability and specialized task handling.

### V. Quality Assurance & Link Integrity
All navigation links, routing paths, and component integration must be verified and correct. Sidebar links, module routes, and cross-references must point to accurate documentation locations. Zero broken links, zero console errors, and zero broken components are non-negotiable requirements before any deployment.

## Quality Assurance Standards

- Zero broken links across all documentation and navigation
- All routing paths must point to correct documentation locations
- All sidebar links must resolve to valid endpoints
- Build must compile with zero errors and zero warnings
- Console must be clean with no JavaScript errors
- All components must render correctly on mobile, tablet, and desktop
- Lighthouse performance score ≥90, accessibility ≥95, SEO ≥95
- All changes verified before deployment

## Book Standards

- Docusaurus book with 15+ chapters
- Each chapter: concepts, code, diagrams, labs, troubleshooting
- Uses Spec-Kit Plus templates
- Deployable to GitHub Pages

## RAG Chatbot Standards

- Answers only from book content
- User-selected text override
- Uses OpenAI Agents, FastAPI, Neon Postgres, Qdrant
- Chunk size: 400–800 tokens
- Must cite exact book location

## Bonus Features

- BetterAuth signup + onboarding questionnaire
- Personalized chapter content
- Urdu translation toggle

## Robotics Content Requirements

- ROS 2 (nodes, topics, services, URDF)
- Gazebo physics & sensors
- Unity visualization
- NVIDIA Isaac Sim + VSLAM + Nav2
- VLA: Whisper, LLM planning, action execution

## Governance

All project activities, including design, development, and testing, must align with the principles outlined in this constitution. Amendments to this constitution require a documented proposal, review by project stakeholders, and approval by the project lead. Compliance reviews will be conducted periodically to ensure adherence.

**Version**: 1.1.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-10