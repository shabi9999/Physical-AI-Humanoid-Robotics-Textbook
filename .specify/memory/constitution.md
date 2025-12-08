<!-- Sync Impact Report:
Version change: 0.0.0 (initial) → 1.0.0
List of modified principles:
  - Added: Spec-driven development using Spec-Kit Plus
  - Added: Accurate, reproducible, runnable code
  - Added: Clarity for beginner–intermediate AI/robotics learners
  - Added: Modular intelligence via Claude Code Subagents & Skills
Added sections:
  - Book Standards
  - RAG Chatbot Standards
  - Bonus Features
  - Robotics Content Requirements
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md ⚠ pending
  - .specify/templates/spec-template.md ⚠ pending
  - .specify/templates/tasks-template.md ⚠ pending
  - .specify/templates/commands/*.md ⚠ pending
Follow-up TODOs: None
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

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07