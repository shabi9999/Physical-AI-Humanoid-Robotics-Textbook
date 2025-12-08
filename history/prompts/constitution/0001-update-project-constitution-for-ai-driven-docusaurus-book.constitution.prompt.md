---
id: 0001
title: Update project constitution for AI-Driven Docusaurus Book
stage: constitution
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.constitution
labels: ["project-setup", "constitution"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

/sp.constitution

Project: AI-Driven Docusaurus Book + RAG Chatbot + Physical AI Robotics Capstone

Core Principles:
- Spec-driven development using Spec-Kit Plus
- Accurate, reproducible, runnable code
- Clarity for beginner–intermediate AI/robotics learners
- Modular intelligence via Claude Code Subagents & Skills

Book Standards:
- Docusaurus book with 15+ chapters
- Each chapter: concepts, code, diagrams, labs, troubleshooting
- Uses Spec-Kit Plus templates
- Deployable to GitHub Pages

RAG Chatbot Standards:
- Answers only from book content
- User-selected text override
- Uses OpenAI Agents, FastAPI, Neon Postgres, Qdrant
- Chunk size: 400–800 tokens
- Must cite exact book location

Bonus Features:
- BetterAuth signup + onboarding questionnaire
- Personalized chapter content
- Urdu translation toggle

Robotics Content Requirements:
- ROS 2 (nodes, topics, services, URDF)
- Gazebo physics & sensors
- Unity visualization
- NVIDIA Isaac Sim + VSLAM + Nav2
- VLA: Whisper, LLM planning, action execution

## Response snapshot

The project constitution has been updated to version 1.0.0 based on the provided input. This included defining core principles, book standards, RAG chatbot standards, bonus features, and robotics content requirements. Dependent templates (.specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md) and command files were checked for consistency and found to not require modification as they are placeholders. No runtime guidance documents were found.

## Outcome

- ✅ Impact: Project constitution defined and updated.
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md updated.
- 🔁 Next prompts: Continue with project setup or feature development.
- 🧠 Reflection: The bash script for PHR creation was not found, requiring manual PHR generation.

## Evaluation notes (flywheel)

- Failure modes observed: Bash script for PHR creation not found.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Ensure PHR creation script is available or improve agent-native fallback.