---
id: 1
title: Initial Project Constitution
stage: constitution
date: 2025-12-16
surface: agent
model: gemini-1.5-pro
feature: none
branch: main
user: Yousuf Traders
command: /sp.constitution I am starting a hackathon project to create an AI-native textbook for 'Physical AI & Humanoid Robotics'. Please create a comprehensive Constitution document that will govern the entire project. PROJECT OVERVIEW: The goal is to create a textbook using Docusaurus, deployed to GitHub Pages. It includes a RAG chatbot embedded in the book (OpenAI Agents SDK + FastAPI + Qdrant + Neon PostgreSQL). The project phases are: 1. Book content creation with Spec-Kit Plus, 2. FastAPI backend with single /chat endpoint, 3. Chatbot integration on UI (ChatKit), 4. UI enhancement with professional robotics theme, and 5. RAG implementation with Qdrant Cloud and content sync. BONUS FEATURES TO SUPPORT: User authentication (better-auth.com) with background questions, content personalization per user, and Urdu translation at chapter level. TECHNICAL STACK: Frontend includes Docusaurus, React, and ChatKit; Backend includes FastAPI and OpenAI Agents SDK; Database includes Neon PostgreSQL (serverless) and Qdrant Cloud (vector DB); AI Tools include Claude Code, Spec-Kit Plus, and Gemini-CLI; Deployment includes GitHub Pages and Vercel. CONSTITUTION REQUIREMENTS: 1. Project Principles & Goals define success criteria for each phase, establish AI-native development philosophy, and set standards for reusable intelligence accumulation. 2. Technical Standards include code quality rules (type hints, docstrings, linting), testing requirements (coverage, unit/integration tests), security guidelines (API keys, user data), and performance benchmarks. 3. Workflow Rules follow the Spec-Kit Plus SDD workflow (Constitution → Specify → Clarify → Plan → Tasks → Implement), define Git branching strategy and commit conventions, collaboration guidelines between human and AI agents, and phase review and checkpoint process. 4. Phase-Specific Guidelines cover Phase 1 (Book Content): Docusaurus structure, content organization, Spec-Kit Plus usage; Phase 2 (Backend): FastAPI structure, endpoint design, error handling; Phase 3 (Frontend Integration): ChatKit setup, API communication, state management; Phase 4 (UI Enhancement): Theme consistency, responsive design, accessibility; Phase 5 (RAG System): Vector DB schema, embedding strategy, sync process. 5. Quality Gates & Validation ensure each phase must pass specific tests before moving to next phase, include code review criteria, performance and security audits, and documentation completeness checks. 6. Bonus Features Framework defines authentication flow design, personalization architecture, and translation implementation strategy. OUTPUT FORMAT: Generate a complete Constitution document in markdown format, structured with clear sections and actionable guidelines that AI agents can follow. The Constitution should be placed in .specify/memory/constitution.md in the project. The Constitution must be clear and testable (no vague statements), enforceable by AI agents during development, adaptive to phase-specific requirements, and supportive of reusable intelligence patterns.
labels: [constitution, setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
tests:
null
---

## Prompt

/sp.constitution I am starting a hackathon project to create an AI-native textbook for "Physical AI & Humanoid Robotics". Please create a comprehensive Constitution document that will govern the entire project. PROJECT OVERVIEW: The goal is to create a textbook using Docusaurus, deployed to GitHub Pages. It includes a RAG chatbot embedded in the book (OpenAI Agents SDK + FastAPI + Qdrant + Neon PostgreSQL). The project phases are: 1. Book content creation with Spec-Kit Plus, 2. FastAPI backend with single /chat endpoint, 3. Chatbot integration on UI (ChatKit), 4. UI enhancement with professional robotics theme, and 5. RAG implementation with Qdrant Cloud and content sync. BONUS FEATURES TO SUPPORT: User authentication (better-auth.com) with background questions, content personalization per user, and Urdu translation at chapter level. TECHNICAL STACK: Frontend includes Docusaurus, React, and ChatKit; Backend includes FastAPI and OpenAI Agents SDK; Database includes Neon PostgreSQL (serverless) and Qdrant Cloud (vector DB); AI Tools include Claude Code, Spec-Kit Plus, and Gemini-CLI; Deployment includes GitHub Pages and Vercel. CONSTITUTION REQUIREMENTS: 1. Project Principles & Goals define success criteria for each phase, establish AI-native development philosophy, and set standards for reusable intelligence accumulation. 2. Technical Standards include code quality rules (type hints, docstrings, linting), testing requirements (coverage, unit/integration tests), security guidelines (API keys, user data), and performance benchmarks. 3. Workflow Rules follow the Spec-Kit Plus SDD workflow (Constitution → Specify → Clarify → Plan → Tasks → Implement), define Git branching strategy and commit conventions, collaboration guidelines between human and AI agents, and phase review and checkpoint process. 4. Phase-Specific Guidelines cover Phase 1 (Book Content): Docusaurus structure, content organization, Spec-Kit Plus usage; Phase 2 (Backend): FastAPI structure, endpoint design, error handling; Phase 3 (Frontend Integration): ChatKit setup, API communication, state management; Phase 4 (UI Enhancement): Theme consistency, responsive design, accessibility; Phase 5 (RAG System): Vector DB schema, embedding strategy, sync process. 5. Quality Gates & Validation ensure each phase must pass specific tests before moving to next phase, include code review criteria, performance and security audits, and documentation completeness checks. 6. Bonus Features Framework defines authentication flow design, personalization architecture, and translation implementation strategy. OUTPUT FORMAT: Generate a complete Constitution document in markdown format, structured with clear sections and actionable guidelines that AI agents can follow. The Constitution should be placed in .specify/memory/constitution.md in the project. The Constitution must be clear and testable (no vague statements), enforceable by AI agents during development, adaptive to phase-specific requirements, and supportive of reusable intelligence patterns.

## Response snapshot

Successfully created the initial project constitution document at `.specify/memory/constitution.md`. The document outlines project principles, technical standards, workflow rules, phase-specific guidelines, quality gates, and a framework for bonus features. The initial version is 1.0.0, ratified and last amended on 2025-12-16.

## Outcome

- ✅ Impact: Created initial project constitution document.
- 🧪 Tests: No tests were run for this document creation.
- 📁 Files: One file created/updated: .specify/memory/constitution.md
- 🔁 Next prompts: Proceed with project specification or initial tasks based on the constitution.
- 🧠 Reflection: The constitution was generated based on detailed user input, covering all requested sections and adhering to the specified format. The process involved extracting requirements, mapping them to constitutional principles, and populating a template. The Sync Impact Report was generated as an HTML comment within the constitution file itself. Manual updates are pending for other templates and command files to ensure consistency with the new constitution.

## Evaluation notes (flywheel)

- Failure modes observed: Failed to execute powershell script due to command not recognized. Used agent-native file write as fallback.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
