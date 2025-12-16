<!--
Sync Impact Report:
Version change: 0.0.0 -> 1.0.0
Modified principles:
  - PRINCIPLE_1_NAME (AI-Native Development & Reusable Intelligence)
  - PRINCIPLE_2_NAME (Technical Standards)
  - PRINCIPLE_3_NAME (Development Workflow)
  - PRINCIPLE_4_NAME (Phase-Specific Guidelines)
  - PRINCIPLE_5_NAME (Quality Gates & Validation)
  - PRINCIPLE_6_NAME (Bonus Features Framework)
Added sections: Technical Standards, Development Workflow, Phase-Specific Guidelines, Quality Gates & Validation, Bonus Features Framework
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md (⚠ pending)
  - .specify/templates/spec-template.md (⚠ pending)
  - .specify/templates/tasks-template.md (⚠ pending)
  - .gemini/commands/sp.adr.toml (⚠ pending)
  - .gemini/commands/sp.analyze.toml (⚠ pending)
  - .gemini/commands/sp.checklist.toml (⚠ pending)
  - .gemini/commands/sp.clarify.toml (⚠ pending)
  - .gemini/commands/sp.constitution.toml (⚠ pending)
  - .gemini/commands/sp.git.commit_pr.toml (⚠ pending)
  - .gemini/commands/sp.implement.toml (⚠ pending)
  - .gemini/commands/sp.phr.toml (⚠ pending)
  - .gemini/commands/sp.plan.toml (⚠ pending)
  - .gemini/commands/sp.specify.toml (⚠ pending)
  - .gemini/commands/sp.tasks.toml (⚠ pending)
Follow-up TODOs: None
-->
# AI-native Textbook for Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. AI-Native Development & Reusable Intelligence
*   **Success Criteria**: Each project phase MUST have clearly defined and measurable success criteria.
*   **AI-Native Philosophy**: Development MUST embrace AI as a first-class collaborator and enabler, focusing on tools like Spec-Kit Plus, Claude Code, and Gemini-CLI for accelerated, spec-driven development.
*   **Reusable Intelligence**: Components, patterns, and generated artifacts MUST be designed for maximum reusability and accumulation of intelligence across the project and potentially future endeavors.

### II. Technical Standards
*   **Code Quality**:
    *   **Type Hints**: All Python and TypeScript code MUST extensively use type hints.
    *   **Docstrings/Comments**: Public functions, classes, and complex logic MUST include clear docstrings (Python) or JSDoc-style comments (TypeScript/JavaScript) explaining purpose, arguments, and return values.
    *   **Linting**: Code MUST adhere to project-defined linting rules, enforced via pre-commit hooks and CI/CD pipelines.
*   **Testing Requirements**:
    *   **Coverage**: Strive for high test coverage for critical components; minimum acceptable coverage percentage will be defined per component.
    *   **Unit Tests**: All new functions and methods MUST have corresponding unit tests.
    *   **Integration Tests**: Key interactions between services (e.g., FastAPI endpoints, Qdrant integration) MUST have integration tests.
*   **Security Guidelines**:
    *   **API Keys/Secrets**: Sensitive information (e.g., API keys, database credentials) MUST NEVER be hardcoded and MUST be managed via environment variables or secure secret management systems.
    *   **User Data**: Handling of user data MUST comply with privacy best practices and relevant regulations.
*   **Performance Benchmarks**: Critical paths (e.g., `/chat` endpoint latency) MUST have defined performance benchmarks and be monitored.

### III. Development Workflow
*   **Spec-Driven Development (SDD)**: The project MUST strictly follow the Spec-Kit Plus SDD workflow: Constitution → Specify → Clarify → Plan → Tasks → Implement.
*   **Git Strategy**:
    *   **Branching**: `main` branch MUST always be deployable. Features MUST be developed on dedicated feature branches.
    *   **Commit Conventions**: Commit messages MUST follow Conventional Commits specification.
*   **Collaboration Guidelines**:
    *   **Human-AI Interaction**: Human developers and AI agents (e.g., Gemini-CLI, Claude Code) MUST collaborate by leveraging each other's strengths, with AI handling repetitive tasks and initial drafts, and humans providing oversight, refinement, and strategic direction.
    *   **Prompt Engineering**: Clear and concise prompt engineering MUST be practiced for effective AI agent interaction.
*   **Phase Review & Checkpoint**: Each phase MUST conclude with a formal review and approval process before proceeding to the next.

### IV. Phase-Specific Guidelines
*   **Phase 1 (Book Content Creation)**:
    *   **Docusaurus Structure**: Content MUST adhere to a predefined Docusaurus file and directory structure.
    *   **Content Organization**: Chapters and sections MUST be logically organized and follow a consistent narrative flow for "Physical AI & Humanoid Robotics".
    *   **Spec-Kit Plus Usage**: Content generation MUST leverage Spec-Kit Plus for structured creation and consistency.
*   **Phase 2 (FastAPI Backend)**:
    *   **FastAPI Structure**: Backend MUST follow a clear, modular FastAPI application structure.
    *   **Endpoint Design**: The `/chat` endpoint MUST be robust, well-documented, and handle requests and responses according to defined API contracts.
    *   **Error Handling**: Backend MUST implement comprehensive error handling and logging.
*   **Phase 3 (Chatbot Integration on UI)**:
    *   **ChatKit Setup**: Chatbot UI MUST be integrated using ChatKit, adhering to its best practices.
    *   **API Communication**: Frontend-to-backend communication for the chatbot MUST be secure and efficient.
    *   **State Management**: UI state related to the chatbot MUST be managed effectively (e.g., React Context).
*   **Phase 4 (UI Enhancement)**:
    *   **Theme Consistency**: UI MUST maintain a consistent, professional robotics theme across all pages.
    *   **Responsive Design**: UI MUST be fully responsive and optimized for various device sizes.
    *   **Accessibility**: UI MUST adhere to web accessibility standards (WCAG 2.1 AA).
*   **Phase 5 (RAG System Implementation)**:
    *   **Vector DB Schema**: Qdrant Cloud vector database MUST have a well-defined schema for efficient retrieval.
    *   **Embedding Strategy**: A consistent and effective embedding model and strategy MUST be used for content.
    *   **Content Sync Process**: A reliable process MUST be established for syncing book content with the RAG system.

### V. Quality Gates & Validation
*   **Phase Exit Criteria**: Each phase MUST pass all defined quality gates before transitioning to the next.
*   **Code Review**: All code MUST undergo peer review, with AI-assisted reviews (e.g., Copilot) encouraged.
*   **Performance & Security Audits**: Regular audits MUST be conducted to ensure performance and security compliance.
*   **Documentation Completeness**: Documentation MUST be thorough, up-to-date, and aligned with code.

### VI. Bonus Features Framework
*   **Authentication Flow**: User authentication (better-auth.com) MUST be designed with clear user journeys and secure practices, including background questions for personalization.
*   **Personalization Architecture**: The system MUST support content personalization based on user profiles and preferences.
*   **Translation Strategy**: Urdu translation MUST be implementable at the chapter level, ensuring maintainability and accuracy.

## Governance
*   This Constitution supersedes all other project practices.
*   Amendments MUST be documented, approved by core contributors, and include a clear migration plan if necessary.
*   All Pull Requests and code reviews MUST verify compliance with these principles.
*   Any deviation or complexity MUST be explicitly justified.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16