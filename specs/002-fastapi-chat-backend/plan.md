# Implementation Plan: Minimalistic FastAPI Backend for Chat

**Feature Branch**: `002-fastapi-chat-backend`
**Created**: 2025-12-17
**Status**: Draft

## Technical Context

This implementation plan focuses on building "Phase 2: Minimalistic FastAPI Backend" for a "Physical AI & Humanoid Robotics" textbook chatbot. The backend will provide a `/chat` endpoint for user interaction, integrating with OpenAI's API for responses, and persisting conversation history in a Neon PostgreSQL database.

**Key Technologies:**
-   **FastAPI**: Asynchronous web framework for building APIs.
-   **Python 3.12+**: Programming language.
-   **OpenAI Python SDK (openai package)**: For interacting with OpenAI's Chat Completions API.
-   **Neon (Serverless PostgreSQL)**: Cloud-native PostgreSQL for database persistence.
-   **async SQLAlchemy 2.0 with Alembic**: ORM for database interaction and migrations.
-   **Pydantic v2**: For data validation and serialization.
-   **uv or Poetry**: For package management.
-   **uvicorn**: ASGI server to run FastAPI application.
-   **python-dotenv**: For managing environment variables.
-   **asyncpg**: PostgreSQL driver for async operations.
-   **pytest**: For unit and integration testing.
-   **ruff**: For linting.
-   **black**: For code formatting.
-   **Dockerfile**: For containerization and deployment.
-   **Vercel**: Target platform for serverless deployment.

**Assumptions:**
-   A Neon PostgreSQL account and database are provisioned and accessible.
-   An OpenAI API key is available.
-   The frontend (Phase 3) will be a Docusaurus application running on `localhost:3000` during local development.
-   The "selected_text" bonus feature will directly inject the text into the OpenAI prompt or context, requiring careful prompt engineering.

**Unknowns / Clarifications Needed:**
-   **Logging Level and Granularity**: While structured logging is mandated, the specific logging levels (INFO, DEBUG, ERROR) and granularity for different events (e.g., each message, API call details) need to be detailed. This directly impacts verbosity and storage costs. [NEEDS CLARIFICATION: Define specific logging levels and event granularity for structured logging.]
-   **Message History Strategy**: The spec mentions "approximately the last ten messages." This should be more precisely defined (e.g., exactly 10, or up to 10 if fewer exist, or if there are other criteria to select messages). [NEEDS CLARIFICATION: Precisely define the strategy for selecting and sending "approximately the last ten messages" to the OpenAI API.]

## Constitution Check

**Relevant Principles**:
-   **I. AI-Native Development & Reusable Intelligence**: This phase aligns by building a core AI interaction component that will be reusable for future phases (RAG). Success criteria are defined per milestone.
-   **II. Technical Standards**:
    -   **Code Quality**: Type hints, docstrings, and linting (ruff, black) are explicitly included in Deliverables and Acceptance Criteria.
    -   **Testing Requirements**: Unit and integration tests with coverage are mandated.
    -   **Security Guidelines**: API keys/secrets via environment variables are specified.
    -   **Performance Benchmarks**: `/chat` endpoint latency is set at <2 seconds.
-   **III. Development Workflow**: Follows SDD by creating a detailed plan. Git strategy (feature branch, Conventional Commits) will be adhered to during implementation.
-   **IV. Phase-Specific Guidelines (Phase 2)**: Adheres to FastAPI structure, robust endpoint design, and comprehensive error handling.
-   **V. Quality Gates & Validation**: Milestones include explicit validation steps. Code review and performance audits are part of overall project governance.

**Violations / Justifications**:
-   None. The plan explicitly incorporates and aligns with all relevant constitution principles.

## Phase 0: Outline & Research

**Objective**: Resolve remaining ambiguities and establish best practices for key technologies.

### Key Tasks:
1.  **Research Logging Strategy**:
    *   Investigate best practices for structured logging in FastAPI applications (e.g., `Loguru`, `structlog`).
    *   Determine appropriate logging levels and detail for requests, responses, API calls, and errors.
2.  **Clarify Message History Selection**:
    *   Define the exact logic for selecting "approximately the last ten messages" to maintain conversation context (e.g., strictly last 10, or context window management).

### Deliverables:
-   `specs/002-fastapi-chat-backend/research.md`: Documented decisions for logging strategy and message history selection.

### Dependencies:
-   `specs/002-fastapi-chat-backend/spec.md` approved.

### Validation Criteria:
-   `research.md` exists and addresses all `[NEEDS CLARIFICATION]` markers from `plan.md`'s Technical Context section.

## Phase 1: Project Bootstrap & Configuration

**Objective**: Create the project foundation with all tools and configuration ready.

### Key Tasks:
1.  **Setup Python Environment**:
    *   Initialize Python 3.12+ virtual environment.
    *   Configure `pyproject.toml` (if using poetry) or `requirements.txt` (if using uv).
2.  **Install Core Dependencies**:
    *   Install `fastapi`, `uvicorn`, `openai`, `sqlalchemy`, `alembic`, `pydantic`, `python-dotenv`, `asyncpg`.
3.  **Establish Project Structure**:
    *   Create base directories: `src/`, `alembic/`, `tests/`, `requirements/`.
4.  **Configure Environment Variables**:
    *   Create `.env.example` with placeholders for `OPENAI_API_KEY`, `DATABASE_URL`, and `FRONTEND_URL`.
5.  **Initialize FastAPI App**:
    *   Create a basic `src/main.py` with a FastAPI instance.
    *   Implement CORS middleware allowing `localhost:3000` and `FRONTEND_URL`.
    *   Implement `GET /health` endpoint returning `{"status": "healthy"}`.

### Deliverables:
-   Python 3.12+ environment (virtualenv or poetry project).
-   `pyproject.toml` (or `requirements.txt`).
-   Installed dependencies: `fastapi`, `uvicorn`, `openai`, `sqlalchemy`, `alembic`, `pydantic`, `python-dotenv`, `asyncpg`.
-   Project directory structure: `src/`, `alembic/`, `tests/`, `requirements/`.
-   `.env.example` file.
-   `src/main.py` with basic FastAPI app, CORS, and `/health` endpoint.

### Dependencies:
-   Phase 0 (Research) complete.

### Validation Criteria:
-   `uvicorn src.main:app --reload` runs successfully without errors.
-   Accessing `http://localhost:8000/health` returns `200 OK` with `{"status": "healthy"}`.
-   CORS headers are correctly set for allowed origins.

## Phase 2: Database Integration & Data Layer

**Objective**: Set up Neon PostgreSQL and implement all data models and migrations.

### Key Tasks:
1.  **Provision Neon PostgreSQL**:
    *   Create a Neon PostgreSQL instance.
    *   Obtain the connection string and update `.env.example` accordingly.
2.  **Define SQLAlchemy Models**:
    *   Implement `ChatSession` model: `id` (UUID, PK), `created_at` (Timestamp), `updated_at` (Timestamp).
    *   Implement `Message` model: `id` (UUID, PK), `session_id` (UUID, FK to `ChatSession.id`), `role` (Enum: 'user', 'assistant', 'system'), `content` (Text), `created_at` (Timestamp).
3.  **Configure Alembic**:
    *   Initialize Alembic in the `alembic/` directory.
    *   Configure `alembic.ini` and `env.py` for async SQLAlchemy.
    *   Generate initial migration script reflecting `ChatSession` and `Message` models.
4.  **Implement Async Database Utilities**:
    *   Create an async database engine and session factory.
    *   Develop utilities for dependency injection of database sessions.
5.  **Implement Basic CRUD Operations**:
    *   Create repository functions for `ChatSession`: `find_by_id`, `create`.
    *   Create repository functions for `Message`: `create`, `get_last_n_messages_for_session`.

### Deliverables:
-   Configured Neon PostgreSQL instance.
-   Updated `.env.example` with `DATABASE_URL`.
-   `src/models.py` with `ChatSession` and `Message` SQLAlchemy models.
-   Configured Alembic directory (`alembic/`).
-   Initial Alembic migration script.
-   `src/database.py` with async database engine, session factory, and dependency.
-   `src/repositories/chat_repository.py` with basic CRUD functions for `ChatSession` and `Message`.

### Dependencies:
-   Milestone 1 complete.
-   `research.md` decisions on message history are incorporated.

### Validation Criteria:
-   `alembic upgrade head` runs successfully, creating `chat_sessions` and `messages` tables in the Neon database.
-   Python scripts can connect to the Neon database and perform basic inserts/reads using the defined models.

## Phase 3: Core Chat API & OpenAI Integration

**Objective**: Implement the main `/chat` endpoint with complete OpenAI integration.

### Key Tasks:
1.  **Define Pydantic Models**:
    *   Implement `ChatRequest` (message: str, session_id: Optional[str], selected_text: Optional[str]).
    *   Implement `ChatResponse` (response: str, session_id: str, timestamp: str).
2.  **Develop Service Layer**:
    *   `session_manager`: Logic for `find_or_create_session(session_id: str | None) -> ChatSession`.
    *   `message_repository`: Functions for saving user/assistant messages and retrieving conversation history.
    *   `openai_client`: Encapsulate calls to OpenAI Chat Completions API.
3.  **Implement System Prompt Engineering**:
    *   Construct the system prompt to guide the AI as a helpful assistant for the "Physical AI & Humanoid Robotics" textbook.
4.  **Implement `/chat` Endpoint Logic**:
    *   Receive `ChatRequest` and validate using Pydantic.
    *   Use `session_manager` to get or create `ChatSession`.
    *   Save user's incoming message to the database.
    *   Retrieve the last ten messages for context, adhering to `research.md` decisions.
    *   Inject `selected_text` into the OpenAI call context (e.g., in the system prompt or as a specific message).
    *   Call `openai_client` for completions.
    *   Save AI's response to the database.
    *   Return formatted `ChatResponse`.

### Deliverables:
-   `src/schemas.py` with `ChatRequest`, `ChatResponse` Pydantic models.
-   `src/services/session_manager.py`.
-   `src/services/message_repository.py` (or extend existing).
-   `src/services/openai_client.py`.
-   Updated `src/main.py` with the functional `POST /chat` endpoint.
-   System prompt defined and integrated.

### Dependencies:
-   Milestone 2 complete.
-   `research.md` decisions on message history and logging are implemented.

### Validation Criteria:
-   The `POST /chat` endpoint accepts valid requests and returns a `200 OK` response with a correctly formatted `ChatResponse`.
-   Successful calls to `/chat` result in new `Message` entries (user and assistant) in the database.
-   The AI's responses for consecutive calls with the same `session_id` demonstrate contextual awareness.
-   When `selected_text` is provided, the AI's response is grounded in that text.
-   OpenAI API is successfully called and returns responses.

## Phase 4: Robustness, Testing & Deployment Prep

**Objective**: Harden the application and prepare it for Phase 3 integration.

### Key Tasks:
1.  **Implement Comprehensive Error Handling**:
    *   Create custom HTTP exception handlers for Pydantic validation errors (400), database errors (500), and OpenAI errors (500).
    *   Ensure consistent error response format.
2.  **Integrate Structured Logging**:
    *   Set up structured logging (e.g., `Loguru` or `structlog`) to log requests, responses, and errors as defined in `research.md`.
3.  **Input Validation & Sanitization**:
    *   Review and enhance Pydantic models for stricter validation.
    *   Implement any necessary input sanitization (though Pydantic often handles much of this).
4.  **Develop Unit Tests**:
    *   Write `pytest` unit tests for `Pydantic models`, `SQLAlchemy models`, and core service functions (e.g., `session_manager`, `openai_client`).
5.  **Develop Integration Tests**:
    *   Write `pytest` integration tests for the `POST /chat` endpoint, including mocking the OpenAI API calls.
    *   Test various scenarios: new session, existing session, `selected_text`, error conditions.
6.  **Enforce Code Quality**:
    *   Add type hints to all relevant code.
    *   Add docstrings to all public functions, classes, and complex logic.
    *   Configure and integrate `ruff` for linting and `black` for formatting.
7.  **Documentation**:
    *   Update `README.md` with setup instructions, running the app locally, and basic API usage.
    *   Generate API documentation (FastAPI's auto-generated docs should suffice initially).
8.  **Deployment Preparation**:
    *   Create a simple `Dockerfile` for containerizing the application.
    *   Develop a basic environment validation script to check for mandatory environment variables.

### Deliverables:
-   Custom exception handlers in `src/exceptions.py` (or similar).
-   Integrated structured logging configuration in `src/main.py` and service layers.
-   Enhanced Pydantic models.
-   `tests/unit/` directory with unit tests.
-   `tests/integration/` directory with integration tests for `/chat`.
-   Type hints and docstrings across the codebase.
-   `pyproject.toml` or equivalent with `ruff` and `black` configurations.
-   Updated `README.md`.
-   `Dockerfile`.
-   Environment validation script.

### Dependencies:
-   Milestone 3 complete.

### Validation Criteria:
-   All `pytest` tests pass successfully.
-   Code coverage for critical paths is above 80%.
-   `ruff` and `black` run without reporting issues.
-   Error conditions (e.g., invalid input, simulated OpenAI error) return correct HTTP status codes and structured error responses.
-   Logs are generated in a structured format and contain relevant information.
-   `README.md` provides clear instructions for setup and running.
-   The backend can be built into a Docker image.
-   The environment validation script correctly identifies missing mandatory environment variables.

## Conclusion

This plan outlines a phased approach to developing the Minimalistic FastAPI Backend for Chat. Upon successful completion of Phase 4, the backend will be robust, well-tested, and ready for integration with the ChatKit frontend in Phase 3 of the overall project.