---

description: "Task list for the Minimalistic FastAPI Backend for Chat feature implementation"
---

# Tasks: Minimalistic FastAPI Backend for Chat

**Input**: Design documents from `/specs/002-fastapi-chat-backend/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Project Bootstrap & Configuration)

**Purpose**: Create the project foundation with all tools and configuration ready.

- [X] T001 Setup Python Environment (Python 3.12+ virtual environment, `pyproject.toml` or `requirements.txt`)
- [X] T002 Install Core Dependencies (`fastapi`, `uvicorn`, `google-generativeai`, `sqlalchemy`, `alembic`, `pydantic`, `python-dotenv`, `asyncpg`)
- [X] T003 Establish Project Structure (`src/`, `alembic/`, `tests/`, `requirements/`)
- [X] T004 Configure Environment Variables (`.env.example` with `GEMINI_API_KEY`, `DATABASE_URL`, `FRONTEND_URL`)
- [X] T005 Initialize FastAPI App (`src/main.py`)
- [X] T006 Implement CORS middleware allowing `localhost:3000` and `FRONTEND_URL` (`src/main.py`)
- [X] T007 Implement `GET /health` endpoint returning `{"status": "healthy"}` (`src/main.py`)

---

## Phase 2: Foundational (Database Integration & Data Layer)

**Purpose**: Set up Neon PostgreSQL and implement all data models and migrations.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T008 Provision Neon PostgreSQL instance and obtain connection string (External - documentation task). Ensure 'uuid-ossp' extension is enabled for UUID generation.
- [X] T009 Define `ChatSession` SQLAlchemy Model (`src/models.py`)
- [X] T010 Define `Message` SQLAlchemy Model (`src/models.py`)
- [X] T011 Configure Alembic (`alembic/`)
- [X] T012 Generate initial migration script reflecting `ChatSession` and `Message` models (`alembic/versions/`)
- [X] T013 Implement Async Database Engine and Session Factory (`src/database.py`)
- [X] T014 Develop Database Session Dependency Injection (`src/database.py`)
- [X] T015 Implement Repository Functions for `ChatSession`: `find_by_id`, `create`, and `Message`: `create`, `get_last_n_messages_for_session` (`src/repositories/chat_repository.py`)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Initiate New Chat Session (Priority: P1) 🎯 MVP

**Goal**: A user accesses the chat interface for the first time or wishes to start a new conversation.

**Independent Test**: Can be fully tested by sending a chat message without a `session_id` and verifying that a new session is created, the AI responds, and the interaction is saved.

### Tests for User Story 1

- [X] T016 [P] [US1] Unit test Pydantic `ChatRequest`, `ChatResponse`, `HealthResponse` (`tests/unit/test_schemas.py`)
- [X] T017 [P] [US1] Unit test SQLAlchemy Models (`ChatSession`, `Message`) (`tests/unit/test_models.py`)
- [X] T018 [US1] Integration test new chat session flow (`tests/integration/test_chat_new_session.py`)

### Implementation for User Story 1

- [X] T019 [P] [US1] Define Pydantic Models: `ChatRequest`, `ChatResponse`, `HealthResponse` (`src/schemas.py`)
- [X] T020 [P] [US1] Implement `session_manager` logic for `find_or_create_session` (`src/services/session_manager.py`)
- [X] T021 [P] [US1] Implement `message_repository` to save user message (`src/services/message_repository.py`)
- [X] T022 [P] [US1] Construct system prompt for AI persona (`src/services/openai_client.py`)
- [X] T023 [P] [US1] Implement `openai_client` to call OpenAI Chat Completions API (`src/services/openai_client.py`)
- [X] T024 [US1] Implement `POST /chat` endpoint logic for new sessions (no `session_id` provided) (`src/main.py`)
- [X] T025 [US1] Implement `message_repository` to save AI response (`src/services/message_repository.py`)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Continue Existing Chat Session (Priority: P1)

**Goal**: A user continues a conversation with the chatbot, expecting context to be maintained.

**Independent Test**: Can be fully tested by sending multiple messages with the same `session_id` and verifying that the AI's responses are contextually relevant, and all messages are correctly linked in the database.

### Tests for User Story 2

- [ ] T026 [US2] Integration test existing chat session flow with context (`tests/integration/test_chat_existing_session.py`)

### Implementation for User Story 2

- [ ] T027 [US2] Enhance `session_manager` logic to find existing session (`src/services/session_manager.py`)
- [ ] T028 [US2] Implement `message_repository` to retrieve last 10 messages for context (`src/services/message_repository.py`)
- [ ] T029 [US2] Update `POST /chat` endpoint to send conversation history context to OpenAI (`src/main.py`)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Get Answer from Selected Text (Priority: P2)

**Goal**: A user provides specific text from the book and expects the AI's response to be based *solely* on that text, ignoring previous conversation context.

**Independent Test**: Can be fully tested by sending a message with both `message` and `selected_text` fields populated and verifying the AI's response directly addresses the `selected_text`, overriding general context.

### Tests for User Story 3

- [ ] T030 [US3] Integration test `selected_text` functionality (`tests/integration/test_chat_selected_text.py`)

### Implementation for User Story 3

- [ ] T031 [US3] Modify `openai_client` and `POST /chat` endpoint to inject `selected_text` into prompt context (`src/services/openai_client.py`, `src/main.py`)

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Harden the application and prepare it for Phase 3 integration.

- [ ] T032 Implement Comprehensive Error Handling for Pydantic, OpenAI, Database, and generic errors (`src/exceptions.py`, `src/main.py`)
- [ ] T033 Integrate Structured Logging (`Loguru`) as defined in `research.md` (`src/main.py`, `src/config/logging.py` or similar)
- [ ] T034 Review and enhance Pydantic models for stricter input validation (`src/schemas.py`)
- [ ] T035 Develop Unit Tests for Core Logic Components (`session_manager`, `openai_client`) (`tests/unit/`)
- [ ] T036 Develop Integration Tests for Error Scenarios (e.g., OpenAI API failure, DB connection loss) (`tests/integration/`)
- [ ] T037 Add Type Hints and Docstrings to all public functions, classes, and complex logic (across codebase)
- [ ] T038 Configure and Integrate `ruff` for linting and `black` for formatting (`pyproject.toml`)
- [ ] T039 Update `README.md` with setup instructions, running the app locally, and basic API usage (`README.md`)
- [ ] T040 Create a simple `Dockerfile` and environment validation script (`Dockerfile`, `scripts/validate_env.py` or similar)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- Many tasks within Phase 1, Phase 2, and the "Tests" sub-sections of user stories are marked `[P]` and can run in parallel.
- Once the Foundational phase completes, different user stories can be worked on in parallel by different team members.

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together:
Task: "Unit test Pydantic ChatRequest, ChatResponse, HealthResponse in tests/unit/test_schemas.py"
Task: "Unit test SQLAlchemy Models (ChatSession, Message) in tests/unit/test_models.py"
Task: "Integration test new chat session flow in tests/integration/test_chat_new_session.py"

# Launch these implementation tasks for User Story 1 together:
Task: "Define Pydantic Models: ChatRequest, ChatResponse, HealthResponse in src/schemas.py"
Task: "Implement session_manager logic for find_or_create_session in src/services/session_manager.py"
Task: "Implement message_repository to save user message in src/services/message_repository.py"
Task: "Construct system prompt for AI persona in src/services/openai_client.py"
Task: "Implement openai_client to call OpenAI Chat Completions API in src/services/openai_client.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- `[P]` tasks = different files, no dependencies
- `[Story]` label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
