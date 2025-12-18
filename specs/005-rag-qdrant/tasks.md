# Actionable Tasks for Phase 5: RAG with Qdrant

This document outlines the actionable, dependency-ordered tasks for implementing the Retrieval-Augmented Generation (RAG) system using Qdrant Cloud. Tasks are organized by milestones and mapped to acceptance criteria from the specification.

## Implementation Strategy

The implementation will follow a milestone-driven, incremental approach. Each milestone builds upon the previous one, ensuring a stable foundation for the RAG system. MVP scope is considered to be the successful execution of Milestone 4, where the `/chat` endpoint provides grounded answers.

## Dependency Graph (User Story Completion Order)

Milestone 1 (Setup) -> Milestone 2 (Sync Tool) -> Milestone 3 (RAG Service) -> Milestone 4 (Backend Integration) -> Milestone 5 (Final Prep)

## Parallel Execution Opportunities

*   Some unit tests can be written in parallel with their respective implementations.
*   Development of `TextChunker`, `EmbeddingGenerator`, and `QdrantIngestor` utilities in Milestone 2 can be done in parallel once interfaces are defined.
*   Frontend `selected_text` testing in Milestone 5 can run in parallel once Milestone 4 is complete.

---

## Phase 1: Setup - Qdrant Cloud & Development Environment Setup

**Story Goal**: Establish the vector database infrastructure and local development tools, and configure the backend to connect to it.
**Independent Test Criteria**: A Python script can successfully connect to the Qdrant Cloud cluster and perform basic operations (e.g., create/list/delete a dummy collection).

- [x] T001 Create a Qdrant Cloud account and a new free-tier cluster.
- [x] T002 Create Qdrant collection `physical_ai_textbook` with `vector_size=1536` and `distance_metric=Cosine` (can be manual or via script)
- [x] T003 Obtain Qdrant API key and URL.
- [x] T004 Securely store Qdrant API key and URL in `backend/.env` (e.g., `QDRANT_API_KEY`, `QDRANT_URL`).
- [x] T005 Install `qdrant-client` Python library in the backend project: `poetry add qdrant-client` in `backend/`
- [x] T006 [P] Create `backend/scripts/test_qdrant_connection.py` to verify Qdrant connection and collection operations.
- [x] T007 Run `backend/scripts/test_qdrant_connection.py` to validate Qdrant connection (AC-001 validation).

---

## Phase 2: Foundational - Book Content Processing Pipeline ("Sync Tool")

**Story Goal**: Build a standalone tool to chunk, embed, and upload the textbook content to Qdrant.
**Independent Test Criteria**: Running `tools/sync_book_to_qdrant.py` successfully populates the Qdrant collection with points containing correct vectors and metadata.

- [x] T008 Create `tools/` directory in project root.
- [x] T009 Create `tools/sync_book_to_qdrant.py` script.
- [x] T010 [P] Implement `TextChunker` utility within `tools/sync_book_to_qdrant.py` to split Markdown files from `frontend/docs/` into coherent chunks (~300 words).
- [x] T011 [P] Implement `EmbeddingGenerator` utility within `tools/sync_book_to_qdrant.py` to use OpenAI embeddings API (`text-embedding-3-small`) to create vectors for chunks, including rate limiting and retry logic.
- [x] T012 [P] Implement `QdrantIngestor` utility within `tools/sync_book_to_qdrant.py` to format chunks with metadata (`source`, `chapter`, `section`) and upsert to Qdrant.
- [x] T013 Integrate `TextChunker`, `EmbeddingGenerator`, and `QdrantIngestor` into `tools/sync_book_to_qdrant.py` with command-line arguments.
- [ ] T014 Run `tools/sync_book_to_qdrant.py` with `frontend/docs/` as input to populate Qdrant (AC-002 validation).
- [ ] T015 Verify sample Qdrant points for correct data, vectors, and metadata.

---

## Phase 3: Core RAG Service Module [US1]

**Story Goal**: Develop the backend service module (`rag_service.py`) that performs retrieval from Qdrant.
**Independent Test Criteria**: The `retrieve_relevant_context` function returns relevant text chunks with accurate metadata for example queries, both with and without `selected_text_hint`.

- [ ] T016 [US1] Create `backend/src/services/rag_service.py` module.
- [ ] T017 [P] [US1] Implement `_generate_query_embedding(query: str)` in `backend/src/services/rag_service.py` using OpenAI embeddings API.
- [ ] T018 [P] [US1] Implement `_build_qdrant_filter(selected_text_hint: Optional[str])` in `backend/src/services/rag_service.py` to parse `selected_text_hint` and construct Qdrant filter clauses.
- [ ] T019 [US1] Implement `retrieve_relevant_context(query: str, limit: int = 3, selected_text_hint: Optional[str] = None)` in `backend/src/services/rag_service.py`.
- [ ] T020 [US1] Write unit tests for `retrieve_relevant_context` in `backend/tests/unit/test_rag_service.py` using mock Qdrant client (AC-003, AC-006 validation).

---

## Phase 4: Backend Integration & Enhanced Chat Endpoint [US2]

**Story Goal**: Modify the FastAPI backend to use the RAG service and provide grounded answers via the `/chat` endpoint.
**Independent Test Criteria**: The `/chat` endpoint returns accurate, grounded answers, handles cases with no relevant context, and correctly incorporates `selected_text` for filtering.

- [ ] T021 [US2] Integrate `rag_service` into FastAPI's dependency injection/service layer in `backend/src/main.py`.
- [ ] T022 [US2] Modify the `POST /chat` endpoint logic in `backend/src/main.py` to:
    *   Extract `message`, `session_id`, `selected_text`.
    *   Call `rag_service.retrieve_relevant_context`.
    *   Concatenate retrieved passages.
    *   Construct strict system prompt.
    *   Pass prompt and history to OpenAI.
- [ ] T023 [US2] Update Pydantic `ChatResponse` model in `backend/src/schemas.py` to include optional `sources: List[str]`.
- [ ] T024 [P] [US2] Create a new `POST /ingest` (admin) endpoint in `backend/src/main.py` that securely triggers the `sync_book_to_qdrant.py` tool.
- [ ] T025 [US2] Implement basic authentication for the `/ingest` endpoint in `backend/src/main.py`.
- [ ] T026 [US2] Write integration tests for modified `/chat` endpoint in `backend/tests/integration/test_rag_chat.py` (AC-001, AC-004, AC-005, AC-006 validation).
- [ ] T027 [US2] Write integration tests for `/ingest` endpoint in `backend/tests/integration/test_ingest_endpoint.py` (AC-007 validation).

---

## Phase 5: System Integration, Testing & Final Demo Prep [US3]

**Story Goal**: Ensure the full RAG system works reliably and is ready for the hackathon demo.
**Independent Test Criteria**: The entire system functions end-to-end; chatbot answers are accurate and book-specific; the project is stable, performant, and well-documented.

- [ ] T028 [US3] Perform full integration test: Update sample Markdown in `frontend/docs/`, run `/ingest`, query via simulated chat UI, verify grounded response.
- [ ] T029 [US3] Thoroughly test `selected_text` feature end-to-end through simulated UI interactions.
- [ ] T030 [US3] Perform load testing on the `/chat` endpoint to ensure acceptable latency under load.
- [ ] T031 [US3] Add monitoring/logging for RAG steps (embedding calls, search results, latency) in `backend/src/config/logging.py` or relevant service files.
- [ ] T032 [US3] Update `README.md` with RAG architecture, sync tool instructions, and Qdrant/OpenAI configuration details.
- [ ] T033 [US3] Verify all Acceptance Criteria from `spec.md` (AC-001 to AC-007) are met and document findings.

---

## Independent Test Criteria Summary

*   **Phase 1 (Setup)**: `backend/scripts/test_qdrant_connection.py` connects to Qdrant and performs basic collection operations.
*   **Phase 2 (Sync Tool)**: Running `tools/sync_book_to_qdrant.py` populates Qdrant with correct data; sample points verified.
*   **Phase 3 (Core RAG Service)**: Unit tests for `rag_service.retrieve_relevant_context` pass for various queries and hints.
*   **Phase 4 (Backend Integration)**: Integration tests for `/chat` and `/ingest` pass, demonstrating grounded responses and successful ingestion.
*   **Phase 5 (System Integration)**: End-to-end tests demonstrate a functional RAG system with accurate, book-specific, and performant responses. All ACs from `spec.md` are verified.
