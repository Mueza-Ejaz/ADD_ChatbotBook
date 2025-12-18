# Implementation Plan: Phase 5 - RAG with Qdrant

## 1. Technical Context

This phase focuses on implementing Retrieval-Augmented Generation (RAG) to enhance the existing FastAPI chat backend with a Qdrant Cloud vector database. The goal is to provide accurate, book-specific answers by grounding the OpenAI model's responses in the textbook content.

**Key Components:**
*   **Qdrant Cloud**: Vector database for storing and searching textbook content embeddings.
*   **OpenAI Embedding API**: Used for generating vector embeddings for text chunks and user queries.
*   **Sync Tool (`sync_book_to_qdrant.py`)**: A standalone Python script to process Docusaurus Markdown files, chunk them, embed them, and upload them to Qdrant.
*   **RAG Service (`rag_service.py`)**: A new backend module responsible for orchestrating the retrieval process, including query embedding, Qdrant search, and returning relevant text passages.
*   **FastAPI `/chat` Endpoint**: Modification of the existing endpoint to integrate the RAG service, construct context-aware prompts, and pass them to the OpenAI chat completion API.
*   **Optional `/ingest` Endpoint**: An administrative endpoint to trigger the sync tool.

**Data Flow (RAG):**
1.  User query and optional `selected_text` received by the backend `/chat` endpoint.
2.  `rag_service.py` is called with the query and `selected_text`.
3.  `rag_service.py` generates an embedding for the query.
4.  `rag_service.py` performs a similarity search in Qdrant, optionally filtering by metadata derived from `selected_text`.
5.  Relevant text passages are retrieved from Qdrant.
6.  A strict system prompt is constructed for the OpenAI model, instructing it to answer ONLY based on the provided context.
7.  The context-aware prompt and conversation history are sent to the OpenAI chat completion API.
8.  The grounded answer is returned to the user.

## 2. Constitution Check

*   **AI-Native Development & Reusable Intelligence**:
    *   **Success Criteria**: Clearly defined per milestone and in the `spec.md`.
    *   **AI-Native Philosophy**: Leverages OpenAI APIs and integrates with existing FastAPI backend, consistent with AI-native development.
    *   **Reusable Intelligence**: `rag_service.py` and `sync_book_to_qdrant.py` are designed as reusable components.

*   **Technical Standards**:
    *   **Code Quality**: Type hints, docstrings, and linting will be enforced for new Python code (`rag_service.py`, sync tool).
    *   **Testing Requirements**: Unit and integration tests are explicitly planned for `rag_service.py` and the `/chat` endpoint.
    *   **Security Guidelines**: API keys for OpenAI and Qdrant will be managed via environment variables. `POST /ingest` endpoint will require authentication.
    *   **Performance Benchmarks**: Latency and throughput NFRs are defined for the `/chat` endpoint.

*   **Development Workflow**:
    *   **Spec-Driven Development (SDD)**: This plan is a direct result of the provided `spec.md`.
    *   **Git Strategy**: Feature branch (`005-rag-qdrant`) is used for development. Commit conventions will be followed.
    *   **Collaboration Guidelines**: Human-AI interaction is ongoing (this planning phase).
    *   **Phase Review & Checkpoint**: This plan will be reviewed before implementation.

*   **Phase-Specific Guidelines (Phase 5 - RAG System Implementation)**:
    *   **Vector DB Schema**: Qdrant Cloud schema defined in `spec.md` with `physical_ai_textbook` collection and payload metadata.
    *   **Embedding Strategy**: `text-embedding-3-small` via OpenAI API is the chosen strategy.
    *   **Content Sync Process**: `sync_book_to_qdrant.py` tool is specifically designed for this purpose.

*   **Quality Gates & Validation**:
    *   **Phase Exit Criteria**: Acceptance criteria are detailed in `spec.md` and will be validated.
    *   **Code Review**: Code reviews will be conducted.
    *   **Performance & Security Audits**: Load testing is planned for the `/chat` endpoint.

*   **Bonus Features Framework**:
    *   **Selected Text Integration**: The `selected_text` feature is explicitly integrated into the RAG workflow to enhance search relevance.

**Overall**: The plan aligns well with the project constitution.

## 3. Phase 0: Outline & Research

**Research Tasks:**

1.  **Effective Text Chunking Strategies**: Investigate and compare different Markdown chunking strategies (e.g., recursive character text splitter, semantic chunking, fixed-size chunks with overlap) to determine the optimal approach for the "Physical AI & Humanoid Robotics" textbook.
    *   NEEDS CLARIFICATION: Best practices for chunking markdown content for RAG, especially considering Docusaurus structure (headings, code blocks, etc.).
2.  **OpenAI Embeddings API Integration Best Practices**: Research best practices for integrating with the OpenAI Embeddings API, including rate limits, error handling, and cost optimization.
    *   NEEDS CLARIFICATION: How to handle rate limiting effectively when embedding potentially thousands of chunks.
3.  **Qdrant Client Python Library Usage**: Review comprehensive examples and documentation for `qdrant-client` to ensure efficient and correct usage for collection creation, point insertion (upsert), and similarity search with filtering.
    *   NEEDS CLARIFICATION: Specific Qdrant filter syntax for complex metadata queries (e.g., combining source, chapter, section).
4.  **Prompt Engineering for RAG**: Explore advanced prompt engineering techniques for instructing LLMs to strictly adhere to provided context in a RAG setting, including handling cases where the answer is not in the context.
    *   NEEDS CLARIFICATION: Optimal phrasing for the strict system prompt to minimize LLM hallucination when context is limited or irrelevant.

## 4. Phase 1: Design & Contracts

### 4.1 Data Model (Qdrant Payload)

The Qdrant collection `physical_ai_textbook` will store points with the following payload (metadata) schema:

*   **`text`**: `str` - The original text content of the chunk.
*   **`source`**: `str` - The file path of the original Markdown document (e.g., `docs/module1/chapter-1.md`).
*   **`chapter`**: `str` - The title or identifier of the chapter the chunk belongs to.
*   **`section`**: `str` - The heading or subsection title the chunk belongs to.
*   **`vector`**: `List[float]` - The embedding vector generated by `text-embedding-3-small` (dimension 1536).

### 4.2 API Contracts

#### `rag_service.py`

**Function:** `get_relevant_context`

```python
# backend/src/services/rag_service.py

from typing import List, Dict, Optional

async def get_relevant_context(
    query: str,
    limit: int = 3,
    selected_text_hint: Optional[str] = None
) -> List[Dict]:
    """
    Retrieves relevant text passages from the Qdrant collection based on a query.

    Args:
        query: The user's question or search query.
        limit: The maximum number of relevant text passages to retrieve.
        selected_text_hint: Optional text selected by the user from the UI.
                            Used to bias search towards specific source/chapter/section.

    Returns:
        A list of dictionaries, each containing:
        - "text": The retrieved text chunk.
        - "source": The source file path.
        - "chapter": The chapter title.
        - "section": The section title.
    """
    pass # Implementation details will follow in later phases
```

#### FastAPI Endpoints

**1. `POST /chat` (Modified)**

*   **Method**: `POST`
*   **Path**: `/chat`
*   **Request Body**:
    ```json
    {
      "message": "string",
      "session_id": "string",
      "selected_text": "string (optional)"
    }
    ```
*   **Response Body**:
    ```json
    {
      "response": "string",
      "session_id": "string",
      "sources": ["string"] // Optional: List of source identifiers (e.g., chapter/section)
    }
    ```
*   **Behavior**: Integrates RAG logic. Uses `selected_text` to filter Qdrant search. Constructs context-aware prompt for OpenAI.

**2. `POST /ingest` (New - Admin Endpoint)**

*   **Method**: `POST`
*   **Path**: `/ingest`
*   **Authentication**: Requires administrative credentials/token (to be implemented).
*   **Request Body**:
    ```json
    {
      "force_resync": "boolean (optional, default: false)" // If true, wipes and resyncs all content
    }
    ```
*   **Response Body**:
    ```json
    {
      "status": "string",
      "message": "string",
      "ingested_count": "integer (optional)"
    }
    ```
*   **Behavior**: Triggers the `sync_book_to_qdrant.py` script.

## 5. Milestones

### Milestone 1: Qdrant Cloud & Development Environment Setup

*   **Objective**: Establish the vector database infrastructure and local development tools.
*   **Key Tasks**:
    1.  Create a Qdrant Cloud account and a new free-tier cluster.
    2.  Create a collection named `physical_ai_textbook` with `vector_size=1536` (for `text-embedding-3-small`) and `distance_metric=Cosine`.
    3.  Obtain and securely store the Qdrant API key and URL in the backend's `.env` file.
    4.  Install the `qdrant-client` Python library (`poetry add qdrant-client`) in the Phase 2 backend project.
    5.  Write a simple connection test script (e.g., `python backend/scripts/test_qdrant_connection.py`) to verify communication with the Qdrant cluster (create/list/delete a dummy collection).
*   **Deliverables**: A live, empty Qdrant collection (`physical_ai_textbook`) and a backend environment configured to connect to it.
*   **Validation**: The test script successfully connects to the Qdrant cluster and performs basic collection operations without errors.

### Milestone 2: Book Content Processing Pipeline ("Sync Tool")

*   **Objective**: Build the standalone tool to chunk, embed, and upload the textbook content.
*   **Key Tasks**:
    1.  Analyze the Phase 1 Docusaurus book structure (`frontend/docs/`) to locate all source Markdown files.
    2.  Implement a `TextChunker` utility that splits Markdown by headings or tokens into coherent chunks (~300 words). Research best practices for robust Markdown parsing and chunking.
    3.  Implement an `EmbeddingGenerator` utility that uses the OpenAI embeddings API (`text-embedding-3-small`) to create vectors for each chunk. Handle OpenAI API key securely and implement basic retry logic for API calls.
    4.  Implement a `QdrantIngestor` utility that formats each chunk with metadata (`source`, `chapter`, `heading`/`section`) and upserts it to the `physical_ai_textbook` collection.
    5.  Assemble these components into a runnable script: `python tools/sync_book_to_qdrant.py`. This script should accept command-line arguments for paths to Docusaurus `docs/` and optionally a flag to force full re-sync.
*   **Dependencies**: Milestone 1 must be complete. Requires access to Phase 1 book content.
*   **Deliverables**: A fully functional "sync tool" that can populate the vector database with the entire book content.
*   **Validation**: Running `python tools/sync_book_to_qdrant.py` successfully populates the Qdrant collection with hundreds/thousands of points, each with a vector and the defined metadata (`text`, `source`, `chapter`, `section`). Verify a sample of points for correct data.

### Milestone 3: Core RAG Service Module

*   **Objective**: Develop the backend service module that performs retrieval.
*   **Key Tasks**:
    1.  Create a `rag_service.py` module in `backend/src/services/`.
    2.  Implement `_generate_query_embedding(query: str)` private helper function using OpenAI embeddings API.
    3.  Implement `_build_qdrant_filter(selected_text_hint: Optional[str])` private helper function to parse `selected_text_hint` and construct Qdrant filter clauses based on inferred `source`, `chapter`, or `section`.
    4.  Implement the `retrieve_relevant_context(query: str, limit: int = 3, selected_text_hint: Optional[str] = None) -> List[Dict]` function. This function will:
        *   Generate embedding for the `query`.
        *   Build Qdrant filters using `selected_text_hint`.
        *   Perform a similarity search on the `physical_ai_textbook` collection.
        *   Return the top N text chunks along with their metadata.
    5.  Write unit tests for the retrieval logic in `rag_service.py` using a mock or in-memory Qdrant client to ensure correct embedding generation, filtering, and search invocation.
*   **Dependencies**: Milestones 1 & 2 must be complete (requires a populated Qdrant DB for realistic testing, even if mocked).
*   **Deliverables**: A tested, importable service module (`rag_service.py`) that can find relevant book passages for any query, with optional filtering.
*   **Validation**: The `retrieve_relevant_context` function, when tested with example queries (from the book's domain) and optional `selected_text_hint`s, returns relevant text chunks and accurate metadata.

### Milestone 4: Backend Integration & Enhanced Chat Endpoint

*   **Objective**: Modify the Phase 2 FastAPI backend to use the RAG service.
*   **Key Tasks**:
    1.  Integrate the `rag_service` module into the existing FastAPI backend's dependency injection or service layer.
    2.  Refactor the `POST /chat` endpoint logic:
        *   Extract `message`, `session_id`, and `selected_text` from the request.
        *   Call `rag_service.retrieve_relevant_context` with the user's query and `selected_text_hint`.
        *   Concatenate retrieved text passages.
        *   Construct a new, strict system prompt using the retrieved context.
        *   Pass this new prompt and the conversation history to the OpenAI chat completion call.
        *   Update the Pydantic response model for `/chat` to include an optional `sources: List[str]` field, populating it with chapter/section identifiers from retrieved metadata.
    3.  Create a new `POST /ingest` (admin) endpoint that securely triggers the `sync_book_to_qdrant.py` tool. This could involve running the script in a subprocess or exposing its logic via an internal function. Implement basic authentication for this endpoint.
    4.  Write integration tests for the modified `/chat` endpoint to verify end-to-end RAG functionality (mocking Qdrant and OpenAI API responses for controlled testing).
*   **Dependencies**: Milestone 3 must be complete.
*   **Deliverables**: The live `/chat` endpoint now provides answers grounded in the textbook via RAG, and an administrative `/ingest` endpoint is available.
*   **Validation**:
    *   Queries about specific book topics through the `/chat` endpoint return accurate answers that directly reference book content (via the context provided to OpenAI).
    *   Answers to queries unrelated to the book state that the information is not in the book.
    *   The `selected_text` feature correctly influences the RAG output.
    *   The `/ingest` endpoint successfully triggers the content synchronization.

### Milestone 5: System Integration, Testing & Final Demo Prep

*   **Objective**: Ensure the full RAG system works reliably and is ready for the hackathon demo.
*   **Key Tasks**:
    1.  **Full Integration Test**:
        *   Update a sample book content Markdown file.
        *   Run the `sync_book_to_qdrant.py` tool (via `/ingest` endpoint or directly).
        *   Query via the Docusaurus chat UI (frontend).
        *   Verify that the chatbot's response is updated and grounded in the *newly updated* content.
    2.  Test the `selected_text` bonus feature thoroughly from the frontend, ensuring highlighting text appropriately narrows and influences search results, leading to more focused answers.
    3.  Perform load testing on the `/chat` endpoint to ensure acceptable latency (considering embedding + Qdrant search + OpenAI completion times) under anticipated load.
    4.  Add comprehensive monitoring/logging for key RAG steps: OpenAI embedding calls, Qdrant search results count, and latency at various stages.
    5.  Prepare final documentation: Update the main `README.md` with an explanation of the RAG architecture, instructions for running the sync tool, and details on configuring Qdrant/OpenAI.
    6.  Verify all Acceptance Criteria from the Phase 5 `spec.md` are met.
*   **Dependencies**: All previous milestones must be complete.
*   **Deliverables**: A complete, robust, and documented RAG system powering the textbook's chatbot, ready for demonstration.
*   **Validation**:
    *   The entire system works end-to-end, from content update to grounded chatbot response.
    *   The chatbot's answers are demonstrably more accurate and book-specific.
    *   The project is stable, performant, and well-documented for submission.
    *   All acceptance criteria from `spec.md` are explicitly checked and passed.