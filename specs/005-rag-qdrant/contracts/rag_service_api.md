# API Contract: RAG Service and Chat Endpoint Enhancements

This document specifies the API contracts for the new RAG service module and the modifications to the existing chat endpoint within the FastAPI backend.

## 1. `rag_service.py` - Core Retrieval Logic

The `rag_service.py` module, located at `backend/src/services/rag_service.py`, will expose the following primary asynchronous function:

### Function: `retrieve_relevant_context`

```python
from typing import List, Dict, Optional

async def retrieve_relevant_context(
    query: str,
    limit: int = 3,
    selected_text_hint: Optional[str] = None
) -> List[Dict]:
    """
    Retrieves relevant text passages from the Qdrant collection based on a user query.

    The function first generates an embedding for the input `query`. It then performs
    a similarity search on the 'physical_ai_textbook' Qdrant collection. If
    `selected_text_hint` is provided, it attempts to infer context (source, chapter,
    section) from the hint and applies Qdrant filters to prioritize search results
    from within that specific context.

    Args:
        query: The user's question or search query. This will be embedded and used
               to find semantically similar chunks in the vector database.
        limit: The maximum number of relevant text passages to retrieve. Defaults to 3.
               The top `limit` passages by similarity score will be returned.
        selected_text_hint: Optional. A string representing text selected by the user
                            in the UI. This hint is used to bias the Qdrant search
                            towards relevant sections of the textbook (e.g., the
                            chapter or section from which the text was selected).
                            If no context can be inferred, or no results are found
                            within the filtered context, a broader search may be
                            performed as a fallback.

    Returns:
        A list of dictionaries, where each dictionary represents a retrieved text chunk
        and contains the following keys:
        - "text": `str` - The original text content of the retrieved chunk.
        - "source": `str` - The file path of the original Markdown document (e.g., `docs/module1/chapter-1.md`).
        - "chapter": `str` - The title or identifier of the chapter the chunk belongs to.
        - "section": `str` - The heading or subsection title the chunk belongs to.

    Raises:
        OpenAIAPIError: If there's an issue with the OpenAI Embedding API.
        QdrantOperationError: If there's an issue communicating with Qdrant.
    """
    # Implementation will include:
    # 1. Embedding generation for 'query' using OpenAI API.
    # 2. Parsing 'selected_text_hint' to derive Qdrant filter conditions (e.g., `FieldCondition`).
    # 3. Performing `qdrant_client.query_points` (or similar) with vector search and filters.
    # 4. Extracting and formatting payload data from retrieved points.
    pass
```

---

## 2. FastAPI Endpoints

### 2.1 `POST /chat` (Enhanced Endpoint)

The existing `POST /chat` endpoint will be modified to integrate the RAG pipeline.

*   **HTTP Method**: `POST`
*   **Path**: `/chat`
*   **Description**: Processes user chat messages, now enhanced with Retrieval-Augmented Generation (RAG) to provide answers grounded in the "Physical AI & Humanoid Robotics" textbook.

#### Request Body (Pydantic Model)

```python
# Assuming an existing ChatRequest model, modified to include selected_text
from pydantic import BaseModel, Field
from typing import Optional

class ChatRequest(BaseModel):
    message: str = Field(..., description="The user's chat message or question.")
    session_id: str = Field(..., description="Unique identifier for the chat session.")
    selected_text: Optional[str] = Field(
        None,
        description="Optional: Text selected by the user in the UI, used as a hint for RAG retrieval."
    )
```

#### Response Body (Pydantic Model)

```python
# Assuming an existing ChatResponse model, modified to include sources
from pydantic import BaseModel, Field
from typing import List, Optional

class ChatResponse(BaseModel):
    response: str = Field(..., description="The AI's response to the user's message, grounded in textbook content.")
    session_id: str = Field(..., description="The unique identifier for the chat session.")
    sources: Optional[List[str]] = Field(
        None,
        description="Optional: A list of identifiers (e.g., 'Chapter 1', 'Module 2, Section 3') indicating "
                    "where the answer's context was retrieved from in the textbook. Only present if RAG was used."
    )
```

#### Behavior Summary

1.  Receives `message`, `session_id`, and `selected_text`.
2.  Calls `rag_service.retrieve_relevant_context` using `message` as `query` and `selected_text` as `selected_text_hint`.
3.  Constructs a dynamic, strict system prompt for the OpenAI chat completion API, incorporating the retrieved context.
4.  Sends the constructed prompt along with the conversation history to OpenAI.
5.  Returns the AI's response and, if applicable, a list of `sources` derived from the retrieved context metadata.

---

### 2.2 `POST /ingest` (New Administrative Endpoint)

This endpoint provides a secure way to trigger the book content synchronization process.

*   **HTTP Method**: `POST`
*   **Path**: `/ingest`
*   **Description**: Triggers the `sync_book_to_qdrant.py` tool to process and ingest book content into the Qdrant vector database. This is an administrative function and requires authentication.

#### Authentication

*   **Mechanism**: A custom header, e.g., `X-Admin-Key`, containing a pre-shared secret or an authenticated user token.
    *(Details on the specific authentication mechanism will be covered in implementation, but a basic API key check will be used initially).*

#### Request Body (Pydantic Model)

```python
from pydantic import BaseModel, Field
from typing import Optional

class IngestRequest(BaseModel):
    force_resync: bool = Field(
        False,
        description="If true, clears the existing Qdrant collection and re-ingests all content from scratch."
    )
```

#### Response Body (Pydantic Model)

```python
from pydantic import BaseModel, Field

class IngestResponse(BaseModel):
    status: str = Field(..., description="Status of the ingestion process (e.g., 'success', 'failed', 'started').")
    message: str = Field(..., description="A descriptive message about the ingestion operation.")
    ingested_count: Optional[int] = Field(
        None,
        description="The number of documents/chunks successfully ingested, if applicable."
    )
```

#### Behavior Summary

1.  Authenticates the incoming request.
2.  Parses the `force_resync` flag.
3.  Initiates the book content synchronization by calling the `sync_book_to_qdrant.py` tool (or its underlying logic). This operation might be asynchronous for large datasets.
4.  Returns a status message indicating whether the ingestion process has started successfully or encountered an error.
