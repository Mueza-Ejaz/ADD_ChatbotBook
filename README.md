# AI-Powered Chatbot with RAG

This project implements an AI-powered chatbot that leverages Retrieval-Augmented Generation (RAG) to provide grounded answers based on a knowledge base. The knowledge base is built from Markdown documents, which are chunked, embedded using Gemini's text embedding model, and stored in Qdrant, a vector database.

The project consists of two main components:
-   **Backend (FastAPI)**: Handles chat interactions, integrates with the RAG service, and provides an ingestion endpoint for updating the knowledge base.
-   **Frontend (Docusaurus)**: A documentation website that can potentially integrate the chatbot UI.

## RAG Architecture Overview

The RAG system enhances the chatbot's ability to provide accurate and relevant responses by incorporating external knowledge.

1.  **Data Ingestion**: Markdown documents (e.g., from `frontend/docs/`) are processed by a synchronization tool.
    *   **Text Chunking**: Documents are split into smaller, coherent chunks, respecting their structural integrity (e.g., headings, paragraphs).
    *   **Embedding Generation**: Each text chunk is converted into a high-dimensional vector (embedding) using the Gemini text embedding model (`models/text-embedding-004`).
    *   **Vector Storage**: These embeddings, along with their original text and metadata (source, chapter, section), are stored in a Qdrant collection.
2.  **Chat Interaction**: When a user submits a query to the chatbot:
    *   **Query Embedding**: The user's query is also converted into an embedding using the same Gemini model.
    *   **Context Retrieval**: The query embedding is used to search the Qdrant database for the most semantically similar text chunks.
    *   **Prompt Augmentation**: The retrieved text chunks (relevant context) are concatenated and prepended to the user's original query. A strict system prompt is also constructed to instruct the Large Language Model (LLM) to answer questions based *only* on the provided context.
    *   **Response Generation**: The augmented prompt, along with conversation history, is sent to the Gemini API for response generation. The LLM then generates a grounded answer based on the provided context.
    *   **Source Attribution**: The sources of the retrieved context are also returned, allowing the chatbot to provide citations for its answers.

## Sync Tool Instructions (`tools/sync_book_to_qdrant.py`)

This tool is responsible for building and updating the knowledge base in Qdrant.

### Configuration

Ensure the following environment variables are set in your `backend/.env` file:

-   `GEMINI_API_KEY`: Your API key for accessing the Gemini API (for embeddings).
-   `QDRANT_URL`: The URL of your Qdrant Cloud cluster or local instance.
-   `QDRANT_API_KEY`: Your API key for authenticating with Qdrant.

Example `.env` entries:
```dotenv
GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
QDRANT_URL="YOUR_QDRANT_URL"
QDRANT_API_KEY="YOUR_QDRANT_API_KEY"
```

### Usage

To run the synchronization tool:

```bash
python tools/sync_book_to_qdrant.py --docs-path frontend/docs/ --collection-name physical_ai_textbook
```

**Arguments**:
-   `--docs-path`: Path to the directory containing your Markdown files (e.g., `frontend/docs/`).
-   `--collection-name`: The name of the Qdrant collection to use (default: `physical_ai_textbook`).
-   `--force-resync` (optional): If present, the existing Qdrant collection will be deleted and recreated before syncing. Use this with caution, as it will clear all existing data in the collection.

### Ingestion via API Endpoint

The backend provides an `/ingest` endpoint to trigger the synchronization process remotely. This endpoint requires authentication.

**Endpoint**: `POST /ingest`
**Headers**: `X-API-Key: YOUR_INGESTION_API_KEY` (set `INGESTION_API_KEY` in `backend/.env`)
**Query Parameters**:
-   `force_resync`: `true` or `false` (optional, default `false`).

## Qdrant Configuration

-   **Collection Name**: `physical_ai_textbook` (configurable via sync tool and backend service).
-   **Vector Size**: `768` (corresponds to `models/text-embedding-004`).
-   **Distance Metric**: `Cosine` (for similarity search).

## Gemini/LLM Configuration

-   **Embedding Model**: `models/text-embedding-004` (for RAG context retrieval).
-   **Chat Model**: The specific chat model used by `src/services/openai_client.py` (which is named GeminiClient but currently interacts with OpenAI APIs - this is an area for potential future refinement to truly use Gemini chat models if desired).
    *Note*: The `openai_client.py` name is a legacy from initial setup and needs to be updated to `gemini_client.py` for consistency. The code inside `openai_client.py` is already configured to use the Gemini API.

## Project Setup and Development

Refer to the `backend/README.md` and `frontend/README.md` for specific setup and development instructions for each component.
