# Backend for RAG with Qdrant

This directory contains the FastAPI backend for the Retrieval-Augmented Generation (RAG) system, integrating with Qdrant Cloud for vector search and OpenAI for embeddings and chat completions.

## Setup

1.  Ensure you have Python 3.9+ and Poetry installed.
2.  Create a `.env` file in this directory with your OpenAI and Qdrant API keys and Qdrant URL.
    ```
    OPENAI_API_KEY="your_openai_api_key"
    QDRANT_API_KEY="your_qdrant_api_key"
    QDRANT_URL="your_qdrant_url"
    ```
3.  Install dependencies:
    ```bash
    poetry install
    ```

## Running the Application

```bash
poetry run uvicorn src.main:app --reload --port 8000
```

## API Endpoints

*   `POST /chat`: Enhanced chat endpoint with RAG capabilities.
*   `POST /ingest`: (Admin) Endpoint to trigger book content synchronization to Qdrant.

## Development

See `specs/005-rag-qdrant/` for detailed design specifications and tasks.