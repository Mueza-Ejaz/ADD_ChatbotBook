# Quickstart Guide: Phase 5 - RAG with Qdrant

This guide provides a quick overview of how to set up, populate, and interact with the Retrieval-Augmented Generation (RAG) system for the "Physical AI & Humanoid Robotics" textbook.

## 1. Prerequisites

Before you begin, ensure you have:

*   **Python 3.9+** installed.
*   **Poetry** for Python dependency management (for the backend).
*   **Node.js and npm/yarn** (for Docusaurus frontend, if running locally).
*   **OpenAI API Key**: For generating embeddings and chat completions.
*   **Qdrant Cloud Account & API Key**: A free-tier cluster is sufficient for development.

## 2. Environment Setup

1.  **Clone the Repository**:
    ```bash
    git clone <repository_url>
    cd ADD_ChatbotBook
    ```

2.  **Backend Setup**:
    Navigate to the `backend/` directory.
    ```bash
    cd backend
    poetry install
    ```
    Create a `.env` file in the `backend/` directory with your API keys:
    ```
    OPENAI_API_KEY="your_openai_api_key_here"
    QDRANT_API_KEY="your_qdrant_cloud_api_key_here"
    QDRANT_URL="your_qdrant_cloud_url_here" # e.g., "https://<cluster_id>.qdrant.tech"
    ```

## 3. Qdrant Collection Creation

The Qdrant collection named `physical_ai_textbook` needs to be created. This can be done programmatically via the `qdrant-client` (as part of Milestone 1 test script) or manually through the Qdrant Cloud dashboard.
Ensure the collection has:
*   **Name**: `physical_ai_textbook`
*   **Vector Size**: `1536`
*   **Distance Metric**: `Cosine`

## 4. Syncing Book Content to Qdrant (Ingestion)

A standalone tool is used to process the Docusaurus book content and upload it to Qdrant.

1.  **Install dependencies (if running standalone)**:
    Ensure `qdrant-client` and `openai` are installed in your environment if running the sync tool outside the backend's poetry environment.
    ```bash
    # From project root if not using backend's poetry env
    pip install qdrant-client openai markdown-it-py python-dotenv
    ```
    *(Note: If running within the backend's poetry environment, these should already be installed.)*

2.  **Run the Sync Tool**:
    ```bash
    # From project root
    python tools/sync_book_to_qdrant.py --docs-path frontend/docs/
    ```
    This will chunk, embed, and upload all Markdown files from `frontend/docs/` to your Qdrant Cloud instance.

    Alternatively, if the administrative `/ingest` endpoint is implemented and the backend is running:
    ```bash
    curl -X POST http://localhost:8000/ingest -H "Content-Type: application/json" -H "X-Admin-Key: your_admin_key" -d '{}'
    ```

## 5. Running the Backend

1.  **Start the FastAPI Backend**:
    Navigate to the `backend/` directory.
    ```bash
    cd backend
    poetry run uvicorn src.main:app --reload --port 8000
    ```
    The RAG-enabled chat endpoint will now be available at `http://localhost:8000/chat`.

## 6. Interacting with the Chatbot (RAG Enabled)

You can interact with the RAG-enabled chatbot via the modified `POST /chat` endpoint.

**Example `curl` request:**

```bash
curl -X POST http://localhost:8000/chat \
-H "Content-Type: application/json" \
-d 
'{'
  "message": "What are the key differences between soft and hard robotics?",
  "session_id": "test_session_1",
  "selected_text": null
}'
```

**With `selected_text` hint:**

To bias the search towards a specific context (e.g., from a user highlighting text in the UI):

```bash
curl -X POST http://localhost:8000/chat \
-H "Content-Type: application/json" \
-d 
'{'
  "message": "Can you explain proprioception in humanoid robots?",
  "session_id": "test_session_2",
  "selected_text": "Proprioception is the sense of the relative position of neighbouring parts of the body and strength of effort being employed in movement."
}'
```

Expected responses will be grounded in the book content stored in Qdrant. If the answer is not found, the chatbot will indicate so.
