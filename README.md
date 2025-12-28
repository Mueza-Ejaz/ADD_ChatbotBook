# Physical AI & Humanoid Robotics Textbook

This project serves as the companion website and backend API for the "Physical AI & Humanoid Robotics" textbook. It provides a rich, interactive learning experience combining static content (book chapters, tutorials) with a powerful, AI-driven chatbot capable of answering questions based on the book's content using Retrieval-Augmented Generation (RAG).

## Features

### Frontend (Docusaurus)
*   **Static Site Generation**: Fast, SEO-friendly documentation and blog.
*   **Book Chapters & Tutorials**: Organized content for easy navigation.
*   **Interactive Chatbot UI**: User interface to interact with the AI assistant.
*   **Selected Text Integration**: Ability to select text from the book to provide context to the chatbot for more accurate answers.

### Backend (FastAPI with RAG)
*   **Chat Sessions**: Manage persistent chat sessions with a PostgreSQL database.
*   **Google Gemini API Integration**: Generate AI responses using Google's Gemini models.
*   **Retrieval-Augmented Generation (RAG)**: Enhance AI responses by retrieving relevant information from the book's content.
*   **Qdrant Vector Database**: Utilizes Qdrant for efficient storage and semantic search of document embeddings.
*   **Data Ingestion API**: Secure endpoint to synchronize book content into Qdrant.
*   **Asynchronous Operations**: Efficient non-blocking I/O for database and external API calls.
*   **Structured Logging**: Detailed and configurable logging with Loguru.
*   **Pydantic Validation**: Robust input and output validation for API endpoints.
*   **Comprehensive Error Handling**: Graceful error management.
*   **Database Migrations**: Managed with Alembic for schema changes.

## Technologies Used

### Frontend
*   **Docusaurus**: Static site generator
*   **React**: UI library
*   **TypeScript**: Type-safe JavaScript
*   **SCSS Modules**: For modular and scoped styling

### Backend
*   **FastAPI**: Web framework for building APIs
*   **Python**: Programming language
*   **Google Gemini API**: For AI model interaction and embeddings
*   **Qdrant**: Vector database for RAG
*   **SQLAlchemy (Async)**: ORM for PostgreSQL database interaction
*   **Alembic**: Database migrations
*   **Poetry**: Dependency management
*   **Loguru**: Structured logging
*   **Pydantic**: Data validation and settings management

## Project Structure

```
.
├── backend/                      # FastAPI backend application
│   ├── alembic/                  # Alembic migration scripts
│   ├── src/
│   │   ├── config/               # Configuration files (e.g., logging)
│   │   ├── database.py           # Database engine and session management
│   │   ├── exceptions.py         # Custom exception classes
│   │   ├── main.py               # FastAPI application, API endpoints
│   │   ├── models.py             # SQLAlchemy ORM models
│   │   ├── repositories/         # Database repositories
│   │   ├── schemas.py            # Pydantic models for request/response
│   │   └── services/             # Business logic and external service integrations
│   │       ├── message_repository.py
│   │       ├── openai_client.py  # Gemini API client
│   │       ├── rag_service.py    # Retrieval-Augmented Generation logic
│   │       └── session_manager.py
│   └── tests/                    # Unit and integration tests
├── frontend/                     # Docusaurus frontend application
│   ├── docs/                     # Markdown files for book chapters/tutorials
│   ├── blog/                     # Blog posts
│   ├── src/
│   │   ├── components/           # React components (e.g., Chatbot)
│   │   ├── css/                  # Custom CSS
│   │   ├── hooks/
│   │   ├── pages/                # Docusaurus pages (e.g., index.tsx, about.mdx)
│   │   └── theme/                # Docusaurus theme overrides (e.g., Layout)
│   └── docusaurus.config.ts      # Docusaurus configuration
├── tools/                        # Utility scripts (e.g., sync_book_to_qdrant.py)
└── specs/                        # Design specifications and documentation
```

## Setup Instructions

### Prerequisites
*   **Git**: For version control.
*   **Node.js & npm/yarn**: For the frontend Docusaurus application.
*   **Python 3.12+**: For the backend FastAPI application.
*   **Poetry**: Python dependency management tool (`pip install poetry`).
*   **PostgreSQL Database**: A running instance (e.g., local, Docker, Neon DB).
*   **Qdrant Instance**: A running instance (e.g., local, Docker, Qdrant Cloud).
*   **Google Gemini API Key**: For AI model interactions.

### 1. Clone the repository

```bash
git clone <repository_url>
cd <repository_name>
```

### 2. Backend Setup

Navigate to the `backend` directory:
```bash
cd backend
```

**Install Dependencies:**
```bash
poetry install
```

**Environment Variables:**
Create a `.env` file in the `backend` directory based on `.env.example` (if provided, otherwise create manually):
```bash
cp .env.example .env # if .env.example exists
```
Edit the `.env` file and fill in your details:
```ini
DATABASE_URL="postgresql+asyncpg://YOUR_DB_USER:YOUR_DB_PASSWORD@YOUR_DB_HOST:YOUR_DB_PORT/YOUR_DB_NAME?sslmode=require"
GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
QDRANT_URL="YOUR_QDRANT_URL"
QDRANT_API_KEY="YOUR_QDRANT_API_KEY"
INGESTION_API_KEY="YOUR_INGESTION_API_KEY" # Key to secure the /ingest endpoint
FRONTEND_URL="http://localhost:3000" # Or your frontend application's URL
```
**Note**: Ensure your `DATABASE_URL` is a valid `asyncpg` connection string.

**Run Database Migrations:**
```bash
poetry run alembic upgrade head
```

### 3. Frontend Setup

Navigate to the `frontend` directory:
```bash
cd ../frontend
```

**Install Dependencies:**
```bash
npm install # or yarn install
```

## Running the Application

### Running Backend Locally

Navigate to the `backend` directory:
```bash
cd backend
```
Start the FastAPI application:
```bash
poetry run uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```
The backend API will be accessible at `http://localhost:8000`. API documentation (Swagger UI) at `http://localhost:8000/docs`.

### Running Frontend Locally

Navigate to the `frontend` directory:
```bash
cd frontend
```
Start the Docusaurus development server:
```bash
npm start # or yarn start
```
The frontend will be accessible at `http://localhost:3000`.

## API Endpoints (Backend)

*   **`GET /health`**: Checks API health.
*   **`POST /chat`**: Handles chat interactions with the RAG system.
*   **`POST /ingest`**: (Admin) Triggers synchronization of book content to Qdrant. Requires `X-API-Key` header for authentication.

## Data Ingestion

To populate the Qdrant database with your book content:

1.  Ensure your backend is running.
2.  Send a POST request to the `/ingest` endpoint:
    ```bash
    curl -X POST http://localhost:8000/ingest -H "X-API-Key: YOUR_INGESTION_API_KEY"
    ```
    Optionally, add `?force_resync=true` to force a full re-ingestion.

## Testing

### Backend Tests

Navigate to the `backend` directory:
```bash
cd backend
```
Run unit and integration tests:
```bash
poetry run pytest
```

## Code Quality

This project uses `Ruff` for linting and `Black` for code formatting (Backend).

To run linter and formatter (backend):
```bash
cd backend
poetry run ruff check .
poetry run black .
```

To automatically fix linting errors (backend):
```bash
cd backend
poetry run ruff check . --fix
```

- **Live Website**: ""







