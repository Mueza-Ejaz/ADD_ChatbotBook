# FastAPI Chat Backend

A minimalistic FastAPI backend for a chat application, featuring integration with Google Gemini API, PostgreSQL database with SQLAlchemy and Alembic for migrations.

## Features

*   **Chat Sessions**: Manage persistent chat sessions.
*   **Message Storage**: Store user and AI messages in a PostgreSQL database.
*   **Google Gemini API Integration**: Generate AI responses using Google's Gemini-2.5-Flash model.
*   **Context Awareness**: Maintain conversation context for AI responses.
*   **Selected Text Functionality**: Prioritize AI responses based on user-selected text.
*   **Asynchronous Database Operations**: Efficient database interactions using `asyncpg` and `SQLAlchemy` async mode.
*   **Database Migrations**: Manage schema changes with `Alembic`.
*   **CORS Configuration**: Securely handle cross-origin requests.
*   **Structured Logging**: Detailed logging with `Loguru`.
*   **Pydantic Validation**: Robust input and output validation.
*   **Comprehensive Error Handling**: Graceful error management for API, database, and validation issues.
*   **Unit and Integration Tests**: Ensure reliability and correctness.
*   **Code Quality Tools**: Integrated `Ruff` for linting and `Black` for formatting.

## Project Structure

```
.
├── alembic/                      # Alembic migration scripts
├── src/
│   ├── config/                   # Configuration files (e.g., logging)
│   ├── database.py               # Database engine and session management
│   ├── exceptions.py             # Custom exception classes
│   ├── main.py                   # FastAPI application main file, API endpoints
│   ├── models.py                 # SQLAlchemy ORM models
│   ├── repositories/             # Database repositories (e.g., ChatRepository)
│   ├── schemas.py                # Pydantic models for request/response validation
│   └── services/                 # Business logic and external service integrations
│       ├── message_repository.py # Service for message-related operations
│       ├── openai_client.py      # Client for Google Gemini API interaction (named openai_client for historical reasons)
│       └── session_manager.py    # Service for chat session management
└── tests/                        # Unit and integration tests
    ├── integration/              # Integration tests
    └── unit/                     # Unit tests
```

## Setup Instructions

### Prerequisites

*   Python 3.12+
*   Poetry (for dependency management)
*   PostgreSQL database (e.g., [Neon DB](https://neon.tech/))
*   Google Gemini API Key

### 1. Clone the repository

```bash
git clone <repository_url>
cd <repository_name>/backend
```

### 2. Install Dependencies

Using Poetry:

```bash
poetry install
```

### 3. Environment Variables

Create a `.env` file in the `backend` directory based on `.env.example`:

```bash
cp .env.example .env
```

Edit the `.env` file and fill in your details:

```ini
DATABASE_URL="postgresql+asyncpg://YOUR_DB_USER:YOUR_DB_PASSWORD@YOUR_DB_HOST:YOUR_DB_PORT/YOUR_DB_NAME?sslmode=require"
GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
FRONTEND_URL="http://localhost:3000" # Or your frontend application's URL
```

**Note**: Ensure your `DATABASE_URL` is a valid `asyncpg` connection string. If using Neon, it should look something like:
`postgresql+asyncpg://YOUR_USERNAME:YOUR_PASSWORD@ep-YOUR_PROJECT_ID.us-east-1.aws.neon.tech/YOUR_DATABASE_NAME?sslmode=require`

### 4. Run Database Migrations

Apply the database schema:

```bash
poetry run alembic upgrade head
```

## Running the Application Locally

To start the FastAPI application:

```bash
poetry run uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

The application will be accessible at `http://localhost:8000`. The API documentation (Swagger UI) will be available at `http://localhost:8000/docs`.

## Basic API Usage

### Health Check

*   **Endpoint**: `GET /health`
*   **Description**: Checks if the API is running.
*   **Response**:
    ```json
    {
        "status": "healthy"
    }
    ```

### Chat Endpoint

*   **Endpoint**: `POST /chat`
*   **Description**: Sends a message to the AI and manages the chat session.
*   **Request Body**:
    ```json
    {
        "message": "Hello, how are you?",
        "session_id": "optional-uuid-of-existing-session",
        "selected_text": "optional-text-for-context"
    }
    ```
*   **Response**:
    ```json
    {
        "response": "I'm doing well, thank you for asking!",
        "session_id": "new-or-existing-uuid",
        "timestamp": "2023-10-27T10:00:00.123456+00:00"
    }
    ```

## Running Tests

To run the unit and integration tests:

```bash
poetry run pytest
```

## Code Quality

This project uses `Ruff` for linting and `Black` for code formatting.

To run the linter and formatter:

```bash
poetry run ruff check .
poetry run black .
```

To automatically fix linting errors (where possible):

```bash
poetry run ruff check . --fix
```
