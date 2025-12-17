# Feature Specification: Minimalistic FastAPI Backend for Chat

**Feature Branch**: `002-fastapi-chat-backend`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "Create a detailed technical specification for \"Phase 2: Minimalistic FastAPI Backend\". Core Objective: Build a FastAPI backend with a single /chat endpoint that uses the OpenAI SDK to process messages. It must store conversation context in a Neon PostgreSQL database and be built for integration with a ChatKit frontend (Phase 3) and future RAG expansion (Phase 5). 1. Technical Stack: The framework is FastAPI (async), the language is Python 3.12+, the AI SDK is the official OpenAI Python library (openai package), the database is Neon (Serverless PostgreSQL) with async SQLAlchemy 2.0 and Alembic, validation is handled with Pydantic v2, and package management should use uv or poetry. 2. Primary Endpoint - POST /chat: The request body includes a JSON object with fields \"message\": \"string\", \"session_id\": \"string | null\", and \"selected_text\": \"string | null\", where the selected_text field is critical for a hackathon bonus feature in which answers must be based solely on user-selected text from the book. The response body includes a JSON object with \"response\": \"string\", \"session_id\": \"string\", and \"timestamp\": \"string\". 3. Core Logic Flow: The system must validate input with Pydantic, find or create a chat_session in the database using session_id, save the user message to the messages table, construct a system prompt framing the AI as a helpful assistant for a \"Physical AI & Humanoid Robotics\" textbook, call the OpenAI Chat Completions API while sending the conversation history consisting of approximately the last ten messages from the session, save the AI’s response to the messages table, and return the formatted JSON response. 4. Database Schema (Neon PostgreSQL): The chat_sessions table includes id (UUID, Primary Key), created_at (Timestamp), and updated_at (Timestamp). The messages table includes id (UUID, Primary Key), session_id (UUID, Foreign Key to chat_sessions.id), role (Enum: \'user\', \'assistant\', \'system\'), content (Text), and created_at (Timestamp). 5. Mandatory Features: All secrets such as the OpenAI API Key and Database URL must be loaded from environment variables using a .env file, comprehensive error handling must be implemented for API errors, database errors, and validation errors with appropriate HTTP status codes, CORS must be configured to allow requests from a Docusaurus frontend running on localhost:3000 or a deployment URL, a GET /health endpoint must be provided to return service status, and structured logging must be implemented for incoming requests and API calls. 6. Non-Functional & Future-Proofing: The target performance is a response time under two seconds for the /chat endpoint, code quality requirements include the use of type hints, docstrings, and a pyproject.toml, and the codebase must be prepared for Phase 5 (RAG) by allowing easy insertion of a retrieval step before the OpenAI API call and ensuring the database schema is extensible for storing document chunks and embeddings. Specification Document Structure: Generate a complete spec.md file that includes the following sections: an Overview describing the phase goal and context, a System Architecture section with a text description of data flow, an API Specification detailing OpenAPI-style definitions for /chat and /health, Data Models covering Pydantic models for requests and responses and SQLAlchemy models for the database, Configuration and Environment Variables, an Error Handling and Logging Strategy, a Testing Outline describing key unit and integration tests to write, Deployment Notes explaining how to run locally and prepare for Vercel deployment, and Acceptance Criteria presented as a checklist to mark Phase 2 as complete.

## Overview

This specification details "Phase 2: Minimalistic FastAPI Backend" for the Physical AI & Humanoid Robotics textbook chatbot project. The core objective is to develop a robust, scalable, and extensible FastAPI backend service. This backend will feature a primary `/chat` endpoint responsible for processing user messages, interacting with the OpenAI SDK to generate responses, and persistently storing conversation context in a Neon PostgreSQL database. This phase lays the foundation for integration with a ChatKit frontend (Phase 3) and future RAG (Retrieval-Augmented Generation) capabilities (Phase 5).

## System Architecture

The system comprises a FastAPI application serving as the backend. Upon receiving a request at the `/chat` endpoint, the application will:
1. Validate the incoming request data using Pydantic.
2. Interact with a Neon PostgreSQL database to retrieve or create a `chat_session` and store user messages.
3. Construct a system prompt to guide the AI's persona as a helpful assistant for the "Physical AI & Humanoid Robotics" textbook.
4. Utilize the OpenAI Python SDK to communicate with the OpenAI Chat Completions API, sending a concise history of the conversation (approximately the last ten messages) to maintain context.
5. Save the AI's generated response to the database.
6. Return a formatted JSON response to the client. A separate `/health` endpoint will provide service status. Environment variables will manage sensitive configurations.

## API Specification

### POST /chat

Processes user messages, interacts with the OpenAI API, and manages conversation history.

-   **Request Body**:
    ```json
    {
      "message": "string",
      "session_id": "string | null",
      "selected_text": "string | null"
    }
    ```
    -   `message`: The user's input message.
    -   `session_id`: Optional. A unique identifier for the chat session. If not provided, a new session is initiated.
    -   `selected_text`: Optional. Text selected by the user, used for bonus feature (answers based solely on selected text).

-   **Response Body**:
    ```json
    {
      "response": "string",
      "session_id": "string",
      "timestamp": "string"
    }
    ```
    -   `response`: The AI's generated response.
    -   `session_id`: The identifier for the current chat session.
    -   `timestamp`: ISO 8601 formatted timestamp of the AI's response.

-   **Status Codes**:
    -   `200 OK`: Successful response.
    -   `400 Bad Request`: Invalid input (e.g., Pydantic validation error).
    -   `500 Internal Server Error`: Generic server error (e.g., OpenAI API error, database error).

### GET /health

Returns the service status.

-   **Response Body**:
    ```json
    {
      "status": "string"
    }
    ```
    -   `status`: Indicates the health of the service, e.g., "healthy".

-   **Status Codes**:
    -   `200 OK`: Service is healthy.

## Data Models

### Pydantic Models (for API Request/Response)

```python
from pydantic import BaseModel
from typing import Optional

class ChatRequest(BaseModel):
    message: str
    session_id: Optional[str] = None
    selected_text: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    session_id: str
    timestamp: str # ISO 8601 format

class HealthResponse(BaseModel):
    status: str
```

### SQLAlchemy Models (for Neon PostgreSQL Database)

```python
import uuid
from datetime import datetime
from sqlalchemy import Column, String, Text, DateTime, ForeignKey, Enum
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class ChatSession(Base):
    __tablename__ = "chat_sessions"
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    messages = relationship("Message", back_populates="session")

class Message(Base):
    __tablename__ = "messages"
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey("chat_sessions.id"))
    role = Column(Enum("user", "assistant", "system", name="message_role_enum"))
    content = Column(Text)
    created_at = Column(DateTime, default=datetime.utcnow)
    session = relationship("ChatSession", back_populates="messages")
```

## Configuration and Environment Variables

All sensitive information and configurable parameters will be managed via environment variables.

-   `.env` file: Used for local development to load environment variables.
-   Mandatory variables:
    -   `OPENAI_API_KEY`: API key for accessing OpenAI services.
    -   `DATABASE_URL`: Connection string for the Neon PostgreSQL database.
-   CORS Configuration:
    -   `CORS_ORIGINS`: Comma-separated list of allowed origins (e.g., `http://localhost:3000,https://your-frontend-deployment.com`).

## Error Handling and Logging Strategy

### Error Handling

Comprehensive error handling will be implemented for all potential failure points:

-   **Pydantic Validation Errors**: Return `400 Bad Request` with detailed error messages.
-   **OpenAI API Errors**: Catch exceptions from the OpenAI SDK and return `500 Internal Server Error`, obscuring sensitive details from the client but logging them internally.
-   **Database Errors**: Catch SQLAlchemy/PostgreSQL exceptions and return `500 Internal Server Error`, with internal logging.
-   **Generic Exceptions**: Catch unhandled exceptions and return a `500 Internal Server Error`.
-   All error responses will adhere to a consistent JSON format including an error code and a user-friendly message.

### Logging

Structured logging will be implemented using a standard Python logging library (e.g., `logging` or `Loguru`) to ensure clear, searchable logs.

-   **Log Format**: JSON format for easy parsing and analysis by logging systems.
-   **Information logged**:
    -   Incoming request details (endpoint, method, headers, basic request body info).
    -   OpenAI API call details (request parameters, response status, duration).
    -   Database interaction details (query success/failure, duration).
    -   Error details (exception type, message, stack trace).
    -   Session IDs for tracing requests across interactions.

## User Scenarios & Testing

### User Story 1 - Initiate New Chat Session (Priority: P1)

A user accesses the chat interface for the first time or wishes to start a new conversation.

**Why this priority**: This is the fundamental entry point for any user interaction with the chatbot and establishes the core functionality. Without it, no other chat features can be utilized.

**Independent Test**: Can be fully tested by sending a chat message without a `session_id` and verifying that a new session is created, the AI responds, and the interaction is saved.

**Acceptance Scenarios**:

1.  **Given** a user sends a message to the `/chat` endpoint **When** no `session_id` is provided in the request **Then** a new `chat_session` is created in the database.
2.  **Given** a new `chat_session` is created **When** the AI processes the user's message **Then** the user's message and the AI's response are saved in the `messages` table linked to the new `session_id`.
3.  **Given** the AI generates a response for a new session **When** the response is returned to the user **Then** the response JSON includes the new `session_id` and the AI's response.

### User Story 2 - Continue Existing Chat Session (Priority: P1)

A user continues a conversation with the chatbot, expecting context to be maintained.

**Why this priority**: Maintaining conversation context is critical for a coherent and helpful chatbot experience. This directly impacts user satisfaction and the utility of the system.

**Independent Test**: Can be fully tested by sending multiple messages with the same `session_id` and verifying that the AI's responses are contextually relevant, and all messages are correctly linked in the database.

**Acceptance Scenarios**:

1.  **Given** an active `chat_session` exists **When** a user sends a message to the `/chat` endpoint with an existing `session_id` **Then** the user's message is saved and linked to the existing `chat_session`.
2.  **Given** an active `chat_session` with previous messages **When** the AI processes a new message for that session **Then** the AI's response considers the context of approximately the last ten messages from that session.
3.  **Given** the AI generates a response for an existing session **When** the response is returned to the user **Then** the response JSON includes the provided `session_id` and the AI's response.

### User Story 3 - Get Answer from Selected Text (Priority: P2)

A user provides specific text from the book and expects the AI's response to be based *solely* on that text, ignoring previous conversation context.

**Why this priority**: This is a key "hackathon bonus feature" providing enhanced utility by grounding AI responses directly in user-specified content, which is valuable for an educational context.

**Independent Test**: Can be fully tested by sending a message with both `message` and `selected_text` fields populated and verifying the AI's response directly addresses the `selected_text`, overriding general context.

**Acceptance Scenarios**:

1.  **Given** a user sends a message to the `/chat` endpoint **When** both `message` and `selected_text` are provided **Then** the backend prioritizes the `selected_text` to inform the AI's response.
2.  **Given** `selected_text` is provided **When** the AI generates a response **Then** the AI's response is based *solely* on the content of `selected_text` and the user's `message`, effectively ignoring broader conversation history for that specific turn.

## Testing Outline

### Unit Tests

-   **Pydantic Models**: Verify correct validation and serialization/deserialization of `ChatRequest`, `ChatResponse`, and `HealthResponse`.
-   **Database Models**: Test SQLAlchemy model definitions, relationships, and basic CRUD operations in isolation (e.g., using an in-memory SQLite database for structure, or a test PostgreSQL instance).
-   **Core Logic Components**: Test individual functions responsible for constructing the system prompt, managing conversation history (e.g., retrieving the last ten messages), and formatting API responses.

### Integration Tests

-   **`/chat` Endpoint**:
    -   Test successful end-to-end flow: valid request -> database interaction -> OpenAI API call -> database update -> successful response.
    -   Test error scenarios: invalid request body, OpenAI API failure, database connection errors.
    -   Verify conversation context persistence across multiple calls with the same `session_id`.
    -   Test the `selected_text` bonus feature, ensuring responses are grounded in provided text.
-   **`/health` Endpoint**: Verify it consistently returns a `200 OK` status.
-   **Environment Variable Loading**: Ensure all mandatory environment variables are correctly loaded and used.
-   **CORS Configuration**: Verify that requests from allowed origins are accepted and requests from disallowed origins are rejected.

## Deployment Notes

### Local Development Setup

1.  **Clone the repository**: `git clone [repository-url]`
2.  **Navigate to the backend directory**: `cd [backend-directory]`
3.  **Install dependencies**:
    -   Using `uv`: `uv pip install -r requirements.txt` (assuming `requirements.txt` is generated)
    -   Using `poetry`: `poetry install` (if `pyproject.toml` is used with poetry)
4.  **Create a `.env` file**: Populate with `OPENAI_API_KEY`, `DATABASE_URL`, and `CORS_ORIGINS`.
5.  **Run Alembic migrations**: `alembic upgrade head` (after initial setup of Alembic)
6.  **Start the FastAPI application**: `uvicorn main:app --reload` (assuming `main.py` is the entry point)

### Vercel Deployment Preparation

-   **`vercel.json`**: Configure build settings and routes for serverless functions.
-   **Environment Variables**: Securely configure `OPENAI_API_KEY`, `DATABASE_URL`, and `CORS_ORIGINS` in Vercel project settings.
-   **Database Migrations**: Alembic migrations will need to be run as part of the deployment process or manually applied to the Neon database.
-   **Build Command**: Ensure a `build` command is defined that installs Python dependencies and prepares the application for deployment (e.g., `pip install -r requirements.txt`).

## Acceptance Criteria

This phase is complete when the following are met:

-   [ ] The FastAPI application successfully starts and serves requests.
-   [ ] The `POST /chat` endpoint processes valid requests and returns AI-generated responses.
-   [ ] Conversation history (last 10 messages) is correctly maintained and sent to OpenAI.
-   [ ] New chat sessions are initiated if `session_id` is not provided.
-   [ ] User messages and AI responses are persistently stored in the Neon PostgreSQL database.
-   [ ] Input validation (Pydantic) for `/chat` endpoint functions correctly, returning `400 Bad Request` for invalid data.
-   [ ] The system prompt correctly frames the AI's persona.
-   [ ] The `selected_text` bonus feature is implemented, ensuring AI responses are grounded in provided text.
-   [ ] All mandatory environment variables are loaded and utilized.
-   [ ] Comprehensive error handling is implemented for API, database, and validation errors.
-   [ ] CORS is correctly configured to allow specified origins.
-   [ ] The `GET /health` endpoint returns a `200 OK` status.
-   [ ] Structured logging is implemented for requests and API calls.
-   [ ] Type hints and docstrings are present in the codebase.
-   [ ] A `pyproject.toml` file is used for package management.
-   [ ] The codebase structure supports easy insertion of a RAG retrieval step.
-   [ ] The database schema is extensible for future RAG needs.
-   [ ] All unit and integration tests pass successfully.
-   [ ] The response time for the `/chat` endpoint is consistently under two seconds.

### Edge Cases

-   **Invalid `session_id`**: If a `session_id` is provided but does not correspond to an existing session, a new session should be created, and the provided `session_id` should be ignored or a new one generated.
-   **OpenAI API Rate Limits/Errors**: The system should return an appropriate `500 Internal Server Error` and log the issue, potentially with a retry mechanism (out of scope for Phase 2).
-   **Database Connection Loss**: If the database becomes unreachable during a chat interaction, the system should return a `500 Internal Server Error` and log the connection issue.
-   **Empty `message`**: Pydantic validation should catch this, returning a `400 Bad Request`.
-   **Excessive `selected_text`**: Extremely long `selected_text` could exceed OpenAI's token limits. This should be handled gracefully by truncating the text or returning an error (out of scope for Phase 2; assume reasonable input length).
-   **CORS Mismatch**: Requests from unapproved origins should be automatically blocked by FastAPI's CORS middleware.