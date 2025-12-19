# Data Models: Minimalistic FastAPI Backend for Chat

**Feature Branch**: `002-fastapi-chat-backend`
**Created**: 2025-12-17

This document outlines the data models for the Minimalistic FastAPI Backend, focusing on entities stored in the Neon PostgreSQL database.

## Entities

### Entity: ChatSession

-   **Description**: Represents a unique, persistent conversation thread between a user and the chatbot. Each new conversation or explicit new session request initiates a new `ChatSession`.
-   **Purpose**: To group related messages and maintain conversation context over time.
-   **Fields**:
    -   `id`:
        -   **Type**: UUID (Universally Unique Identifier)
        -   **Constraints**: Primary Key, Non-nullable, Auto-generated (e.g., `uuid.uuid4()`)
        -   **Description**: A unique identifier for the chat session.
    -   `created_at`:
        -   **Type**: Timestamp with timezone
        -   **Constraints**: Non-nullable, Auto-generated on creation (e.g., `datetime.utcnow()`)
        -   **Description**: The timestamp when the chat session was initiated.
    -   `updated_at`:
        -   **Type**: Timestamp with timezone
        -   **Constraints**: Non-nullable, Auto-updated on modification (e.g., `datetime.utcnow()` on update)
        -   **Description**: The timestamp when any message belonging to this session was last added or modified.
-   **Relationships**:
    -   Has a one-to-many relationship with `Message` entities. A single `ChatSession` can contain multiple `Message` records.
-   **Validation Rules**:
    -   `id` must be a valid UUID.
    -   `created_at` and `updated_at` must be valid timestamps.
    -   `updated_at` should always be greater than or equal to `created_at`.
-   **State Transitions**:
    -   A `ChatSession` is created when a new conversation starts (i.e., no `session_id` is provided in a `/chat` request) or an explicit request to start a new session is made.
    -   `updated_at` is updated whenever a new message (user or assistant) is added to the session.

### Entity: Message

-   **Description**: Represents an individual piece of communication (either from the user, the assistant, or a system prompt) within a specific `ChatSession`.
-   **Purpose**: To store the chronological history of a conversation, allowing for context retrieval.
-   **Fields**:
    -   `id`:
        -   **Type**: UUID
        -   **Constraints**: Primary Key, Non-nullable, Auto-generated (e.g., `uuid.uuid4()`)
        -   **Description**: A unique identifier for the message.
    -   `session_id`:
        -   **Type**: UUID
        -   **Constraints**: Foreign Key referencing `ChatSession.id`, Non-nullable
        -   **Description**: Links this message to its parent chat session.
    -   `role`:
        -   **Type**: Enumeration (String)
        -   **Constraints**: Non-nullable, Allowed values: `'user'`, `'assistant'`, `'system'`
        -   **Description**: Defines the sender of the message (e.g., the human user, the AI assistant, or an internal system prompt).
    -   `content`:
        -   **Type**: Text (String)
        -   **Constraints**: Non-nullable, Cannot be empty.
        -   **Description**: The actual textual content of the message.
    -   `created_at`:
        -   **Type**: Timestamp with timezone
        -   **Constraints**: Non-nullable, Auto-generated on creation (e.g., `datetime.utcnow()`)
        -   **Description**: The timestamp when the message was recorded.
-   **Relationships**:
    -   Belongs to one `ChatSession`.
-   **Validation Rules**:
    -   `id` must be a valid UUID.
    -   `session_id` must reference an existing `ChatSession`.
    -   `role` must be one of `'user'`, `'assistant'`, or `'system'`.
    -   `content` must be a non-empty string.
    -   `created_at` must be a valid timestamp.
-   **State Transitions**:
    -   A `Message` is created whenever a user sends input or the AI generates a response within a session.
    -   `content` is immutable after creation.
