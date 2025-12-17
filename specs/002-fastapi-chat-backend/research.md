# Research Findings: Minimalistic FastAPI Backend for Chat

**Feature Branch**: `002-fastapi-chat-backend`
**Created**: 2025-12-17

## Resolution of NEEDS CLARIFICATION from plan.md

---

### 1. Logging Level and Granularity

-   **Clarification Needed**: Define specific logging levels and event granularity for structured logging.

-   **Decision**: Implement structured logging using the `Loguru` library.
    -   **Log Levels**:
        -   `INFO`: Used for general application flow (e.g., request start/end, health checks, successful API calls).
        -   `DEBUG`: Used for detailed operational insights (e.g., specific database queries, OpenAI API request/response payloads).
        -   `ERROR`: Used for exceptions, critical failures (e.g., unhandled errors, external service outages).
    -   **Event Granularity**: Logs will include:
        -   `request_id` (if available, for tracing across requests).
        -   `session_id` (for chat-related logs).
        -   `endpoint`, `method`, `path`.
        -   `status_code` (for responses).
        -   `duration_ms` (for processing time).
        -   `message` (user-friendly description of the event).
        -   `error_type`, `error_message`, `stack_trace` (for error logs).
        -   Contextual data relevant to the event (e.g., `user_message_length`, `ai_response_length`).

-   **Rationale**: `Loguru` provides a clean, user-friendly API for structured logging, automatic stack trace handling, and easy configuration. The chosen granularity ensures sufficient detail for monitoring and debugging without excessive log volume under normal operation.

-   **Alternatives Considered**:
    -   **Python's built-in `logging` module**: More verbose setup required for structured output, especially when dealing with custom fields and formatting for different handlers.
    -   **`structlog`**: Excellent for structured logging but requires more explicit integration with various log processors and a slightly steeper learning curve compared to `Loguru` for quick setup.

---

### 2. Message History Strategy

-   **Clarification Needed**: Precisely define the strategy for selecting and sending "approximately the last ten messages" to the OpenAI API.

-   **Decision**: The system will retrieve the last 10 messages *exchanged within a specific chat session* from the database.
    -   Messages will be ordered by their `created_at` timestamp in ascending order (oldest first).
    -   If a session has fewer than 10 messages, all available messages for that session will be retrieved.
    -   This list of messages will then be formatted into the `messages` array structure (`[{"role": "user", "content": "..."}]`) as required by the OpenAI Chat Completions API.
    -   The system prompt will always be prepended to this message history when constructing the payload for OpenAI.

-   **Rationale**: This approach provides a consistent and manageable context window for the AI, balancing conversational coherence with API token usage limits. Retrieving from the database ensures persistence and accuracy of the conversation flow.

-   **Alternatives Considered**:
    -   **Retrieving all messages**: Could lead to very long contexts for extended conversations, exceeding OpenAI's token limits and incurring higher costs.
    -   **Dynamic message selection based on token count**: More complex to implement in this initial phase and potentially introduces variability that could be harder to debug.
    -   **Storing summarized context**: Adds complexity to the database schema and application logic, also out of scope for a minimalistic initial phase.

---
