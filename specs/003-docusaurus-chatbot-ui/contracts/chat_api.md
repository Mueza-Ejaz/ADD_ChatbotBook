# API Contract: Chat Endpoint

This document details the API contract for the chat endpoint, used by the Docusaurus Chatbot UI to communicate with the FastAPI backend.

## Endpoint

*   **URL**: `/chat`
*   **Method**: `POST`

## Request

The frontend sends a JSON payload to the `/chat` endpoint.

*   **Content-Type**: `application/json`

```json
{
    "message": "string",            // The user's query or message. (Required)
    "session_id": "string",         // The current conversation session ID. (Optional)
                                    // Will be omitted if starting a new conversation or if client-side session management is not yet established.
    "selected_text": "string"       // Text highlighted by the user on the Docusaurus page, providing additional context. (Optional)
}
```

## Response

The backend responds with a JSON payload containing the AI's reply and conversation metadata.

*   **Content-Type**: `application/json`
*   **Status Codes**:
    *   `200 OK`: Successful response.
    *   `400 Bad Request`: Invalid request payload (e.g., missing `message`).
    *   `500 Internal Server Error`: An unexpected error occurred on the server or with the AI service.

```json
{
    "response": "string",           // The AI's generated reply.
    "session_id": "string",         // The session ID of the conversation. This will be the same as the request's `session_id` if provided,
                                    // or a new one if a new conversation was initiated by the backend.
    "timestamp": "datetime"         // The time when the AI response was generated (ISO 8601 format, e.g., "2025-12-18T12:30:00Z").
}
```

## Error Response Example

```json
{
    "detail": "string"              // A descriptive error message.
}
```