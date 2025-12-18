# Data Model: Docusaurus Chatbot UI

This document outlines the key data entities and their structures for the Docusaurus Chatbot UI, based on the feature specification and development plan.

## 1. Message

Represents a single message within a conversation, sent by either the user or the AI.

*   **`id`**: `string` (UUID)
    *   Unique identifier for the message.
*   **`sender`**: `enum ('user', 'ai')`
    *   Indicates who sent the message.
*   **`content`**: `string`
    *   The text content of the message.
*   **`timestamp`**: `datetime` (ISO 8601 format)
    *   The time when the message was sent or received.
*   **`isContextuallyDriven`**: `boolean` (Optional, default: `false`)
    *   *Applies only to AI messages.* Indicates if the AI's response was specifically influenced by selected text.

## 2. Conversation Session

Represents an ongoing conversation between the user and the AI. Managed client-side.

*   **`id`**: `string` (UUID)
    *   Unique identifier for the conversation session. This `session_id` is persisted in `sessionStorage`.
*   **`messages`**: `Array<Message>`
    *   A chronological list of `Message` objects within this session.

## 3. User Input (for API Request)

The structure of the data sent from the frontend to the backend `/chat` endpoint.

*   **`message`**: `string`
    *   The user's query or message.
*   **`session_id`**: `string` (Optional)
    *   The current conversation session ID. Will be omitted if starting a new conversation.
*   **`selected_text`**: `string` (Optional)
    *   Text highlighted by the user on the Docusaurus page, providing additional context.

## 4. AI Response (from API)

The structure of the data received from the backend `/chat` endpoint.

*   **`response`**: `string`
    *   The AI's generated reply.
*   **`session_id`**: `string`
    *   The session ID of the conversation. This will be the same as the request's `session_id` if provided, or a new one if a new conversation was initiated.
*   **`timestamp`**: `datetime` (ISO 8601 format)
    *   The time when the AI response was generated.

## 5. UI State Management

Additional state variables managed by the React frontend for UI presentation.

*   **`isLoading`**: `boolean`
    *   Indicates if an AI response is currently being generated (for typing indicators).
*   **`error`**: `string` (Optional)
    *   Stores any error messages to be displayed to the user.
*   **`isChatWidgetOpen`**: `boolean`
    *   Controls the visibility of the chatbot widget.
*   **`highlightedText`**: `string` (Optional)
    *   Temporarily stores the user's currently selected text before it's sent with a message.