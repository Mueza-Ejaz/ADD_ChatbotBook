# Docusaurus Chatbot UI Feature Documentation

This document provides an overview and integration guide for the Docusaurus Chatbot UI feature.

## 1. Overview

This feature integrates an interactive chatbot into the Docusaurus textbook, allowing users to ask questions and receive AI-generated responses, optionally leveraging selected text from the book as context.

## 2. Component API

The core component of this feature is the `ChatWidget`, located at `frontend/src/components/Chatbot/ChatWidget.tsx`.

### `ChatWidget`

*   **Location**: `frontend/src/components/Chatbot/ChatWidget.tsx`
*   **Description**: The main entry point for the chatbot UI. It renders a fixed, toggleable chat interface on the page.
*   **Props**:
    *   Currently, `ChatWidget` does not accept any direct props, as its state and logic are managed internally via React Context (`ChatBotProvider`) and the `@openai/chatkit-react` library.
*   **Dependencies**:
    *   `ChatBotProvider` (for context management)
    *   `@openai/chatkit-react` (for the core chat interface)
    *   `useTextSelection` hook (for text selection functionality)
    *   `chatService.ts` (for backend communication)

### `ChatBotProvider`

*   **Location**: `frontend/src/components/Chatbot/ChatBotProvider.tsx`
*   **Description**: Provides the chat context (messages, loading state, error, session ID, selected text) to its children components. Manages session persistence via `sessionStorage`.
*   **Context Values Provided**:
    *   `messages`: `ChatMessage[]` - Array of chat messages.
    *   `isLoading`: `boolean` - Indicates if an AI response is being generated.
    *   `error`: `string | null` - Stores any error message.
    *   `sessionId`: `string | null` - Unique ID for the current chat session, persisted in `sessionStorage`.
    *   `selectedText`: `string` - Currently selected text from the page.
    *   `addMessage`: `(message: ChatMessage) => void` - Function to add a new message to the chat.
    *   `setLoading`: `(loading: boolean) => void` - Function to set the loading state.
    *   `setError`: `(error: string | null) => void` - Function to set an error message.
    *   `setSessionId`: `(id: string | null) => void` - Function to set the session ID.

### `useTextSelection` Hook

*   **Location**: `frontend/src/hooks/useTextSelection.ts`
*   **Description**: A custom React hook that captures the currently selected text in the browser's DOM using `window.getSelection()`.
*   **Returns**: `string` - The currently selected text, or an empty string.

## 3. Integration Steps

### A. Core Integration (Completed in Phase 1 & 2)

1.  **Dependencies**: Ensure `@openai/chatkit-react` and `axios` are installed in `frontend/package.json`.
2.  **Docusaurus Configuration**: The ChatKit JS script (`https://cdn.platform.openai.com/deployments/chatkit/chatkit.js`) has been added to `frontend/docusaurus.config.ts`.
3.  **Layout Integration**: The `ChatWidget` component has been integrated into the main Docusaurus layout by rendering it in `frontend/src/theme/Layout/index.tsx`.
4.  **Styling**: Custom CSS for the chatbot UI elements has been added to `frontend/src/css/custom.css`.

### B. Backend API Endpoint

The chatbot communicates with a FastAPI backend via the `/chat` endpoint.

*   **Configuration**: The `API_BASE_URL` for the backend should be configured in the frontend environment (e.g., via `process.env.REACT_APP_API_BASE_URL`).
*   **Request/Response**: Refer to `specs/003-docusaurus-chatbot-ui/contracts/chat_api.md` for the detailed API contract.

### C. Future Development & Theming

*   **Theming**: The `ChatWidget` uses Docusaurus CSS variables (e.g., `--ifm-color-primary`) for basic styling. Further theming can be achieved by overriding these variables or directly styling the chatbot's internal components.
*   **Custom UI with ChatKit**: If more granular control over the ChatKit UI is required, explore ChatKit's customization options or "swizzle" its internal components if supported.
*   **Error Handling**: Consider more sophisticated error display and user guidance for specific error types.

## 4. Testing

Refer to `specs/003-docusaurus-chatbot-ui/testing-notes.md` for cross-browser testing findings and a checklist. Unit tests for `ChatBotProvider` and `useTextSelection` are available in their respective `*.test.tsx` files.
