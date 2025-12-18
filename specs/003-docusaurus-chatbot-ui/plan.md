# Development Plan: Docusaurus Chatbot UI

**Feature Branch**: `003-docusaurus-chatbot-ui`
**Created**: 2025-12-18
**Status**: Draft
**Specification**: `specs/003-docusaurus-chatbot-ui/spec.md`

## 1. Milestone-Based Approach

This plan organizes the implementation of the Docusaurus Chatbot UI into four sequential milestones, ensuring a structured and progressive development process.

## 2. Milestone 1: Frontend Project Setup & ChatKit Integration

*   **Objective**: Prepare the Docusaurus project and integrate the core ChatKit UI library.
*   **Key Tasks**:
    *   Audit the existing Docusaurus project structure and identify the best location for the chatbot components (e.g., `src/components/chatbot/`).
    *   Install the required npm packages: `@openai/chatkit`, `axios` (or confirm `fetch` is sufficient).
    *   Create the foundational React component structure: `ChatBotProvider` (context), `ChatWidget` (main container), `MessageList`, `MessageInput`.
    *   Integrate the basic ChatKit `ChatInterface` component into `ChatWidget` with minimal styling.
    *   Verify the component renders correctly in a standalone test page.
*   **Deliverables**: A working Docusaurus environment with ChatKit installed and a basic, renderable chat component.
*   **Validation**: The project builds without errors, and a test page displays the empty chat interface.

## 3. Milestone 2: Core Chat Logic & State Management

*   **Objective**: Implement the internal state, logic, and connect to the FastAPI backend.
*   **Key Tasks**:
    *   Implement the `ChatBotProvider` context to manage global state: `messages[]`, `isLoading`, `error`, `sessionId`.
    *   Create a service module (`chatService.js`) with a function `sendMessage(message, selectedText)` that constructs and sends the POST request to the Phase 2 backend (`/chat`).
    *   Implement the `sessionId` generation (using `crypto.randomUUID()`) and persistence logic with `sessionStorage`.
    *   Connect the `MessageInput` component to call the `sendMessage` service and update the context state with user messages and AI responses.
    *   Connect the `MessageList` component to display messages from the context state.
    *   Implement basic loading indicators and error message displays.
*   **Dependencies**: Milestone 1 must be complete. Assumes the Phase 2 backend endpoint is reachable (provide placeholder URL if not).
*   **Deliverables**: A functional chat widget that can send/receive messages from the backend and maintain session state.
*   **Validation**: A user can type a message, see it appear, see a loading state, and receive a simulated or real backend response in the UI.

## 4. Milestone 3: Advanced Features & Docusaurus Integration

*   **Objective**: Implement key features (text selection) and integrate the widget into the live book.
*   **Key Tasks**:
    *   Implement the "selected text" feature: Add a `useTextSelection` hook or utility that captures `window.getSelection()` and provides the selected string to the chat context.
    *   Modify the `MessageInput` component to automatically populate and visually indicate when a `selectedText` value is available to send.
    *   Integrate the complete `ChatWidget` into the main Docusaurus layout (`src/theme/Layout.jsx` or via `swizzle`).
    *   Style the widget to be a fixed-position element (e.g., bottom-right corner) with toggle (minimize/maximize) functionality.
    *   Ensure the widget's styles do not conflict with the existing Docusaurus theme.
*   **Dependencies**: Milestone 2 must be complete.
*   **Deliverables**: The chatbot is fully embedded in the textbook, with the selected text feature working.
*   **Validation**: The chat widget is visible on all book pages. Highlighting text on the page pre-fills the chat context. The UI is visually cohesive.

## 5. Milestone 4: Polishing, Testing & Handoff

*   **Objective**: Refine the user experience, add resilience, and prepare for the next phase.
*   **Key Tasks**:
    *   Enhance error handling and user feedback (e.g., network errors, empty messages).
    *   Add comprehensive UI/UX polish: smooth animations, improved placeholder text, clearer instructions.
    *   Write basic unit tests for the context and utility functions.
    *   Perform cross-browser testing (Chrome, Firefox, Safari).
    *   Document the component API and integration steps for future phases.
    *   Verify all Acceptance Criteria from the specification are met.
*   **Dependencies**: Milestone 3 must be complete.
*   **Deliverables**: A robust, tested, and documented chatbot UI integrated into the Docusaurus textbook.
*   **Validation**: All specified features work reliably. The code is clean, commented, and ready for Phase 4 (UI theming).

## Key Decisions & Rationale

*   **ChatKit React components**: Utilize `@openai/chatkit-react` for pre-built UI components and state management, accelerating development.
*   **Session Management**: Use `sessionStorage` for `session_id` to maintain conversation context within a browser session, balancing persistence and privacy.
*   **Text Selection**: Implement a global `mouseup` event listener for capturing selected text, ensuring broad compatibility across Docusaurus content.

## Interfaces and API Contracts

*   **Backend `/chat` Endpoint**:
    *   **Request (POST)**:
        ```json
        {
            "message": "string",
            "session_id": "string",  // Optional
            "selected_text": "string" // Optional
        }
        ```
    *   **Response (200 OK)**:
        ```json
        {
            "response": "string",
            "session_id": "string",
            "timestamp": "datetime"
        }
        ```
    *   **Error Responses (e.g., 4xx, 5xx)**: Standard HTTP error codes with descriptive error messages.

## Non-Functional Requirements (NFRs)

*   **Performance**: Chatbot should respond within 2-3 seconds for typical queries. UI rendering should be smooth.
*   **Responsiveness**: The chatbot UI must be fully responsive and functional across desktop, tablet, and mobile devices.
*   **Accessibility**: Adhere to basic accessibility standards for interactive elements.
*   **Reliability**: Graceful degradation and clear error messages in case of backend or network issues.

## Risks and Mitigation

*   **Risk**: Complex Docusaurus integration leading to conflicts with existing layouts.
    *   **Mitigation**: Start with minimal injection, progressively add complexity, and thoroughly test after each step. Consult Docusaurus documentation for best practices in component overriding.
*   **Risk**: Performance issues with large selected text or frequent API calls.
    *   **Mitigation**: Implement input throttling/debouncing for text selection. Optimize API calls and ensure efficient state updates in React.
*   **Risk**: Security concerns with API key exposure or data handling.
    *   **Mitigation**: Ensure API keys are handled securely on the backend, not exposed on the frontend. Adhere to best practices for secure data transmission (HTTPS).

## Evaluation and Validation

*   All Acceptance Criteria (AC-001 to AC-009) from the `spec.md` must be met.
*   All identified Edge Cases (EC-001 to EC-006) should be handled gracefully.
*   Manual and automated tests pass.
*   Code review completed.