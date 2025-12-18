# Feature Tasks: Docusaurus Chatbot UI

## Feature Name: Docusaurus Chatbot UI
## Feature Branch: `003-docusaurus-chatbot-ui`
## Specification: `specs/003-docusaurus-chatbot-ui/spec.md`
## Plan: `specs/003-docusaurus-chatbot-ui/plan.md`
## Data Model: `specs/003-docusaurus-chatbot-ui/data-model.md`
## API Contracts: `specs/003-docusaurus-chatbot-ui/contracts/chat_api.md`
## Quickstart: `specs/003-docusaurus-chatbot-ui/quickstart.md`

## Phase 1: Setup (Milestone 1 from Plan)

*   **Objective**: Prepare the Docusaurus project and integrate the core ChatKit UI library.

- [x] T001 Audit existing Docusaurus project structure in `frontend/`
- [x] T002 Install required npm packages: `@openai/chatkit-react` in `frontend/package.json`
- [x] T003 Install required npm packages: `axios` (or confirm `fetch` sufficiency) in `frontend/package.json`
- [x] T004 Create `frontend/src/components/Chatbot/ChatBotProvider.tsx` for context management
- [x] T005 Create `frontend/src/components/Chatbot/ChatWidget.tsx` as the main container
- [x] T006 Create `frontend/src/components/Chatbot/MessageList.tsx` for displaying messages
- [x] T007 Create `frontend/src/components/Chatbot/MessageInput.tsx` for user input
- [x] T008 Integrate basic ChatKit `ChatInterface` component into `frontend/src/components/Chatbot/ChatWidget.tsx` with minimal styling
- [x] T009 Verify `frontend/src/components/Chatbot/ChatWidget.tsx` renders correctly in a standalone test page (create `frontend/src/pages/chatbot-test.tsx`)

## Phase 2: Foundational (Milestone 2 from Plan - Core Chat Logic & State Management)

*   **Objective**: Implement the internal state, logic, and connect to the FastAPI backend.

### User Story 1 (US1): Display Chatbot and Basic UI (AC-001, AC-002)

*   **Goal**: An accessible and interactive chatbot appears on all pages of the textbook, displaying past conversation turns, an area for new input, and a submission control.
*   **Independent Test Criteria**: The chatbot widget is visible and toggleable, showing input and display areas.
*   **Implementation Tasks**:
    - [x] T010 [US1] Implement `ChatBotProvider` context in `frontend/src/components/Chatbot/ChatBotProvider.tsx` to manage global state: `messages[]`, `isLoading`, `error`, `sessionId`
    - [x] T011 [US1] Implement `sessionId` generation (using `crypto.randomUUID()`) and persistence logic with `sessionStorage` in `frontend/src/components/Chatbot/ChatBotProvider.tsx`
    - [x] T012 [US1] Connect `MessageList` component to display messages from the context state in `frontend/src/components/Chatbot/MessageList.tsx`
    - [x] T013 [US1] Implement toggle functionality for `ChatWidget` visibility in `frontend/src/components/Chatbot/ChatWidget.tsx`
    - [x] T014 [US1] Position `ChatWidget` (e.g., bottom-right corner) using CSS in `frontend/src/components/Chatbot/ChatWidget.tsx` and `frontend/src/css/custom.css`

### User Story 2 (US2): Send/Receive Messages and Loading Indicators (AC-003, AC-007)

*   **Goal**: Users successfully send queries and receive relevant AI-generated replies within the chatbot interface, with appropriate loading cues.
*   **Independent Test Criteria**: A user can type a message, see it appear, see a loading state, and receive a simulated or real backend response in the UI.
*   **Implementation Tasks**:
    - [x] T015 [US2] Create a service module `frontend/src/services/chatService.js` with `sendMessage(message, selectedText)` function
    - [x] T016 [US2] Implement `sendMessage` function in `frontend/src/services/chatService.js` to construct and send POST request to `/chat` endpoint
    - [x] T017 [US2] Connect `MessageInput` component to call `sendMessage` service and update context state with user messages in `frontend/src/components/Chatbot/MessageInput.tsx`
    - [x] T018 [US2] Update context state with AI responses in `frontend/src/components/Chatbot/ChatBotProvider.tsx` after receiving backend response
    - [x] T019 [US2] Implement basic loading indicators (e.g., "AI is typing...") when `isLoading` is true in `frontend/src/components/Chatbot/ChatWidget.tsx`

### User Story 3 (US3): Conversation History Persistence (AC-004)

*   **Goal**: The conversation history remains intact when users navigate between different pages or revisit the textbook within the same browsing session.
*   **Independent Test Criteria**: Navigate to different pages; the chat history remains. Close and reopen browser tab; history is preserved (within session).
*   **Implementation Tasks**:
    - [x] T020 [US3] Ensure `sessionStorage` is correctly utilized for `sessionId` and conversation history within `frontend/src/components/Chatbot/ChatBotProvider.tsx`

## Phase 3: User Stories (Milestone 3 from Plan - Advanced Features & Docusaurus Integration)

*   **Objective**: Implement key features (text selection) and integrate the widget into the live book.

### User Story 4 (US4): Selected Text Context and Visual Indication (AC-005, AC-006)

*   **Goal**: Any text selected by the user on a textbook page is automatically captured and used as additional context for the subsequent AI query, with a clear visual indication.
*   **Independent Test Criteria**: Highlighting text on a page populates the `MessageInput` or a separate context field, and the AI response indicates its use.
*   **Implementation Tasks**:
    - [x] T021 [US4] Implement a `useTextSelection` hook or utility in `frontend/src/hooks/useTextSelection.js` that captures `window.getSelection()`
    - [x] T022 [US4] Provide the selected string to the chat context in `frontend/src/components/Chatbot/ChatBotProvider.tsx`
    - [x] T023 [US4] Modify `MessageInput` component in `frontend/src/components/Chatbot/MessageInput.tsx` to automatically populate and visually indicate when `selectedText` is available to send
    - [x] T024 [US4] Modify `sendMessage` in `frontend/src/services/chatService.js` to include `selected_text` in the backend request if available
    - [x] T025 [US4] Implement visual indication when the AI's reply has specifically utilized user-selected text for its generation in `frontend/src/components/Chatbot/MessageList.tsx`

## Final Phase: Polish & Cross-Cutting Concerns (Milestone 4 from Plan)

*   **Objective**: Refine the user experience, add resilience, and prepare for the next phase.

### User Story 5 (US5): Error Handling (AC-008)

*   **Goal**: The chatbot interface presents clear, understandable messages to the user when underlying services encounter problems.
*   **Independent Test Criteria**: Backend errors (simulated) are displayed as user-friendly messages.
*   **Implementation Tasks**:
    - [x] T026 [US5] Enhance error handling and user feedback (e.g., network errors, empty messages) in `frontend/src/services/chatService.js` and `frontend/src/components/Chatbot/ChatWidget.tsx`
    - [x] T027 [US5] Implement retry mechanisms or clear error states in `frontend/src/components/Chatbot/ChatWidget.tsx`

### User Story 6 (US6): Visual Design and Integration Consistency (AC-009)

*   **Goal**: The chatbot's visual design and behavior seamlessly integrate with the textbook's existing presentation across various devices and screen sizes.
*   **Independent Test Criteria**: The chatbot UI is visually cohesive with the Docusaurus theme and responsive across devices.
*   **Implementation Tasks**:
    - [x] T028 [US6] Integrate the complete `ChatWidget` into the main Docusaurus layout (`frontend/src/theme/Layout/index.tsx` or similar via `swizzle`)
    - [x] T029 [US6] Ensure the widget's styles do not conflict with the existing Docusaurus theme in `frontend/src/css/custom.css`
    - [x] T030 [US6] Add comprehensive UI/UX polish: smooth animations, improved placeholder text, clearer instructions in `frontend/src/components/Chatbot/` components
    - [x] T031 [US6] Perform cross-browser testing (Chrome, Firefox, Safari) and document findings in `specs/003-docusaurus-chatbot-ui/testing-notes.md` (new file)

### Cross-Cutting Tasks

- [x] T032 Write basic unit tests for `ChatBotProvider` context in `frontend/src/components/Chatbot/ChatBotProvider.test.tsx` (new file)
- [x] T033 Write basic unit tests for utility functions (e.g., `useTextSelection`) in `frontend/src/hooks/useTextSelection.test.js` (new file)
- [x] T034 Document the component API and integration steps for future phases in `specs/003-docusaurus-chatbot-ui/README.md` (new file)
- [x] T035 Verify all Acceptance Criteria from `specs/003-docusaurus-chatbot-ui/spec.md` are met (manual verification)

## Dependencies

The completion of each phase is dependent on the prior phase's completion. Within each user story, tasks can generally be executed in parallel where indicated `[P]`.

*   Phase 1 -> Phase 2 -> Phase 3 -> Final Phase
*   US1, US2, US3 are foundational for subsequent user stories.
*   US4 depends on US2 (sending messages).

## Parallel Execution Examples

*   **During Phase 1 (Setup)**:
    *   T002 and T003 can be executed in parallel (installing npm packages).
    *   T004, T005, T006, T007 can be started in parallel after dependencies are installed.
*   **During US1**:
    *   T010, T011, T012, T013, T014 can be worked on in parallel.
*   **During US2**:
    *   T015, T016, T017, T018, T019 can be worked on in parallel.

## Implementation Strategy

The implementation will follow a Minimum Viable Product (MVP) approach, delivering functionality incrementally. Each user story represents a testable increment.

1.  **Phase 1 (Setup)** will establish the basic environment.
2.  **Phase 2 (Foundational)** will build the core chat components and logic (US1, US2, US3).
3.  **Phase 3 (Advanced Features)** will integrate selected text functionality and full Docusaurus integration (US4).
4.  **Final Phase (Polish & Cross-Cutting Concerns)** will focus on hardening, testing, and documentation (US5, US6, Cross-Cutting Tasks).

The MVP for this feature would encompass successfully completing US1, US2, and US3, allowing a user to engage in a basic, session-persistent chat with the backend. Further user stories build upon this core functionality.