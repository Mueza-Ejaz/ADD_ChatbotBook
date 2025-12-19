# Feature Specification: Docusaurus Chatbot UI

**Feature Branch**: `003-docusaurus-chatbot-ui`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "Create a complete specification for Phase 3: Integrate a ChatKit chatbot UI into the Docusaurus textbook and connect it to the existing FastAPI backend (Phase 2). Project Context: This is the next phase of a hackathon to build an AI-native "Physical AI & Humanoid Robotics" textbook. Phase 1 (book content) is complete. Phase 2 (FastAPI backend with /chat endpoint) is in development. This phase focuses on the frontend chatbot. Core Objective: Embed a functional, interactive chat widget into the Docusaurus book using OpenAI's ChatKit React components. This UI must send user messages to and display responses from the Phase 2 backend API. Detailed Requirements: 1. Tech Stack: Use the existing Docusaurus (React) project. Integrate the @openai/chatkit React library for the UI. Manage chat state with React hooks. Use fetch or axios to call the backend. 2. Chat Widget: Create a ChatBotWidget component. It should be a fixed, toggleable UI element (e.g., in the bottom corner). It must display a message list, a text input area, and a send button. Show typing indicators and errors. 3. Backend Connection: The widget must call the Phase 2 POST /chat endpoint. * Request: Send { "message": userInput, "session_id": storedId, "selected_text": highlightedText }. * Response: Handle { "response": assistantReply, "session_id": id, "timestamp": ... }. * Generate and persist a session_id in the browser's sessionStorage to maintain conversation context. 4. Key Feature - Selected Text Context: Implement logic to capture text a user selects (highlights) on any book page. Automatically include this text in the selected_text request field. Visually indicate when the chatbot is answering based on a selection. 5. Component Structure: Design a clean React component structure (e.g., ChatWidget, MessageList, MessageInput). Use React state/context to manage messages, loading states, and the session. 6. Docusaurus Integration: Plan how to inject the ChatBotWidget into the main Docusaurus layout (src/theme/Layout.jsx). Ensure it doesn't break existing site navigation or styles. 7. Acceptance Criteria: * A chat widget is visible and functional on all book pages. * Users can send questions and see responses from the backend. * Conversation history persists during a browser session. * Selected text from the book is captured and sent to the backend. * The UI displays loading states and handles errors gracefully. Output: Generate a spec.md file with these sections: 1. Overview & Objectives 2. System Architecture (Data flow diagram description) 3. Component Specifications & Props 4. API Integration & State Management Logic 5. Docusaurus Integration Steps 6. Testing Strategy 7. Acceptance Criteria Checklist."

## 1. Overview & Objectives

This specification outlines the integration of an interactive chatbot interface into the Docusaurus-based "Physical AI & Humanoid Robotics" textbook. The primary objective is to provide users with a functional and interactive chat experience, allowing them to ask questions related to the textbook content and receive AI-generated responses. This interface will be embedded within the existing Docusaurus application and will communicate with the established backend service to process chat requests, maintain conversation context, and optionally incorporate user-selected text from the book for enhanced AI responses. The goal is to create a seamless and context-aware conversational AI tool that enhances the learning experience.

## 2. System Architecture (Data flow description)

The system comprises three conceptual layers: the User Interface, the Chatbot Interface, and the Backend Service.

1.  **User Interaction**: A user interacts with the Chatbot Interface, which is integrated directly into the textbook's web pages.
2.  **Contextual Input**: The user can highlight text on any page of the textbook. This highlighted text is automatically captured and associated with the user's input.
3.  **Sending Query**: The user types a question or message into the Chatbot Interface and submits it.
4.  **Frontend Processing**: The Chatbot Interface packages the user's message, any captured contextual text, and a unique conversation identifier (if one exists) into a request.
5.  **Service Request**: This request is sent asynchronously from the user's browser to the Backend Service.
6.  **Backend Processing**: The Backend Service receives the request, processes the user's query and contextual information, leverages an external AI model to generate a response, and manages the conversation history.
7.  **Response Delivery**: The Backend Service returns the AI-generated response and the current unique conversation identifier to the Chatbot Interface.
8.  **Interface Update**: The Chatbot Interface displays the AI's response to the user, updates its internal record of the conversation, and visually indicates if the response was influenced by highlighted text.
9.  **Conversation Persistence**: The unique conversation identifier is stored locally in the user's browser, allowing the conversation to continue from where it left off across different pages or visits within a browser session.

## 3. Interface Element Specifications

### Chatbot Interface (Root Element)

*   **Description**: A discreet, interactive element positioned consistently on all book pages, allowing users to initiate and manage conversations. It can be toggled open/closed.
*   **Attributes**:
    *   `apiEndpoint`: `string` - The network address for sending chat requests to the Backend Service.
*   **Internal State**: Manages the sequence of messages exchanged, indicators for AI activity (e.g., "typing"), any active error messages, and the unique identifier for the ongoing conversation.
*   **Composed Of**: Message Display Area, Message Input Area.

### Message Display Area

*   **Description**: Presents the chronological flow of messages, distinguishing between user queries and AI responses.
*   **Attributes**:
    *   `conversationLog`: `Array<Object>` - A collection of objects, each representing a turn in the conversation with content and speaker indication (user/AI).
    *   `isProcessingAIResponse`: `boolean` - Signifies when the system is generating an AI response.
    *   `isContextuallyDriven`: `boolean` - Indicates if the most recent AI response was specifically informed by highlighted text.

### Message Input Area

*   **Description**: Provides a mechanism for users to compose and submit their questions or messages to the AI.
*   **Attributes**:
    *   `onSubmitMessage`: `Function` - An action triggered when the user sends a message. It accepts the message content and any captured contextual text.
    *   `inputActive`: `boolean` - Controls whether the input area is available for user interaction (e.g., disabled during AI processing).

## 4. Backend Communication & Conversation Management

### Backend Communication

*   **Method**: Asynchronous request to the designated chat endpoint on the Backend Service.
*   **Information Sent**:
    *   `userQuery`: `string` - The user's typed message.
    *   `conversationId`: `string` - (Optional) The unique identifier of the current conversation.
    *   `contextualText`: `string` - (Optional) Text that the user has highlighted.
*   **Information Received**:
    *   `aiResponse`: `string` - The AI's generated reply.
    *   `conversationId`: `string` - The unique identifier for the conversation (could be new or existing).
    *   `responseTimestamp`: `datetime` - The time at which the AI response was generated.
*   **Error Handling**: The Chatbot Interface must present clear and actionable messages to the user if the Backend Service encounters issues.

### Conversation Management

*   **Message History**: The sequence of interactions is managed internally by the Chatbot Interface, often requiring a mechanism to efficiently update and display new messages.
*   **Conversation Identifier**: A unique conversation identifier is maintained across user's visits within the same browser session. If a user starts a new conversation or no identifier is found, a new one is established.
*   **Processing Indicators**: Visual cues (e.g., animated ellipsis) inform the user when the AI is processing their request.
*   **Error Display**: User-friendly messages are shown to communicate any problems encountered during the interaction.
*   **Highlighted Text Handling**: When a user selects text, this text is stored and made available for inclusion in the next query to the Backend Service. A visual distinction should be made if the AI's answer is directly influenced by this selection.

## 5. Textbook Integration Approach

1.  **Layout Customization**: Modify the core layout mechanism of the Docusaurus textbook to include the Chatbot Interface as a persistent element. This must be done in a way that preserves the existing navigational and structural integrity of the site.
2.  **Element Placement**: The Chatbot Interface will be positioned in a fixed location (e.g., bottom-right corner) on all content pages.
3.  **Content Interaction**: Implement a mechanism to detect and capture any text selected by the user while browsing the textbook content. This captured text will then be used by the Chatbot Interface.
4.  **Visual Consistency**: Ensure the aesthetic design and interactive behavior of the Chatbot Interface align seamlessly with the overall look and feel of the Docusaurus textbook, avoiding any disruption to the user's reading experience.

## 6. Testing Strategy

*   **Individual Element Verification**:
    *   Confirm that each distinct part of the Chatbot Interface (e.g., message display, input field, toggle button) renders correctly and responds to user actions as expected.
    *   Validate how these elements behave when receiving different types of data or state changes (e.g., empty message list, long messages, loading state).
*   **Interface Flow Validation**:
    *   Test the complete sequence of user interaction, from typing a message, sending it, receiving an AI response, and displaying it, including the persistence of conversation context across pages.
    *   Verify that selected text is accurately captured and influences the AI's response when intended.
    *   Ensure proper visual feedback for loading states and error conditions.
*   **End-to-End System Validation**:
    *   Simulate a full user journey through the Docusaurus textbook, interacting with the Chatbot Interface and confirming that the entire system (from frontend to backend to external AI service) functions as expected.
    *   Validate the continuous flow of conversation and the accurate use of contextual information.

## 7. Acceptance Criteria Checklist

*   **AC-001**: An accessible and interactive chatbot appears on all pages of the textbook, operable by a toggle mechanism.
*   **AC-002**: The chatbot displays past conversation turns, an area for new input, and a submission control.
*   **AC-003**: Users successfully send queries and receive relevant AI-generated replies within the chatbot interface.
*   **AC-004**: The conversation history remains intact when users navigate between different pages or revisit the textbook within the same browsing session.
*   **AC-005**: Any text selected by the user on a textbook page is automatically captured and used as additional context for the subsequent AI query.
*   **AC-006**: The chatbot provides a clear visual indication when the AI's reply has specifically utilized user-selected text for its generation.
*   **AC-007**: The interface shows a visual cue (e.g., "AI is typing...") while an AI response is being generated.
*   **AC-008**: The chatbot interface presents clear, understandable messages to the user when underlying services encounter problems.
*   **AC-009**: The chatbot's visual design and behavior seamlessly integrate with the textbook's existing presentation across various devices and screen sizes.

## Edge Cases

-   **EC-001**: What happens if the user selects text that is extremely long (e.g., an entire chapter)? (Expected: Truncation or backend handling of large inputs.)
-   **EC-002**: How does the system behave if the AI service is unavailable or returns an error (e.g., API key invalid, rate limit exceeded)? (Expected: Graceful error display, retry mechanism or clear message.)
-   **EC-003**: What is the behavior if the user's network connection is lost during a chat interaction? (Expected: Informative error message, ability to retry.)
-   **EC-004**: How does the chatbot handle very short or ambiguous user messages, especially without selected text? (Expected: AI attempts to respond generally or asks for clarification.)
-   **EC-005**: What happens if `sessionStorage` is unavailable or disabled in the user's browser? (Expected: New session is created for every interaction, with a potential warning or degraded experience indication.)
-   **EC-006**: How does the chatbot distinguish between normal user messages and queries meant to specifically leverage selected text if both are present? (Expected: Selected text takes precedence as a strong contextual signal.)