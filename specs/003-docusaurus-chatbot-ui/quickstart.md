# Quickstart: Docusaurus Chatbot UI

This guide provides a quick overview of how to set up and run the Docusaurus Chatbot UI.

## Prerequisites

*   Node.js (LTS version recommended)
*   npm or Yarn package manager
*   Access to the existing Docusaurus project (`frontend/`)
*   A running instance of the FastAPI backend with the `/chat` endpoint accessible.

## 1. Navigate to the Frontend Directory

Open your terminal or command prompt and navigate to the `frontend` directory of your project:

```bash
cd frontend
```

## 2. Install Dependencies

Install the required Node.js packages, including `@openai/chatkit-react` and `axios` (if used).

```bash
npm install
# or
yarn install
```

## 3. Configure Backend API Endpoint

The chatbot UI will need to know where to send its requests. Ensure that the `API_BASE_URL` (or similar environment variable/configuration) for your FastAPI backend is correctly set within the Docusaurus project. This might involve creating a `.env` file or modifying an existing configuration file.

Example (adjust according to actual implementation):

```javascript
// In a configuration file or environment variable
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || "http://localhost:8000";
```

## 4. Start the Docusaurus Development Server

Once dependencies are installed and the API endpoint is configured, you can start the Docusaurus development server:

```bash
npm start
# or
yarn start
```

This will typically open your Docusaurus site in your web browser at `http://localhost:3000` (or a similar address).

## 5. Interact with the Chatbot

*   After the Docusaurus site loads, locate the chatbot widget (e.g., in the bottom-right corner of the page).
*   Click the toggle button to open the chat interface.
*   Type your message into the input area and press Enter or click the send button.
*   Observe the AI's response.
*   Try highlighting text on the page before sending a message to see the "selected text" feature in action.

## Troubleshooting

*   If the Docusaurus site fails to build or run, check the console for error messages and ensure all dependencies are correctly installed.
*   If the chatbot does not respond, verify that your FastAPI backend is running and accessible from the frontend. Check browser developer tools (Network tab) for failed API requests.
*   Ensure `sessionStorage` is enabled in your browser if conversation history is not persisting.
*   Check for any conflicting styles if the chatbot UI does not render correctly.