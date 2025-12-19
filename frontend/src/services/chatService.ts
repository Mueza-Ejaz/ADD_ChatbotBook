// frontend/src/services/chatService.ts

import axios from 'axios';

// Define the API endpoint for the chat service
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000'; // Default to backend's default port

interface ChatRequest {
  message: string;
  session_id?: string;
  selected_text?: string;
}

interface ChatResponse {
  response: string;
  session_id: string;
  timestamp: string;
}

/**
 * Sends a message to the backend chat service.
 * @param message The user's message.
 * @param sessionId The current chat session ID (optional).
 * @param selectedText Any text selected by the user (optional).
 * @returns A promise that resolves with the AI's response.
 */
export const sendMessage = async (
  message: string,
  sessionId?: string,
  selectedText?: string
): Promise<ChatResponse> => {
  const requestBody: ChatRequest = {
    message,
  };

  if (sessionId) {
    requestBody.session_id = sessionId;
  }
  if (selectedText) {
    requestBody.selected_text = selectedText;
  }

  try {
    const response = await axios.post<ChatResponse>(`${API_BASE_URL}/chat`, requestBody);
    return response.data;
  } catch (error) {
    if (axios.isAxiosError(error)) {
      console.error('Axios error sending message:', error.response?.data || error.message);
      throw new Error(error.response?.data?.detail || 'Failed to send message.');
    } else {
      console.error('Unexpected error sending message:', error);
      throw new Error('An unexpected error occurred.');
    }
  }
};
