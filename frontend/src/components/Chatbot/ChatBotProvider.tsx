import React, { createContext, useState, useEffect, ReactNode } from 'react';
import { useTextSelection } from '../../hooks/useTextSelection'; // Import the hook

interface ChatMessage {
  id: string;
  sender: 'user' | 'ai';
  content: string;
  timestamp: string;
  isContextuallyDriven?: boolean;
}

interface ChatContextType {
  messages: ChatMessage[];
  isLoading: boolean;
  error: string | null;
  sessionId: string | null;
  selectedText: string; // Add selectedText to context
  addMessage: (message: ChatMessage) => void;
  setLoading: (loading: boolean) => void;
  setError: (error: string | null) => void;
  setSessionId: (id: string | null) => void;
}

export const ChatContext = createContext<ChatContextType | undefined>(undefined);

interface ChatBotProviderProps {
  children: ReactNode;
}

export const ChatBotProvider: React.FC<ChatBotProviderProps> = ({ children }) => {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);
  const [sessionId, setSessionIdState] = useState<string | null>(null);
  const selectedText = useTextSelection(); // Use the hook

  // Function to set sessionId and persist to sessionStorage
  const setSessionId = (id: string | null) => {
    if (id) {
      sessionStorage.setItem('chatSessionId', id);
    } else {
      sessionStorage.removeItem('chatSessionId');
    }
    setSessionIdState(id);
  };

  useEffect(() => {
    // Load session ID from sessionStorage on component mount
    let storedSessionId = sessionStorage.getItem('chatSessionId');
    if (!storedSessionId) {
      // If no session ID, generate a new one
      storedSessionId = crypto.randomUUID();
      sessionStorage.setItem('chatSessionId', storedSessionId);
    }
    setSessionIdState(storedSessionId);

    // Load messages from sessionStorage on component mount
    const storedMessages = sessionStorage.getItem('chatMessages');
    if (storedMessages) {
      try {
        setMessages(JSON.parse(storedMessages));
      } catch (e) {
        console.error("Failed to parse stored messages from sessionStorage", e);
        sessionStorage.removeItem('chatMessages');
      }
    }
  }, []);

  // Effect to persist messages to sessionStorage whenever they change
  useEffect(() => {
    sessionStorage.setItem('chatMessages', JSON.stringify(messages));
  }, [messages]);

  const addMessage = (message: ChatMessage) => {
    setMessages((prevMessages) => [...prevMessages, message]);
  };

  const setLoading = (loading: boolean) => {
    setIsLoading(loading);
  };

  return (
    <ChatContext.Provider
      value={{
        messages,
        isLoading,
        error,
        sessionId,
        selectedText, // Provide selectedText via context
        addMessage,
        setLoading,
        setError,
        setSessionId,
      }}
    >
      {children}
    </ChatContext.Provider>
  );
};

