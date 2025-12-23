import React, { useState } from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import { ChatBotProvider } from './ChatBotProvider';
import '../../css/chatbot.css'; // Import the new chatbot styles
import { MessageList } from './MessageList';
import { MessageInput } from './MessageInput';

export const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false); // Manages widget visibility

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const { control } = useChatKit({
    api: {
      async getClientSecret(existing) {
        // TODO: Replace with actual backend call to get client_secret (Part of T015, T016)
        console.log('Fetching client secret...');
        return 'sk-dummy-client-secret';
      },
    },
    // Minimal styling to integrate with Docusaurus theme
    theme: {
      '--chatkit-color-primary': 'var(--ifm-color-primary)',
      '--chatkit-border-radius': '8px',
      '--chatkit-font-family': 'var(--ifm-font-family-base)',
      '--chatkit-background-color': 'var(--ifm-background-color)',
      '--chatkit-text-color': 'var(--ifm-font-color-base)',
    },
  });

  return (
    <ChatBotProvider> {/* Keep ChatBotProvider for potential custom context or logic */}
      <div className="chat-widget-container">
        <button className="chat-toggle-button" onClick={toggleChat}>
          {isOpen ? (
            <svg
              xmlns="http://www.w3.org/2000/svg"
              width="24"
              height="24"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
            >
              <line x1="18" y1="6" x2="6" y2="18"></line>
              <line x1="6" y1="6" x2="18" y2="18"></line>
            </svg>
          ) : (
            <svg
              xmlns="http://www.w3.org/2000/svg"
              width="24"
              height="24"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
            >
              <path d="M21 11.5a8.38 8.38 0 0 1-.9 3.8 8.5 8.5 0 0 1-7.6 4.7 8.38 8.38 0 0 1-3.8-.9L3 21l1.9-5.7a8.38 8.38 0 0 1-.9-3.8 8.5 8.5 0 0 1 4.7-7.6 8.38 8.38 0 0 1 3.8-.9h.5a8.48 8.48 0 0 1 8 8v.5z"></path>
            </svg>
          )}
        </button>

        {isOpen && (
          <div className="chat-window">
            <div className="chat-header">
              <h3 style={{ margin: 0, color: 'white' }}>Chat with AI</h3>
              <button onClick={toggleChat}>X</button>
            </div>
            <div className="chat-messages-container">
              <MessageList />
            </div>
            <MessageInput />
          </div>
        )}
      </div>
    </ChatBotProvider>
  );
};

