import React, { useState } from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import { ChatBotProvider } from './ChatBotProvider';

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
          {isOpen ? '−' : '+'}
        </button>

        {isOpen && (
          <div className="chat-window">
            <div className="chat-header">
              <h3 style={{ margin: 0, color: 'white' }}>Chat with AI</h3>
              <button onClick={toggleChat}>X</button>
            </div>
            <div style={{ flexGrow: 1, overflowY: 'auto' }}>
              <ChatKit control={control} />
            </div>
          </div>
        )}
      </div>
    </ChatBotProvider>
  );
};
