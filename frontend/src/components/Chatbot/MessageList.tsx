import React, { useContext } from 'react';
import { ChatContext } from './ChatBotProvider';

export const MessageList: React.FC = () => {
  const context = useContext(ChatContext);

  if (!context) {
    throw new Error('MessageList must be used within a ChatBotProvider');
  }

  const { messages, isLoading, error } = context;

  return (
    <div className="message-list">
      {messages.map((message) => (
        <div key={message.id} className={`message ${message.sender}`}>
          <p>{message.content}</p>
          {message.isContextuallyDriven && <span className="context-indicator"> (Contextual)</span>}
          <span className="timestamp">{new Date(message.timestamp).toLocaleTimeString()}</span>
        </div>
      ))}
      {isLoading && <div className="typing-indicator">AI is typing...</div>}
      {error && <div className="chat-error">{error}</div>}
    </div>
  );
};
