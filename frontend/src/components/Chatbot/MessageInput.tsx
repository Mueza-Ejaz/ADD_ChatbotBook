import React, { useState, useContext, useEffect } from 'react';
import { ChatContext } from './ChatBotProvider';
import { sendMessage as sendChatMessage } from '../../services/chatService';

export const MessageInput: React.FC = () => {
  const [inputText, setInputText] = useState('');
  const context = useContext(ChatContext);

  if (!context) {
    throw new Error('MessageInput must be used within a ChatBotProvider');
  }

  const { isLoading, addMessage, setLoading, setError, sessionId, setSessionId, selectedText } = context;

  const [currentSelectedText, setCurrentSelectedText] = useState<string>('');

  // Update currentSelectedText when context's selectedText changes, but only if it's not already used
  useEffect(() => {
    if (selectedText && selectedText !== currentSelectedText) {
      setCurrentSelectedText(selectedText);
    }
  }, [selectedText, currentSelectedText]);

  const handleClearSelectedText = () => {
    setCurrentSelectedText('');
  };

  const handleSendMessage = async () => {
    if (inputText.trim() === '' && currentSelectedText.trim() === '') return; // Don't send empty messages

    const userMessage = {
      id: Date.now().toString(),
      sender: 'user' as 'user',
      content: inputText,
      timestamp: new Date().toISOString(),
      // Optionally show selectedText in user message as well
      isContextuallyDriven: !!currentSelectedText,
    };
    addMessage(userMessage);
    setInputText('');
    setLoading(true);
    setError(null);

    try {
      const aiResponse = await sendChatMessage(inputText, sessionId || undefined, currentSelectedText || undefined);

      addMessage({
        id: Date.now().toString() + '-ai',
        sender: 'ai',
        content: aiResponse.response,
        timestamp: aiResponse.timestamp,
        // The backend should indicate if it used selected_text, for now, we'll assume it did if sent
        isContextuallyDriven: !!currentSelectedText,
      });

      if (aiResponse.session_id && aiResponse.session_id !== sessionId) {
        setSessionId(aiResponse.session_id);
      }
      handleClearSelectedText(); // Clear selected text after sending message

    } catch (err: any) {
      setError(err.message || 'Failed to send message.');
      console.error('Error sending message:', err);
    } finally {
      setLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <div className="message-input-container">
      {currentSelectedText && (
        <div className="selected-text-indicator">
          <span className="selected-text-content">Context: "{currentSelectedText}"</span>
          <button onClick={handleClearSelectedText} className="clear-selected-text-button">x</button>
        </div>
      )}
      <div className="message-input">
        <input
          type="text"
          value={inputText}
          onChange={(e) => setInputText(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder={currentSelectedText ? "Ask about selected text or type a new message..." : "Type your message or highlight text on the page..."}
          disabled={isLoading}
        />
        <button onClick={handleSendMessage} disabled={isLoading}>
          Send
        </button>
      </div>
    </div>
  );
};
