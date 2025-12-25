import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import styles from './CustomChatbot.module.css'; // Import the CSS module

// Message Component
interface MessageProps {
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
  avatar: string | null; // Avatar can be null
}

const Message: React.FC<MessageProps> = ({ text, sender, timestamp, avatar }) => {
  return (
    <div className={`${styles.messageContainer} ${sender === 'user' ? styles.userMessage : styles.botMessage}`}>
      {avatar && <div className={styles.messageAvatar}>{avatar}</div>}
      <div>
        <div className={styles.messageContent}>{text}</div>
        <div className={styles.messageTimestamp}>
          {timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
        </div>
      </div>
    </div>
  );
};

// Main CustomChatbot Component
const CustomChatbot: React.FC = () => {
  const initialWelcomeMessage = {
    id: 1,
    text: "Hello! I'm your AI assistant for the 'Physical AI & Humanoid Robotics Textbook'. I can answer questions and provide information based on the book's content. What would you like to know?",
    sender: "bot" as const,
    timestamp: new Date(),
    avatar: "🤖"
  };

  const [messages, setMessages] = useState([initialWelcomeMessage]);
  const [inputText, setInputText] = useState('');
  const [isTyping, setIsTyping] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const handleSendMessage = async () => {
    if (inputText.trim() === '') return;

    const newUserMessage = {
      id: messages.length + 1,
      text: inputText,
      sender: 'user' as const,
      timestamp: new Date(),
      avatar: null // Removed user avatar emoji
    };

    setMessages((prevMessages) => [...prevMessages, newUserMessage]);
    setInputText('');
    setIsTyping(true);

    try {
      const response = await axios.post('http://localhost:8000/chat', { message: inputText });
      const botResponse = {
        id: messages.length + 2,
        text: response.data.response || "Sorry, I couldn't get a response.",
        sender: "bot" as const,
        timestamp: new Date(),
        avatar: "🤖"
      };
      setMessages((prevMessages) => [...prevMessages, botResponse]);
    } catch (error) {
      console.error("Error sending message to backend:", error);
      const errorMessage = {
        id: messages.length + 2,
        text: "Error: Could not connect to the chatbot service. Please ensure the backend is running.",
        sender: "bot" as const,
        timestamp: new Date(),
        avatar: null // Removed emoji from error message
      };
      setMessages((prevMessages) => [...prevMessages, errorMessage]);
    } finally {
      setIsTyping(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      handleSendMessage();
    }
  };

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isTyping]);

  return (
    <>
      {/* Chatbot Toggle Button */}
      <button
        onClick={() => setIsOpen(!isOpen)}
        className={styles.chatbotToggleButton} // Use the CSS module class
      >
        {isOpen ? 'Close' : 'Chat'} {/* Replaced emojis with text */}
      </button>

      {/* Chatbot Interface */}
      {isOpen && (
        <div className={styles.chatbotContainer}>
          {/* Header */}
          <div className={styles.chatbotHeader}>
            <h3>Custom Chatbot</h3>
          </div>

          {/* Main Chat Container */}
          <div className={styles.chatMainContainer}>
            {/* Welcome Banner */}
            {messages[0].id === initialWelcomeMessage.id && (
              <div className={styles.welcomeBanner}>
                <h4>{initialWelcomeMessage.sender === 'bot' ? 'Welcome!' : ''}</h4>
                <p>{initialWelcomeMessage.text}</p>
              </div>
            )}

            {/* Message Display Area */}
            {messages.slice(messages[0].id === initialWelcomeMessage.id ? 1 : 0).map((msg) => (
              <Message key={msg.id} {...msg} />
            ))}
            {isTyping && (
              <div className={styles.typingIndicator}>
                <div className={styles.messageAvatar}>🤖</div>
                <div className={styles.messageContent}>
                  <span></span><span></span><span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Section */}
          <div className={styles.inputSection}>
            <input
              type="text"
              placeholder="Type your message..."
              value={inputText}
              onChange={(e) => setInputText(e.target.value)}
              onKeyPress={handleKeyPress}
              className={styles.inputField}
            />
            <button
              onClick={handleSendMessage}
              className={styles.sendButton}
              disabled={inputText.trim() === '' || isTyping}
            >
              Send
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default CustomChatbot;
