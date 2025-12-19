import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './Chatbot.module.css';

interface Message {
  text: string;
  sender: 'user' | 'bot';
}

const Chatbot: React.FC = () => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState<string>('');

  const handleSendMessage = () => {
    if (input.trim()) {
      const newUserMessage: Message = { text: input, sender: 'user' };
      setMessages((prevMessages) => [...prevMessages, newUserMessage]);
      setInput('');
      // Simulate bot response
      setTimeout(() => {
        const botResponse: Message = { text: `Echo: ${input}`, sender: 'bot' };
        setMessages((prevMessages) => [...prevMessages, botResponse]);
      }, 500);
    }
  };

  return (
    <div className={styles.chatbotContainer}>
      <div className={styles.messagesContainer}>
        {messages.map((msg, index) => (
          <div key={index} className={clsx(styles.message, styles[msg.sender])}>
            {msg.text}
          </div>
        ))}
      </div>
      <div className={styles.inputContainer}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyPress={(e) => e.key === 'Enter' && handleSendMessage()}
          placeholder="Ask me anything..."
        />
        <button onClick={handleSendMessage}>Send</button>
      </div>
    </div>
  );
};

export default Chatbot;
