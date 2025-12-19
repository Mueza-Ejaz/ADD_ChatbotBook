// frontend/src/components/Chatbot/ChatBotProvider.test.tsx
import React, { useContext } from 'react';
import { render, screen, act } from '@testing-library/react';
import '@testing-library/jest-dom';
import { ChatBotProvider, ChatContext } from './ChatBotProvider';

// Mock crypto.randomUUID
global.crypto = {
  randomUUID: () => 'mock-uuid',
} as any;

describe('ChatBotProvider', () => {
  beforeEach(() => {
    sessionStorage.clear(); // Clear sessionStorage before each test
  });

  const TestConsumer: React.FC = () => {
    const context = useContext(ChatContext);
    if (!context) {
      throw new Error('TestConsumer must be used within a ChatBotProvider');
    }
    const { messages, isLoading, error, sessionId, selectedText, addMessage, setLoading, setError, setSessionId } = context;

    return (
      <div>
        <div data-testid="messages-count">{messages.length}</div>
        <div data-testid="is-loading">{isLoading.toString()}</div>
        <div data-testid="error">{error}</div>
        <div data-testid="session-id">{sessionId}</div>
        <div data-testid="selected-text">{selectedText}</div>
        <button onClick={() => addMessage({ id: '1', sender: 'user', content: 'hello', timestamp: new Date().toISOString() })}>Add Message</button>
        <button onClick={() => setLoading(true)}>Set Loading</button>
        <button onClick={() => setError('Test Error')}>Set Error</button>
        <button onClick={() => setSessionId('new-session')}>Set Session Id</button>
      </div>
    );
  };

  test('provides initial context values', () => {
    render(
      <ChatBotProvider>
        <TestConsumer />
      </ChatBotProvider>
    );

    expect(screen.getByTestId('messages-count')).toHaveTextContent('0');
    expect(screen.getByTestId('is-loading')).toHaveTextContent('false');
    expect(screen.getByTestId('error')).toBeEmptyDOMElement();
    expect(screen.getByTestId('session-id')).toHaveTextContent('mock-uuid');
    expect(screen.getByTestId('selected-text')).toBeEmptyDOMElement();
  });

  test('adds message correctly', () => {
    render(
      <ChatBotProvider>
        <TestConsumer />
      </ChatBotProvider>
    );

    act(() => {
      screen.getByText('Add Message').click();
    });

    expect(screen.getByTestId('messages-count')).toHaveTextContent('1');
  });

  test('sets loading state correctly', () => {
    render(
      <ChatBotProvider>
        <TestConsumer />
      </ChatBotProvider>
    );

    act(() => {
      screen.getByText('Set Loading').click();
    });

    expect(screen.getByTestId('is-loading')).toHaveTextContent('true');
  });

  test('sets error state correctly', () => {
    render(
      <ChatBotProvider>
        <TestConsumer />
      </ChatBotProvider>
    );

    act(() => {
      screen.getByText('Set Error').click();
    });

    expect(screen.getByTestId('error')).toHaveTextContent('Test Error');
  });

  test('persists sessionId to sessionStorage', () => {
    render(
      <ChatBotProvider>
        <TestConsumer />
      </ChatBotProvider>
    );

    expect(sessionStorage.getItem('chatSessionId')).toBe('mock-uuid');
  });

  test('loads sessionId from sessionStorage if already present', () => {
    sessionStorage.setItem('chatSessionId', 'existing-session-id');
    render(
      <ChatBotProvider>
        <TestConsumer />
      </ChatBotProvider>
    );
    expect(screen.getByTestId('session-id')).toHaveTextContent('existing-session-id');
  });

  test('persists messages to sessionStorage', () => {
    render(
      <ChatBotProvider>
        <TestConsumer />
      </ChatBotProvider>
    );

    act(() => {
      screen.getByText('Add Message').click();
    });

    expect(JSON.parse(sessionStorage.getItem('chatMessages') || '[]').length).toBe(1);
  });

  test('loads messages from sessionStorage if already present', () => {
    const mockMessages = [{ id: '2', sender: 'ai', content: 'hi', timestamp: new Date().toISOString() }];
    sessionStorage.setItem('chatMessages', JSON.stringify(mockMessages));
    render(
      <ChatBotProvider>
        <TestConsumer />
      </ChatBotProvider>
    );
    expect(screen.getByTestId('messages-count')).toHaveTextContent('1');
  });
});
