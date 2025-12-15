import React, { useState, useRef, useEffect } from 'react';
import './chatbot.css';
import api from '@site/src/API/api';

const Chatbot = ({
  botName = 'AI Assistant',
  initialMessages = [],
  onSendMessage = null,
  theme = 'primary',
  placeholder = 'Type your message...'
}) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    ...initialMessages,
    { id: 1, text: 'Hello! How can I help you today?', sender: 'bot', timestamp: new Date() }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      setTimeout(() => {
        inputRef.current.focus();
      }, 100);
    }
  }, [isOpen]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const closeChat = () => {
    setIsOpen(false);
  };

  const handleSend = async () => {
    if (!inputValue.trim()) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // If custom send handler provided, use it
      if (onSendMessage) {

        const response = await onSendMessage(inputValue);
        const botMessage = {
          id: Date.now() + 1,
          text: response,
          sender: 'bot',
          timestamp: new Date()
        };
        setMessages(prev => [...prev, botMessage]);
        setIsLoading(false);
      } else {
        
        // Simulate bot response for demo
        setTimeout(() => {

          const botMessage = {
            id: Date.now() + 1,
            text: `I received your message: "${inputValue}". This is a simulated response.`,
            sender: 'bot',
            timestamp: new Date()
          };
          setMessages(prev => [...prev, botMessage]);
          setIsLoading(false);
        }, 1000);
      }
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error. Please try again.',
        sender: 'bot',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const formatTime = (date) => {
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <>
      {/* Floating Chat Button */}
      <button
        className={`floating-chatbot-button ${isOpen ? 'open' : ''}`}
        onClick={toggleChat}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
        title={isOpen ? 'Close chat' : 'Open chat'}
      >
        💬
      </button>

      {/* Chat Panel */}
      <div className={`chatbot-panel ${isOpen ? 'open' : ''} theme-${theme}`}>
        {/* Chat Header */}
        <div className="chat-header">
          <h3 className="chat-title">{botName}</h3>
          <button
            className="close-button"
            onClick={closeChat}
            aria-label="Close chat"
          >
            ×
          </button>
        </div>

        {/* Messages Container */}
        <div className="messages-container">
          {messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.sender}`}
              role="log"
              aria-live="polite"
            >
              <div className="message-text">{message.text}</div>
              <div className="message-time" style={{ fontSize: '10px', opacity: 0.7, marginTop: '4px' }}>
                {formatTime(message.timestamp)}
              </div>
            </div>
          ))}
          {isLoading && (
            <div className="message bot">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        {/* Input Area */}
        <div className="input-area">
          <input
            ref={inputRef}
            type="text"
            className="message-input"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder={placeholder}
            disabled={isLoading}
            aria-label="Type your message"
          />
          <button
            className="send-button"
            onClick={handleSend}
            disabled={!inputValue.trim() || isLoading}
            aria-label="Send message"
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M22 2L11 13M22 2l-7 20-4-9-9-4 20-7z" />
            </svg>
          </button>
        </div>
      </div>
    </>
  );
};

export default Chatbot;