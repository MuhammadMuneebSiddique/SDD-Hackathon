import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import Chatbot from '../components/ChatBot UI/Chatbot';
import api from '../API/api';

// Custom Layout that wraps the original layout with the floating chat button
const Layout = (props) => {
  const message_send = async (query) => {
    const response = await api.post("/api/chat", {message: query, sessionId: "string"});
    return await response.data.reply;
  };

  return (
    <>
      <OriginalLayout {...props} />
      <Chatbot
        botName="AI Assistant"
        theme="primary"
        placeholder="Ask me anything about Physical AI & Robotics..."
        onSendMessage={message_send}
      />
    </>
  );
};

export default Layout;