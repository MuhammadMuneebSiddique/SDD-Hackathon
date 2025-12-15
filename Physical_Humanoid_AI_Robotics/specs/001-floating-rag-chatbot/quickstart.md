# Quickstart Guide: Floating RAG Chatbot for Docusaurus Book

## Overview
This guide explains how to set up and use the floating RAG chatbot that integrates with your Docusaurus book website.

## Prerequisites
- Node.js 18+ for Docusaurus
- Python 3.13+ for backend
- Access to Cohere API key
- Access to Qdrant Cloud API key
- (Optional) OpenRouter API key for Gemini integration

## Backend Setup

### 1. Environment Configuration
```bash
cd backend/
cp .env.example .env  # if you have an example file
# Add your API keys to .env:
# COHERE_API_KEY=your_cohere_key
# QDRANT_API_KEY=your_qdrant_key
# GEMINI_API_KEY=your_gemini_key
```

### 2. Install Dependencies
```bash
uv sync  # or use pip install -r requirements.txt if available
```

### 3. Start the Backend API Server
```bash
# You'll need to create the FastAPI server (this will be generated as part of implementation)
uvicorn main:app --reload
```

## Frontend Integration

### 1. Install Components
The following components will be added to your Docusaurus project:
- `src/components/FloatingChatButton.jsx` - The floating button that appears on all pages
- `src/components/ChatPopup.jsx` - The popup chat window UI

### 2. Integrate into Layout
The floating chat button will be injected into:
- `src/theme/Layout.js` - Added globally to appear on all pages

## Usage

### 1. Building the Docusaurus Site
```bash
cd my-docusaurus-project/
npm install
npm run build
npm run serve  # to test locally
```

### 2. Using the Chatbot
1. Navigate to any page in your Docusaurus book
2. Click the floating chat button (appears as a circular icon in the bottom-right corner)
3. Type your question about the book content in the chat input
4. The RAG system will retrieve relevant information from the book and generate a response

## API Endpoints

### Chat Endpoint
- **URL**: `/api/chat`
- **Method**: POST
- **Request Body**:
  ```json
  {
    "message": "Your question here",
    "sessionId": "optional session id"
  }
  ```
- **Response**:
  ```json
  {
    "reply": "Generated response",
    "sources": ["source1", "source2"],
    "sessionId": "session identifier"
  }
  ```

## Development

### Running in Development Mode
1. Start the backend API server
2. Run Docusaurus in development mode: `npm run start`
3. The floating chatbot will be available on all pages

### Testing the Integration
1. Open your Docusaurus site
2. Verify the floating button appears on all pages
3. Test chat functionality with various questions about book content
4. Verify responses are contextually relevant to the book content