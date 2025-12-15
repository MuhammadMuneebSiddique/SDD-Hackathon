# RAG Chatbot Backend API

This backend provides a FastAPI server that exposes the RAG (Retrieval-Augmented Generation) functionality for the Docusaurus chatbot interface.

## Setup

1. Install dependencies:
   ```bash
   pip install fastapi uvicorn python-dotenv
   ```

2. Set up environment variables:
   Create a `.env` file with the following variables:
   ```
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_API_KEY=your_qdrant_api_key
   GEMINI_API_KEY=your_gemini_api_key
   ```

## Running the Server

```bash
uvicorn api_server:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000`.

## API Endpoints

- `POST /api/chat` - Send a message and receive a RAG-enhanced response
- `GET /health` - Health check endpoint

## Frontend Integration

The frontend chat component should send requests to `http://localhost:8000/api/chat` (or your deployed backend URL).