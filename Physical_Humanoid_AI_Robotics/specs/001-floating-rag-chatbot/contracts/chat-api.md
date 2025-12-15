# OpenAPI Contract: RAG Chatbot API

## Chat API

### POST /api/chat
Process a user's query and return a RAG-enhanced response.

**Request**:
```json
{
  "message": "string",
  "sessionId": "string (optional)"
}
```

**Response** (200 OK):
```json
{
  "reply": "string",
  "sources": ["string"],
  "sessionId": "string"
}
```

**Error Response** (400 Bad Request):
```json
{
  "error": "string"
}
```

### POST /api/chat/stream (Optional Enhancement)
Stream a response as it's being generated.

**Request**:
```json
{
  "message": "string",
  "sessionId": "string (optional)"
}
```

**Response**: Server-sent events with response chunks

## Backend Architecture

The backend needs to be enhanced to expose the existing RAG agent functionality through these API endpoints. The implementation will:

1. Accept user queries via the API
2. Use the existing retrieval tool to find relevant book content
3. Generate responses based on the retrieved content
4. Return the response with source references