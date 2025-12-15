# Data Model: Floating RAG Chatbot for Docusaurus Book

## Entity: ChatMessage

**Description**: Represents a single message in the chat conversation

**Fields**:
- `id`: String - Unique identifier for the message
- `sender`: String - Either "user" or "bot"
- `content`: String - The text content of the message
- `timestamp`: DateTime - When the message was created
- `status`: String - "sending", "sent", "received", "error" (for UI state management)

**Relationships**:
- Belongs to one ChatSession
- Part of an ordered sequence within a ChatSession

## Entity: ChatSession

**Description**: Represents a single chat conversation session

**Fields**:
- `id`: String - Unique identifier for the session
- `messages`: Array<ChatMessage> - Ordered list of messages in the conversation
- `createdAt`: DateTime - When the session was created
- `lastActive`: DateTime - When the last message was sent/received

**Relationships**:
- Contains many ChatMessage entities
- Belongs to one user session (frontend local state)

## Entity: ChatRequest

**Description**: Represents a request from the frontend to the backend API

**Fields**:
- `message`: String - The user's message/query
- `sessionId`: String - (Optional) Session identifier for maintaining conversation context
- `context`: Object - (Optional) Additional context information for the RAG system

## Entity: ChatResponse

**Description**: Represents a response from the backend API to the frontend

**Fields**:
- `reply`: String - The bot's response to the user's query
- `sources`: Array<String> - List of source documents/references used to generate the response
- `sessionId`: String - Session identifier for maintaining conversation context
- `error`: String - (Optional) Error message if the request failed

## Validation Rules

1. **ChatMessage Content**:
   - Must not be empty or contain only whitespace
   - Maximum length: 10000 characters (prevent extremely long messages)

2. **ChatSession Messages**:
   - Must maintain chronological order
   - Maximum 100 messages per session (for performance)
   - Messages cannot be modified after creation (append-only)

3. **ChatRequest**:
   - Message field must be present and not empty
   - SessionId must be a valid UUID format if provided

## State Transitions

### ChatMessage Status Transitions
```
sending → sent → received
sending → error (if network failure)
```

### ChatSession Lifecycle
```
created → active → (session timeout or user closes)
```

## API Contract Summary

### Frontend → Backend
- **Request**: POST `/api/chat` with ChatRequest payload
- **Response**: 200 OK with ChatResponse payload
- **Error**: 400 Bad Request, 500 Internal Server Error

### Backend → Frontend
- **Response Format**: JSON with reply, sources, and session information
- **Streaming Option**: Server-sent events for real-time response generation (optional enhancement)