# Research: Floating RAG Chatbot Integration

## Backend Analysis

### Current Backend Structure
- **Location**: `./backend/`
- **Technology**: Python with Cohere embeddings, Qdrant vector database
- **Current functionality**: Document ingestion pipeline and AI agent with retrieval capabilities

### API Endpoints Assessment
- **Findings**: No existing FastAPI server with REST endpoints for frontend communication
- **Current agent**: Uses `agents` library with retrieval tool but no exposed API endpoints
- **Missing**: HTTP endpoints for frontend chatbot to communicate with the backend

### Required Backend API Development
- Need to create FastAPI server with endpoints for chat functionality
- Need to expose the existing RAG agent functionality through API endpoints
- Required endpoints:
  - POST `/api/chat` - Process user queries and return RAG responses
  - POST `/api/chat/stream` - Optional streaming endpoint for real-time responses

### CORS Configuration
- Will need to configure CORS to allow requests from Docusaurus frontend
- Should allow requests from localhost:3000 (or appropriate Docusaurus port)

## Frontend Integration Context

### Docusaurus Project Location
- **Assumed location**: `./my-docusaurus-project/` (as mentioned in spec)
- **Integration point**: `/src/theme/Layout.js` to add floating button globally
- **Component locations**: `/src/components/` for chat UI components

### UI Component Requirements
- **FloatingChatButton.jsx**: Circular button with open/close state
- **ChatPopup.jsx**: Popup window with message history and input
- **Styling**: TailwindCSS for responsive, clean design
- **Behavior**: Overlay that doesn't affect underlying book content

## Technology Stack Alignment

### Frontend Technologies
- React (for component-based architecture)
- Docusaurus v3 (static site generation)
- TailwindCSS (styling and responsive design)

### Backend Technologies
- FastAPI (REST API framework)
- Cohere (embeddings)
- Qdrant Cloud (vector database)
- Python 3.13+ (backend runtime)

## Implementation Approach

### Recommended Architecture
1. **Backend Enhancement**: Add FastAPI endpoints to existing RAG functionality
2. **Frontend Development**: Create React components for chat UI
3. **Integration**: Add floating button to Docusaurus layout
4. **Connection**: Frontend communicates with backend via API calls

### Decision: Backend API
- **Decision**: Need to create FastAPI endpoints to expose existing RAG agent functionality
- **Rationale**: Current backend has RAG capabilities but no API endpoints for frontend communication
- **Alternatives considered**:
  - Modify existing main.py to include FastAPI (rejected - would complicate ingestion pipeline)
  - Create separate API server (selected - cleaner separation of concerns)

### Decision: Component Structure
- **Decision**: Create two main components - FloatingChatButton and ChatPopup
- **Rationale**: Separates the floating button logic from the chat interface logic
- **Implementation**: Both components will be stateful with React hooks

### Decision: State Management
- **Decision**: Use React useState and useEffect hooks for component state
- **Rationale**: Simple state management is sufficient for this feature
- **Implementation**: Local component state for UI elements, session state for chat history