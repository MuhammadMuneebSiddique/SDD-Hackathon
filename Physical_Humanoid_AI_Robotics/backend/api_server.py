from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import uuid, os
from agent import run_agent
from dotenv import load_dotenv


load_dotenv()

app = FastAPI(title="RAG Chatbot API", version="1.0.0")

origin = [
    "http://localhost:3000"
]

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Data models
class ChatRequest(BaseModel):
    message: str
    sessionId: Optional[str] = None

class ChatResponse(BaseModel):
    reply: str
    sources: List[str]
    sessionId: str
    error: Optional[str] = None

# In-memory session storage (in production, use a proper database)
chat_sessions = {}

@app.post("/api/chat", response_model=ChatResponse)
def chat_endpoint(request: ChatRequest):
    try:
        # Create or retrieve session
        session_id = request.sessionId or str(uuid.uuid4())
        
        reply = run_agent(request.message)

        # For now, we'll return an empty sources list
        # In a real implementation, we'd need to extract the sources from the agent's tool usage
        sources = []

        return ChatResponse(
            reply=reply,
            sources=sources,
            sessionId=session_id
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=os.getenv("PORT"))