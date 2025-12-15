from cohere import Client
from qdrant_client import QdrantClient
from dotenv import load_dotenv
import os

load_dotenv()
cohere_api_key = os.getenv("COHERE_API_KEY")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
cohere_client = Client(cohere_api_key)


# Connect to Qdrant
qdrant = QdrantClient(
    url="https://35b9b7db-d5b5-4d3a-a2cc-6b1cfecafd27.europe-west3-0.gcp.cloud.qdrant.io:6333",
    api_key=qdrant_api_key
)

def get_embedding(text):
    """Get embedding vector from Cohere Embed v3"""
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",  # Use search_query for queries
        texts=[text],
    )
    return response.embeddings[0]  # Return the first embedding
