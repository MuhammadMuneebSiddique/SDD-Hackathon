import requests
import xml.etree.ElementTree as ET
import trafilatura, os
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import cohere
from dotenv import load_dotenv

# -------------------------------------
# CONFIG
# -------------------------------------
# Your Deployment Link:


load_dotenv()
sitemap_url = os.getenv("SITEMAP_URL")
cohere_api_key = os.getenv("COHERE_API_KEY")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

SITEMAP_URL = sitemap_url
COLLECTION_NAME = "physical_humanoid_ai_book"

cohere_client = cohere.Client(cohere_api_key)
EMBED_MODEL = "embed-english-v3.0"

# Connect to Qdrant Cloud
qdrant = QdrantClient(
    url="https://35b9b7db-d5b5-4d3a-a2cc-6b1cfecafd27.europe-west3-0.gcp.cloud.qdrant.io:6333",
    api_key=qdrant_api_key 
)

# -------------------------------------
# Step 1 — Extract URLs from sitemap
# -------------------------------------
def get_all_urls(sitemap_url):
    xml = requests.get(sitemap_url).text
    root = ET.fromstring(xml)

    urls = []
    for child in root:
        loc_tag = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
        if loc_tag is not None:
            urls.append(loc_tag.text)

    print("\nFOUND URLS:")
    for u in urls:
        print(" -", u)

    return urls


# -------------------------------------
# Step 2 — Download page + extract text
# -------------------------------------
def extract_text_from_url(url):
    html = requests.get(url).text
    text = trafilatura.extract(html)

    if not text:
        print("[WARNING] No text extracted from:", url)

    return text


# -------------------------------------
# Step 3 — Chunk the text
# -------------------------------------
def chunk_text(text, max_chars=1200):
    text = text.strip()                 # remove leading/trailing spaces
    chunks = []

    while len(text) > max_chars:
        # Try to split on sentence boundary
        split_pos = text[:max_chars].rfind(". ")

        # If no sentence found, fallback to max_chars
        if split_pos == -1:
            split_pos = max_chars

        # Safety: avoid infinite loop if split_pos == 0
        if split_pos == 0:
            split_pos = max_chars

        # Extract chunk
        chunk = text[:split_pos].strip()

        # Safety: skip empty chunks
        if chunk:
            chunks.append(chunk)

        # Move forward
        text = text[split_pos:].strip()

        # Extra safety: if no progress, break
        if len(text) < max_chars:
            break

    # Add last remaining text
    if text.strip():
        chunks.append(text.strip())

    return chunks



# -------------------------------------
# Step 4 — Create embedding
# -------------------------------------
def embed(text):
    response = cohere_client.embed(
        model=EMBED_MODEL,
        input_type="search_query",  # Use search_query for queries
        texts=[text],
    )
    return response.embeddings[0]  # Return the first embedding


# -------------------------------------
# Step 5 — Store in Qdrant
# -------------------------------------
def create_collection():
    print("\nCreating Qdrant collection...")
    qdrant.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
        size=1024,        # Cohere embed-english-v3.0 dimension
        distance=Distance.COSINE
        )
    )

def save_chunk_to_qdrant(chunk, chunk_id, url):
    vector = embed(chunk)

    qdrant.upsert(
        collection_name=COLLECTION_NAME,
        points=[
            PointStruct(
                id=chunk_id,
                vector=vector,
                payload={
                    "url": url,
                    "text": chunk,
                    "chunk_id": chunk_id
                }
            )
        ]
    )


# -------------------------------------
# MAIN INGESTION PIPELINE
# -------------------------------------
def ingest_book():
    urls = get_all_urls(SITEMAP_URL)

    create_collection()

    global_id = 1

    for url in urls:
        print("\nProcessing:", url)
        text = extract_text_from_url(url)

        if not text:
            continue

        chunks = chunk_text(text)

        for ch in chunks:
            save_chunk_to_qdrant(ch, global_id, url)
            print(f"Saved chunk {global_id}")
            global_id += 1

    print("\n✔️ Ingestion completed!")
    print("Total chunks stored:", global_id - 1)


if __name__ == "__main__":
    ingest_book()