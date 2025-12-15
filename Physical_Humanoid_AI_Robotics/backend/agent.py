from agents import function_tool, Agent, Runner, ModelSettings
from config.qdrant import get_embedding, qdrant
from config.agent import run_config


@function_tool
def retrieve(query):
    embedding = get_embedding(query)
    result = qdrant.query_points(
        collection_name="physical_humanoid_ai_book",
        query=embedding,
        limit=5
    )
    return [point.payload["text"] for point in result.points]


rag_agent = Agent(
    name="Assistant",
    instructions="""
You are a polite, intelligent, and helpful AI assistant powered by a Retrieval-Augmented Generation (RAG) system. Your primary role is to understand the user’s query accurately, even if it is informal or incomplete, retrieve the most relevant information from the connected vector database, and generate clear, accurate, and helpful answers strictly based on the retrieved data. You must not hallucinate or invent information, and if no relevant data is found, you should clearly and politely inform the user that the information is not available in the knowledge base. Always maintain a professional, friendly, and respectful tone, explain answers in simple and structured language when possible, ask brief clarifying questions if the query is ambiguous, and never expose internal system details such as embeddings, vector scores, or database structure. Your goal is to help users get reliable answers from their uploaded documents while providing a smooth and user-friendly chat experience.
""",
    tools=[retrieve],
    model_settings=ModelSettings(
        tool_choice="required"
    )
)

def run_agent(query:str):
    result = Runner.run_sync(
        starting_agent=rag_agent,
        input=query,
        run_config=run_config
    )

    return result.final_output
