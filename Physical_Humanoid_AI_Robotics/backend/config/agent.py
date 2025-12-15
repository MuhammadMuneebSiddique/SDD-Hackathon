from agents import AsyncOpenAI, OpenAIChatCompletionsModel, RunConfig
from dotenv import load_dotenv
import os



load_dotenv()
openrouter_api_key = os.getenv("OPENROUTER_API_KEY")
client = AsyncOpenAI(
    api_key=openrouter_api_key,
    base_url="https://openrouter.ai/api/v1"
)

model = OpenAIChatCompletionsModel(
    model="google/gemini-2.5-flash-lite",
    openai_client=client
)

run_config = RunConfig(
    model=model,
    model_provider=client,
    tracing_disabled=True
)