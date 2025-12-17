import os
import google.generativeai as genai
from dotenv import load_dotenv # Import load_dotenv
from src.models import Message # Import Message model
from src.exceptions import GeminiAPIException # Import GeminiAPIException
from google.api_core.exceptions import GoogleAPIError # Import base Google API exception

class GeminiClient:
    def __init__(self):
        load_dotenv() # Load environment variables here
        genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
        self.model = genai.GenerativeModel('gemini-2.5-flash')

    def construct_system_prompt(self) -> str:
        return "You are a helpful AI assistant. You answer questions concisely and accurately."

    async def generate_response(self, system_prompt: str, message: str, conversation_history: list[Message]) -> str:
        # Convert conversation history to Gemini format
        formatted_history = []
        for msg in conversation_history:
            role = "model" if msg.role == "assistant" else msg.role
            formatted_history.append({"role": role, "parts": [msg.content]})
        
        # Start chat with history
        chat = self.model.start_chat(history=formatted_history)
        
        try:
            # Send system prompt and new message
            response = await chat.send_message_async(system_prompt + "\n" + message)
            return response.text
        except GoogleAPIError as e:
            raise GeminiAPIException(detail=str(e))