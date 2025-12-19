import os
import google.generativeai as genai
from dotenv import load_dotenv # Import load_dotenv
from src.models import Message # Import Message model
from src.exceptions import GeminiAPIException # Import GeminiAPIException
from google.api_core.exceptions import GoogleAPIError # Import base Google API exception

class GeminiClient:
    """
    Client for interacting with the Google Gemini API.
    Handles initialization, system prompt construction, and response generation.
    """
    def __init__(self):
        """
        Initializes the GeminiClient, loads environment variables, and configures
        the Gemini API with the provided API key.
        """
        load_dotenv() # Load environment variables here
        genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
        self.model = genai.GenerativeModel('gemini-2.5-flash')

    def construct_system_prompt(self) -> str:
        """
        Constructs the system prompt for the AI persona.

        Returns:
            str: The system prompt string.
        """
        return "You are a helpful AI assistant. You answer questions concisely and accurately."

    async def generate_response(self, system_prompt: str, message: str, conversation_history: list[Message]) -> str:
        """
        Generates a response from the Gemini model based on the system prompt, current message,
        and conversation history.

        Args:
            system_prompt (str): The initial system prompt defining the AI's persona.
            message (str): The current message from the user.
            conversation_history (list[Message]): A list of previous messages in the conversation.

        Returns:
            str: The AI's generated response.

        Raises:
            GeminiAPIException: If an error occurs during the Gemini API call.
        """
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