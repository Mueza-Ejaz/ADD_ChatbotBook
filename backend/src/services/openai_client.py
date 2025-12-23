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
        Constructs the system prompt for the AI agent behavior.
        """
        return (
            "You are an intelligent, helpful, and reliable AI assistant.\n\n"
            "Your task is to answer user questions as accurately and clearly as possible.\n\n"
            "IMPORTANT BEHAVIOR RULES:\n"
            "1. Always first use the provided context, retrieved content, or database knowledge if it contains relevant information.\n"
            "2. If the answer is clearly available in the provided context, answer strictly based on that information.\n"
            "3. If the provided context does NOT contain the answer, then answer using your own general knowledge and reasoning.\n"
            "4. Never say phrases like 'the answer is not in the provided text' or 'the context does not mention this'.\n"
            "5. Never mention internal processes such as database lookup, retrieval, or reasoning steps.\n\n"
            "ADDITIONAL GUIDELINES:\n"
            "- If the user greets (e.g., 'hi', 'hello'), respond politely and naturally.\n"
            "- If the question is unclear, politely ask for clarification.\n"
            "- Keep responses concise, friendly, and easy to understand.\n"
            "- Use simple language unless the user asks for technical detail.\n\n"
            "Your goal is to always be helpful, even when information is missing."
        )
    


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

            