import os
from typing import List
import google.generativeai as genai
from src.models import Message
from src.exceptions import GeminiAPIException

class GeminiClient:
    def __init__(self):
        # Configure Gemini API key for chat completion
        genai.configure(api_key=os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY"))
        self.model = genai.GenerativeModel("gemini-pro") # Using gemini-pro for chat completion

    def construct_system_prompt(self) -> str:
        """
        Constructs the initial system prompt for the chat model.
        """
        return (
            "You are an AI assistant for the 'Physical AI & Humanoid Robotics' textbook. "
            "Your primary goal is to provide accurate and helpful information based on the "
            "provided book content. If you cannot find the answer in the provided context, "
            "state that you don't have enough information to answer."
        )

    async def generate_response(self, system_prompt: str, user_message: str, conversation_history: List[Message]) -> str:
        """
        Generates a response from the Gemini chat model.
        """
        try:
            # Format conversation history for Gemini API
            # Gemini API expects alternating user and model roles.
            # Convert our Message objects to dicts for the history.
            history_for_gemini = []
            for msg in conversation_history:
                if msg.role == "user":
                    history_for_gemini.append({"role": "user", "parts": [msg.content]})
                elif msg.role == "assistant":
                    history_for_gemini.append({"role": "model", "parts": [msg.content]})
            
            # The initial prompt in main.py already combines system prompt + RAG context + user message.
            # So, the 'user_message' here is already the full RAG-augmented prompt.
            # The conversation history should precede this.

            # Gemini's chat.send_message takes turns
            # The 'start_chat' method initializes with history
            chat = self.model.start_chat(history=history_for_gemini)
            response = await chat.send_message_async(user_message)
            
            return response.text
        except Exception as e:
            raise GeminiAPIException(detail=f"Error generating response from Gemini: {e}")
