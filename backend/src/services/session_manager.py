from typing import Optional
from uuid import UUID

from src.models import ChatSession
from src.repositories.chat_repository import ChatRepository

class SessionManager:
    """
    Manages chat session creation and retrieval.
    """
    def __init__(self, chat_repository: ChatRepository):
        """
        Initializes the SessionManager with a ChatRepository instance.

        Args:
            chat_repository (ChatRepository): An instance of ChatRepository for database operations.
        """
        self.chat_repository = chat_repository

    async def find_or_create_session(self, session_id: Optional[UUID]) -> ChatSession:
        """
        Finds an existing chat session or creates a new one if not found or not provided.

        Args:
            session_id (Optional[UUID]): The ID of the session to find. If None, a new session is created.

        Returns:
            ChatSession: The found or newly created chat session.
        """
        if session_id:
            session = await self.chat_repository.find_by_id(session_id)
            if session:
                return session
        
        # If no session_id is provided, or if not found, create a new one
        return await self.chat_repository.create_session()