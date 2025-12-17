from typing import Optional
from uuid import UUID

from src.models import ChatSession
from src.repositories.chat_repository import ChatRepository

class SessionManager:
    def __init__(self, chat_repository: ChatRepository):
        self.chat_repository = chat_repository

    async def find_or_create_session(self, session_id: Optional[UUID]) -> ChatSession:
        if session_id:
            session = await self.chat_repository.find_by_id(session_id)
            if session:
                return session
        
        # If no session_id is provided, or if not found, create a new one
        return await self.chat_repository.create_session()