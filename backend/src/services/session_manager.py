from typing import Optional
from uuid import UUID, uuid4
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy import exc # Import for handling exceptions
from src.models import ChatSession
from src.exceptions import ChatSessionNotFoundException # Import from exceptions.py

class SessionManager:
    def __init__(self, db_session: AsyncSession):
        self.db_session = db_session

    async def find_or_create_session(self, session_id: Optional[UUID]) -> ChatSession:
        if session_id:
            # Try to find the session
            result = await self.db_session.execute(
                select(ChatSession).filter(ChatSession.id == session_id)
            )
            chat_session = result.scalars().first()
            if chat_session:
                return chat_session
            else:
                # If a session_id was provided but not found, raise an exception
                raise ChatSessionNotFoundException(str(session_id)) # Convert UUID to string for exception
        
        # If no session_id provided or not found (and no exception raised), create a new one
        new_session = ChatSession(id=uuid4()) # Generate new UUID for new session
        self.db_session.add(new_session)
        await self.db_session.commit()
        await self.db_session.refresh(new_session)
        return new_session
