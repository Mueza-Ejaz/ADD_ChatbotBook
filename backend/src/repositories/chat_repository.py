from typing import List, Optional
from uuid import UUID
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, desc

from src.models import ChatSession, Message

class ChatRepository:
    """
    Manages database operations for chat sessions and messages.
    """
    def __init__(self, session: AsyncSession):
        """
        Initializes the ChatRepository with an asynchronous database session.

        Args:
            session (AsyncSession): The SQLAlchemy asynchronous session.
        """
        self.session = session

    async def find_by_id(self, session_id: UUID) -> Optional[ChatSession]:
        """
        Finds a chat session by its ID.

        Args:
            session_id (UUID): The unique identifier of the chat session.

        Returns:
            Optional[ChatSession]: The found chat session, or None if not found.
        """
        result = await self.session.execute(
            select(ChatSession).where(ChatSession.id == session_id)
        )
        return result.scalars().first()

    async def create_session(self) -> ChatSession:
        """
        Creates and persists a new chat session in the database.

        Returns:
            ChatSession: The newly created chat session.
        """
        chat_session = ChatSession()
        self.session.add(chat_session)
        await self.session.commit()
        await self.session.refresh(chat_session)
        return chat_session

    async def create_message(self, session_id: UUID, role: str, content: str) -> Message:
        """
        Creates and persists a new message within a specified chat session.

        Args:
            session_id (UUID): The ID of the chat session to which the message belongs.
            role (str): The role of the message sender (e.g., "user", "assistant").
            content (str): The text content of the message.

        Returns:
            Message: The newly created message.
        """
        message = Message(session_id=session_id, role=role, content=content)
        self.session.add(message)
        await self.session.commit()
        await self.session.refresh(message)
        return message

    async def get_last_n_messages_for_session(self, session_id: UUID, n: int = 10) -> List[Message]:
        """
        Retrieves the last 'n' messages for a given chat session, ordered by creation time.

        Args:
            session_id (UUID): The ID of the chat session.
            n (int): The maximum number of messages to retrieve. Defaults to 10.

        Returns:
            List[Message]: A list of message objects, ordered from oldest to newest.
        """
        result = await self.session.execute(
            select(Message)
            .where(Message.session_id == session_id)
            .order_by(desc(Message.created_at))
            .limit(n)
        )
        return list(result.scalars().all())