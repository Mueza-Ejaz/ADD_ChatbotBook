from typing import List
from uuid import UUID
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy import desc
from src.models import Message, ChatSession # Assuming ChatSession is needed for relationship but not directly used here
from src.exceptions import DatabaseException # Import custom exception

class MessageRepository:
    def __init__(self, db_session: AsyncSession):
        self.db_session = db_session

    async def save_user_message(self, session_id: UUID, message_content: str) -> Message:
        try:
            user_message = Message(session_id=session_id, role="user", content=message_content)
            self.db_session.add(user_message)
            await self.db_session.commit()
            await self.db_session.refresh(user_message)
            return user_message
        except Exception as e:
            await self.db_session.rollback()
            raise DatabaseException(detail=f"Failed to save user message: {e}")

    async def save_ai_message(self, session_id: UUID, message_content: str) -> Message:
        try:
            ai_message = Message(session_id=session_id, role="assistant", content=message_content)
            self.db_session.add(ai_message)
            await self.db_session.commit()
            await self.db_session.refresh(ai_message)
            return ai_message
        except Exception as e:
            await self.db_session.rollback()
            raise DatabaseException(detail=f"Failed to save AI message: {e}")

    async def get_last_n_messages_for_session(self, session_id: UUID, n: int) -> List[Message]:
        try:
            result = await self.db_session.execute(
                select(Message)
                .filter(Message.session_id == session_id)
                .order_by(desc(Message.created_at))
                .limit(n)
            )
            # Fetch messages in reverse order of creation to get 'last n'
            # Then reverse again to get them in chronological order for conversation history
            messages = result.scalars().all()
            return list(reversed(messages))
        except Exception as e:
            raise DatabaseException(detail=f"Failed to retrieve messages: {e}")
