from typing import List, Optional
from uuid import UUID
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, desc

from src.models import ChatSession, Message

class ChatRepository:
    def __init__(self, session: AsyncSession):
        self.session = session

    async def find_by_id(self, session_id: UUID) -> Optional[ChatSession]:
        result = await self.session.execute(
            select(ChatSession).where(ChatSession.id == session_id)
        )
        return result.scalars().first()

    async def create_session(self) -> ChatSession:
        chat_session = ChatSession()
        self.session.add(chat_session)
        await self.session.commit()
        await self.session.refresh(chat_session)
        return chat_session

    async def create_message(self, session_id: UUID, role: str, content: str) -> Message:
        message = Message(session_id=session_id, role=role, content=content)
        self.session.add(message)
        await self.session.commit()
        await self.session.refresh(message)
        return message

    async def get_last_n_messages_for_session(self, session_id: UUID, n: int = 10) -> List[Message]:
        result = await self.session.execute(
            select(Message)
            .where(Message.session_id == session_id)
            .order_by(desc(Message.created_at))
            .limit(n)
        )
        return list(result.scalars().all())