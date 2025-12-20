from uuid import UUID
from typing import List, Optional
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from src.models import ChatSession, Message

class ChatRepository:
    def __init__(self, session: AsyncSession):
        self.session = session

    async def create_session(self) -> ChatSession:
        session = ChatSession()
        self.session.add(session)
        await self.session.commit()
        await self.session.refresh(session)
        return session

    async def find_session_by_id(self, session_id: UUID) -> Optional[ChatSession]:
        result = await self.session.execute(
            select(ChatSession).filter(ChatSession.id == session_id)
        )
        return result.scalars().first()

    async def create_message(self, session_id: UUID, role: str, content: str) -> Message:
        message = Message(session_id=session_id, role=role, content=content)
        self.session.add(message)
        await self.session.commit()
        await self.session.refresh(message)
        return message

    async def get_last_n_messages_for_session(self, session_id: UUID, n: int) -> List[Message]:
        result = await self.session.execute(
            select(Message)
            .filter(Message.session_id == session_id)
            .order_by(Message.created_at.desc())
            .limit(n)
        )
        return list(reversed(result.scalars().all())) # Return in chronological order