from uuid import UUID
from sqlalchemy.ext.asyncio import AsyncSession
from src.repositories.chat_repository import ChatRepository


class MessageRepository:
    def __init__(self, db_session: AsyncSession):
        self.chat_repository = ChatRepository(db_session)

    async def save_user_message(self, session_id: UUID, message_content: str):
        await self.chat_repository.create_message(session_id, "user", message_content)

    async def save_ai_message(self, session_id: UUID, message_content: str):
        await self.chat_repository.create_message(session_id, "assistant", message_content)

    async def get_last_n_messages_for_session(self, session_id: UUID, n: int = 10):
        return await self.chat_repository.get_last_n_messages_for_session(session_id, n)