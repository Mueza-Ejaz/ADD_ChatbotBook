import pytest
from unittest.mock import AsyncMock
from uuid import UUID, uuid4

from src.models import ChatSession
from src.repositories.chat_repository import ChatRepository
from src.services.session_manager import SessionManager


@pytest.mark.asyncio
async def test_find_or_create_session_creates_new_session_if_none_provided():
    mock_chat_repository = AsyncMock(spec=ChatRepository)
    
    new_session_id = uuid4()
    mock_chat_repository.create_session.return_value = ChatSession(id=new_session_id)

    session_manager = SessionManager(mock_chat_repository)

    session = await session_manager.find_or_create_session(None)

    mock_chat_repository.create_session.assert_called_once()
    mock_chat_repository.find_by_id.assert_not_called() # Corrected to find_by_id
    assert session.id == new_session_id

@pytest.mark.asyncio
async def test_find_or_create_session_finds_existing_session():
    mock_chat_repository = AsyncMock(spec=ChatRepository)

    existing_session_id = uuid4()
    existing_session = ChatSession(id=existing_session_id)
    mock_chat_repository.find_by_id.return_value = existing_session # Corrected to find_by_id

    session_manager = SessionManager(mock_chat_repository)

    session = await session_manager.find_or_create_session(existing_session_id)

    mock_chat_repository.find_by_id.assert_called_once_with(existing_session_id) # Corrected to find_by_id
    mock_chat_repository.create_session.assert_not_called()
    assert session.id == existing_session_id