import pytest
from httpx import AsyncClient, ASGITransport
from uuid import UUID
from sqlalchemy import select

from src.main import app
from src.models import ChatSession, Message


@pytest.mark.asyncio
async def test_new_chat_session_creates_session_and_message(session):
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post(
            "/chat",
            json={"message": "Hello, AI!"}
        )

    assert response.status_code == 200
    response_data = response.json()

    assert "session_id" in response_data
    assert "response" in response_data
    assert isinstance(UUID(response_data["session_id"]), UUID)
    assert isinstance(response_data["response"], str)

    # Verify session and message are created in the database
    chat_session = await session.get(ChatSession, UUID(response_data["session_id"]))
    assert chat_session is not None

    result = await session.execute(
        select(Message).filter_by(session_id=chat_session.id)
    )
    messages = result.scalars().all()
    assert len(messages) == 2  # User message and AI response
    assert messages[0].role == "user"
    assert messages[0].content == "Hello, AI!"
    assert messages[1].role == "assistant"
    assert messages[1].content == response_data["response"]

