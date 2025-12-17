import pytest
from httpx import AsyncClient, ASGITransport
from uuid import UUID
from sqlalchemy import select

from src.main import app
from src.models import ChatSession, Message


@pytest.mark.asyncio
async def test_existing_chat_session_continues_conversation(session):
    # 1. Create an existing chat session with some messages
    existing_session = ChatSession()
    session.add(existing_session)
    await session.commit()
    await session.refresh(existing_session)

    message1 = Message(session_id=existing_session.id, role="user", content="Hi there!")
    message2 = Message(session_id=existing_session.id, role="assistant", content="Hello! How can I help you?")
    session.add_all([message1, message2])
    await session.commit()

    # 2. Send a POST /chat request with the existing session_id and a new message
    new_user_message = "What's the weather like today?"
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
        response = await client.post(
            "/chat",
            json={"session_id": str(existing_session.id), "message": new_user_message}
        )

    # 3. Assert response status code is 200
    assert response.status_code == 200
    response_data = response.json()

    # 4. Assert response contains correct session_id and AI response
    assert "session_id" in response_data
    assert "response" in response_data
    assert UUID(response_data["session_id"]) == existing_session.id
    assert isinstance(response_data["response"], str)
    assert response_data["response"] != new_user_message # Ensure AI didn't just echo

    # 5. Verify new user message and AI response are saved
    result = await session.execute(
        select(Message)
        .where(Message.session_id == existing_session.id)
        .order_by(Message.created_at)
    )
    messages = result.scalars().all()

    assert len(messages) == 4 # Original 2 + new user message + new AI response
    assert messages[2].role == "user"
    assert messages[2].content == new_user_message
    assert messages[3].role == "assistant"
    assert messages[3].content == response_data["response"]