import pytest
from httpx import AsyncClient, ASGITransport
from uuid import UUID
from sqlalchemy import select
from unittest.mock import patch

from src.main import app
from src.models import ChatSession, Message
from src.services.openai_client import GeminiClient


@pytest.mark.asyncio
async def test_chat_with_selected_text_prioritizes_selected_text(session):
    # Mock the GeminiClient to return a predictable response
    with patch('src.services.openai_client.GeminiClient.generate_response') as mock_generate_response:
        mock_generate_response.return_value = "AI response based on selected text: 'Docusaurus is a static site generator.'"

        selected_text = "Docusaurus is a static site generator."
        user_message = "What is this about?"

        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
            response = await client.post(
                "/chat",
                json={"message": user_message, "selected_text": selected_text}
            )

        assert response.status_code == 200
        response_data = response.json()

        assert "session_id" in response_data
        assert "response" in response_data
        assert isinstance(UUID(response_data["session_id"]), UUID)
        assert isinstance(response_data["response"], str)
        assert "Docusaurus is a static site generator." in response_data["response"]

        # Verify the GeminiClient was called with the correct parameters
        mock_generate_response.assert_called_once()
        args, kwargs = mock_generate_response.call_args
        system_prompt_arg, message_arg, history_arg = args
        
        # The important part is that the message_arg should incorporate selected_text
        assert selected_text in message_arg
        assert user_message in message_arg
