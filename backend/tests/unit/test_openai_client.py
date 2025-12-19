import pytest
from unittest.mock import AsyncMock, patch
from google.api_core.exceptions import GoogleAPIError

from src.services.openai_client import GeminiClient
from src.models import Message
from src.exceptions import GeminiAPIException


@pytest.fixture
def gemini_client():
    # Patch os.getenv to prevent actual API key loading during unit tests
    with patch('os.getenv', return_value='dummy_api_key'):
        return GeminiClient()

def test_construct_system_prompt(gemini_client):
    prompt = gemini_client.construct_system_prompt()
    assert prompt == "You are a helpful AI assistant. You answer questions concisely and accurately."

@pytest.mark.asyncio
async def test_generate_response_sends_message_and_returns_response(gemini_client):
    mock_chat = AsyncMock()
    mock_chat.send_message_async.return_value.text = "Mocked AI Response"

    with patch('google.generativeai.GenerativeModel', MagicMock()) as MockGenerativeModel:
        MockGenerativeModel.return_value.start_chat.return_value = mock_chat
        
        system_prompt = "You are an AI."
        user_message = "Hello!"
        conversation_history = [
            Message(session_id=uuid4(), role="user", content="Hi"),
            Message(session_id=uuid4(), role="assistant", content="How can I help?")
        ]

        response = await gemini_client.generate_response(system_prompt, user_message, conversation_history)

        mock_chat.send_message_async.assert_called_once_with(system_prompt + "\n" + user_message)
        assert response == "Mocked AI Response"

        # Verify chat history was set up correctly
        history_args, history_kwargs = MockGenerativeModel.return_value.start_chat.call_args
        assert history_kwargs['history'] == [
            {"role": "user", "parts": ["Hi"]},
            {"role": "model", "parts": ["How can I help?"]}
        ]

@pytest.mark.asyncio
async def test_generate_response_raises_gemini_api_exception_on_api_error(gemini_client):
    mock_chat = AsyncMock()
    mock_chat.send_message_async.side_effect = GoogleAPIError("API error occurred")

    with patch('google.generativeai.GenerativeModel', MagicMock()) as MockGenerativeModel:
        MockGenerativeModel.return_value.start_chat.return_value = mock_chat

        system_prompt = "You are an AI."
        user_message = "Hello!"
        conversation_history = []

        with pytest.raises(GeminiAPIException) as excinfo:
            await gemini_client.generate_response(system_prompt, user_message, conversation_history)

        assert "API error occurred" in str(excinfo.value.detail)
