import pytest
from unittest.mock import AsyncMock, patch, MagicMock
from uuid import UUID, uuid4
from google.api_core.exceptions import GoogleAPIError
import google.generativeai as genai # Import genai for patching

from src.services.openai_client import GeminiClient
from src.models import Message
from src.exceptions import GeminiAPIException


@pytest.fixture
def gemini_client():
    with patch('os.getenv', return_value='dummy_api_key'), \
         patch('google.generativeai.configure') as mock_genai_configure, \
         patch('google.generativeai.GenerativeModel') as MockGenerativeModel:
        
        # Configure the mock GenerativeModel to return a mock chat object
        mock_chat = AsyncMock()
        MockGenerativeModel.return_value.start_chat.return_value = mock_chat
        
        client = GeminiClient()
        yield client

def test_construct_system_prompt(gemini_client):
    prompt = gemini_client.construct_system_prompt()
    assert prompt == "You are a helpful AI assistant. You answer questions concisely and accurately."

@pytest.mark.asyncio
async def test_generate_response_sends_message_and_returns_response(gemini_client):
    # The MockGenerativeModel and its mock_chat are set up in the fixture
    # We need to access the mock_chat created within the fixture's scope
    # Access the mock from the client instance
    mock_chat = gemini_client.model.start_chat.return_value
    mock_chat.send_message_async.return_value.text = "Mocked AI Response"
    
    system_prompt = "You are an AI."
    user_message = "Hello!"
    conversation_history = [
        Message(session_id=uuid4(), role="user", content="Hi"),
        Message(session_id=uuid4(), role="assistant", content="How can I help?")
    ]

    response = await gemini_client.generate_response(system_prompt, user_message, conversation_history)

    mock_chat.send_message_async.assert_called_once_with(system_prompt + "\n" + user_message)
    assert response == "Mocked AI Response"

    # History assertion needs to be done on the mock GenerativeModel if it was patched globally
    # or on the client.model.start_chat mock
    gemini_client.model.start_chat.assert_called_once()
    history_args, history_kwargs = gemini_client.model.start_chat.call_args
    assert history_kwargs['history'] == [
        {"role": "user", "parts": ["Hi"]},
        {"role": "model", "parts": ["How can I help?"]}
    ]

@pytest.mark.asyncio
async def test_generate_response_raises_gemini_api_exception_on_api_error(gemini_client):
    mock_chat = gemini_client.model.start_chat.return_value
    mock_chat.send_message_async.side_effect = GoogleAPIError("API error occurred")

    system_prompt = "You are an AI."
    user_message = "Hello!"
    conversation_history = []

    with pytest.raises(GeminiAPIException) as excinfo:
        await gemini_client.generate_response(system_prompt, user_message, conversation_history)

    assert "Gemini API error" in str(excinfo.value.detail)
