import pytest
from httpx import AsyncClient
from unittest.mock import AsyncMock, patch, MagicMock
from src.main import app # Assuming main.py is in src directory for testing setup
import os
from uuid import UUID

# Mock environment variables
@pytest.fixture(autouse=True)
def mock_env_vars():
    with patch.dict(os.environ, {
        "QDRANT_URL": "https://d6a06572-5a56-497a-9a2f-bdf94d137ce8.europe-west3-0.gcp.cloud.qdrant.io",
        "QDRANT_API_KEY": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.syw1JVbam6R7ON0iWTuMGpjDNYuiwUmwjwlm4fsUmyo",
        "GEMINI_API_KEY": "AIzaSyB8hcOSYiEjEYgh2n-kiH1NkZAI8QihTbQ",
        "QDRANT_COLLECTION_NAME": "test_collection",
        "INGESTION_API_KEY": "test-ingestion-key", # For ingest endpoint tests
        "FRONTEND_URL": "http://localhost:3000"
    }):
        yield

@pytest.fixture
async def async_client():
    async with AsyncClient(app=app, base_url="http://test") as client:
        yield client

@pytest.fixture
def mock_rag_service():
    # Mock RAGService for `retrieve_relevant_context`
    with patch('src.services.rag_service.RAGService', autospec=True) as MockRAGService:
        mock_instance = MockRAGService.return_value
        mock_instance.retrieve_relevant_context = AsyncMock()
        yield mock_instance

@pytest.fixture
def mock_gemini_client():
    # Mock GeminiClient for `generate_response` and `construct_system_prompt`
    with patch('src.services.openai_client.GeminiClient', autospec=True) as MockGeminiClient:
        mock_instance = MockGeminiClient.return_value
        mock_instance.generate_response = AsyncMock()
        mock_instance.construct_system_prompt = MagicMock(return_value="Mock System Prompt")
        yield mock_instance

@pytest.fixture
def mock_session_manager():
    # Mock SessionManager for `find_or_create_session`
    with patch('src.services.session_manager.SessionManager', autospec=True) as MockSessionManager:
        mock_instance = MockSessionManager.return_value
        mock_session = MagicMock(id=UUID("00000000-0000-0000-0000-000000000001"))
        mock_instance.find_or_create_session = AsyncMock(return_value=mock_session)
        yield mock_instance

@pytest.fixture
def mock_message_repository():
    # Mock MessageRepository for `save_user_message`, `save_ai_message`, `get_last_n_messages_for_session`
    with patch('src.services.message_repository.MessageRepository', autospec=True) as MockMessageRepository:
        mock_instance = MockMessageRepository.return_value
        mock_instance.save_user_message = AsyncMock()
        mock_instance.save_ai_message = AsyncMock()
        mock_instance.get_last_n_messages_for_session = AsyncMock(return_value=[]) # Default empty history
        yield mock_instance

@pytest.mark.asyncio
async def test_chat_endpoint_no_rag_context(
    async_client: AsyncClient,
    mock_rag_service: MagicMock,
    mock_gemini_client: MagicMock,
    mock_session_manager: MagicMock,
    mock_message_repository: MagicMock
):
    """
    Test chat endpoint when no RAG context is retrieved.
    """
    mock_rag_service.retrieve_relevant_context.return_value = []
    mock_gemini_client.generate_response.return_value = "AI response without RAG context."

    response = await async_client.post("/chat", json={"message": "Hello", "session_id": str(UUID("00000000-0000-0000-0000-000000000001"))})

    assert response.status_code == 200
    assert response.json()["response"] == "AI response without RAG context."
    assert response.json()["session_id"] == "00000000-0000-0000-0000-000000000001"
    assert response.json()["sources"] == []
    mock_rag_service.retrieve_relevant_context.assert_called_once()
    mock_gemini_client.generate_response.assert_called_once()

@pytest.mark.asyncio
async def test_chat_endpoint_with_rag_context(
    async_client: AsyncClient,
    mock_rag_service: MagicMock,
    mock_gemini_client: MagicMock,
    mock_session_manager: MagicMock,
    mock_message_repository: MagicMock
):
    """
    Test chat endpoint when RAG context is successfully retrieved.
    """
    mock_context = [
        {"text": "Contextual info 1", "source": "book_a", "chapter": "ch1", "section": "sec1"},
        {"text": "Contextual info 2", "source": "book_b", "chapter": "ch2", "section": "sec1"}
    ]
    mock_rag_service.retrieve_relevant_context.return_value = mock_context
    mock_gemini_client.generate_response.return_value = "AI response with RAG context."

    response = await async_client.post("/chat", json={"message": "Tell me about it", "session_id": str(UUID("00000000-0000-0000-0000-000000000002"))})

    assert response.status_code == 200
    assert response.json()["response"] == "AI response with RAG context."
    assert response.json()["session_id"] == "00000000-0000-0000-0000-000000000002"
    assert set(response.json()["sources"]) == {
        "book_a - ch1 - sec1",
        "book_b - ch2 - sec1"
    }

    # Verify the generate_response was called with an augmented prompt
    args, kwargs = mock_gemini_client.generate_response.call_args
    assert "Contextual info 1" in args[1]
    assert "Contextual info 2" in args[1]
    assert "Tell me about it" in args[1]
    mock_rag_service.retrieve_relevant_context.assert_called_once()

@pytest.mark.asyncio
async def test_chat_endpoint_with_selected_text_and_rag_context(
    async_client: AsyncClient,
    mock_rag_service: MagicMock,
    mock_gemini_client: MagicMock,
    mock_session_manager: MagicMock,
    mock_message_repository: MagicMock
):
    """
    Test chat endpoint with both selected text hint and RAG context.
    """
    mock_context = [
        {"text": "More specific context", "source": "book_c", "chapter": "ch3", "section": "sec1"}
    ]
    mock_rag_service.retrieve_relevant_context.return_value = mock_context
    mock_gemini_client.generate_response.return_value = "AI response with selected text and RAG context."

    selected_text = "The quick brown fox"
    response = await async_client.post(
        "/chat",
        json={"message": "What about it?", "session_id": str(UUID("00000000-0000-0000-0000-000000000003")), "selected_text": selected_text}
    )

    assert response.status_code == 200
    assert response.json()["response"] == "AI response with selected text and RAG context."
    assert response.json()["session_id"] == "00000000-0000-0000-0000-000000000003"
    assert response.json()["sources"] == ["book_c - ch3 - sec1"]

    # Verify the generate_response was called with an augmented prompt including selected text
    args, kwargs = mock_gemini_client.generate_response.call_args
    assert f"### Focus Text:\n'{selected_text}'" in args[1]
    assert "More specific context" in args[1]
    assert "What about it?" in args[1]
    mock_rag_service.retrieve_relevant_context.assert_called_once()

# Add a test for error handling
@pytest.mark.asyncio
async def test_chat_endpoint_gemini_api_exception(
    async_client: AsyncClient,
    mock_rag_service: MagicMock,
    mock_gemini_client: MagicMock,
    mock_session_manager: MagicMock,
    mock_message_repository: MagicMock
):
    """
    Test chat endpoint handles GeminiAPIException.
    """
    from src.exceptions import GeminiAPIException # Import here to avoid circular dependency issues if main.py is not fully loaded
    mock_gemini_client.generate_response.side_effect = GeminiAPIException(detail="Gemini error")

    response = await async_client.post("/chat", json={"message": "Error test", "session_id": str(UUID("00000000-0000-0000-0000-000000000004"))})

    assert response.status_code == 500 # HTTPException status for GeminiAPIException
    assert "Gemini API error: Gemini error" in response.json()["detail"]
    mock_gemini_client.generate_response.assert_called_once()
