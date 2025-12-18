import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from backend.src.services.rag_service import RAGService
from qdrant_client import models
import os

# Mock environment variables for testing
@pytest.fixture(autouse=True)
def mock_env_vars():
    with patch.dict(os.environ, {
        "QDRANT_URL": "http://mock-qdrant:6333",
        "QDRANT_API_KEY": "mock-qdrant-key",
        "GEMINI_API_KEY": "mock-gemini-key",
        "QDRANT_COLLECTION_NAME": "test_collection"
    }):
        yield

@pytest.fixture
def rag_service():
    # Patch the QdrantClient and genai.configure during RAGService instantiation
    with patch('backend.src.services.rag_service.QdrantClient') as MockQdrantClient, \
         patch('backend.src.services.rag_service.genai.configure') as MockGenaiConfigure:
        
        service = RAGService()
        service.qdrant_client = MockQdrantClient.return_value # Ensure instance uses our mock
        yield service

@pytest.mark.asyncio
async def test_generate_query_embedding_success(rag_service):
    """
    Test successful generation of a query embedding.
    """
    mock_embedding = [0.1] * 768
    with patch('backend.src.services.rag_service.genai.embed_content_async', new_callable=AsyncMock) as mock_embed_content:
        mock_embed_content.return_value = {'embedding': mock_embedding}
        
        query = "test query"
        embedding = await rag_service._generate_query_embedding(query)
        
        mock_embed_content.assert_awaited_once_with(
            model=rag_service.embedding_model,
            content=query,
            task_type="RETRIEVAL_QUERY"
        )
        assert embedding == mock_embedding

@pytest.mark.asyncio
async def test_generate_query_embedding_empty_query(rag_service):
    """
    Test generating embedding for an empty query.
    """
    embedding = await rag_service._generate_query_embedding("")
    assert embedding == []

@pytest.mark.asyncio
async def test_retrieve_relevant_context_no_hint(rag_service):
    """
    Test retrieving context with a simple query and no hint.
    """
    # Mock embedding generation
    mock_query_embedding = [0.2] * 768
    with patch('backend.src.services.rag_service.genai.embed_content_async', new_callable=AsyncMock) as mock_embed_content:
        mock_embed_content.return_value = {'embedding': mock_query_embedding}
        
        # Mock Qdrant search result
        mock_search_result = [
            MagicMock(
                payload={"text": "chunk1", "source": "src1", "chapter": "ch1", "section": "sec1"},
                score=0.9
            ),
            MagicMock(
                payload={"text": "chunk2", "source": "src2", "chapter": "ch2", "section": "sec2"},
                score=0.8
            )
        ]
        rag_service.qdrant_client.search.return_value = mock_search_result

        query = "What is AI?"
        contexts = await rag_service.retrieve_relevant_context(query, limit=2)

        mock_embed_content.assert_awaited_once()
        rag_service.qdrant_client.search.assert_called_once_with(
            collection_name="test_collection",
            query_vector=mock_query_embedding,
            query_filter=None,
            limit=2,
            with_payload=True,
            with_vectors=False,
        )
        assert len(contexts) == 2
        assert contexts[0]["text"] == "chunk1"
        assert contexts[0]["source"] == "src1"
        assert contexts[0]["score"] == 0.9
        assert contexts[1]["text"] == "chunk2"

@pytest.mark.asyncio
async def test_retrieve_relevant_context_with_hint(rag_service):
    """
    Test retrieving context with a query and a selected text hint.
    Since _build_qdrant_filter currently returns None, the hint won't be used for filtering.
    """
    mock_query_embedding = [0.3] * 768
    with patch('backend.src.services.rag_service.genai.embed_content_async', new_callable=AsyncMock) as mock_embed_content:
        mock_embed_content.return_value = {'embedding': mock_query_embedding}
        
        mock_search_result = [
            MagicMock(
                payload={"text": "hinted chunk", "source": "hint_src", "chapter": "hint_ch", "section": "hint_sec"},
                score=0.95
            )
        ]
        rag_service.qdrant_client.search.return_value = mock_search_result

        query = "latest developments"
        selected_text_hint = "chapter: Advanced Robotics"
        contexts = await rag_service.retrieve_relevant_context(query, limit=1, selected_text_hint=selected_text_hint)

        mock_embed_content.assert_awaited_once()
        # Expect filter to be None as per current _build_qdrant_filter implementation
        rag_service.qdrant_client.search.assert_called_once_with(
            collection_name="test_collection",
            query_vector=mock_query_embedding,
            query_filter=None, # Expect None because _build_qdrant_filter returns None
            limit=1,
            with_payload=True,
            with_vectors=False,
        )
        assert len(contexts) == 1
        assert contexts[0]["text"] == "hinted chunk"
        assert contexts[0]["source"] == "hint_src"
        assert contexts[0]["score"] == 0.95

@pytest.mark.asyncio
async def test_retrieve_relevant_context_empty_query(rag_service):
    """
    Test retrieving context with an empty query.
    """
    contexts = await rag_service.retrieve_relevant_context("")
    assert contexts == []
    rag_service.qdrant_client.search.assert_not_called()

@pytest.mark.asyncio
def test_rag_service_initialization_missing_env_vars():
    """
    Test RAGService initialization fails with missing Qdrant env vars.
    """
    with patch.dict(os.environ, {}, clear=True): # Clear env vars
        with pytest.raises(ValueError, match="QDRANT_URL and QDRANT_API_KEY must be set in environment variables."):
            RAGService()
