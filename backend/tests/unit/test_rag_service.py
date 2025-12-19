import os
import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from src.services.rag_service import RAGService
from qdrant_client import models

@pytest.fixture
def rag_service_instance(mocker):
    """Fixture to provide a RAGService instance with mocked Qdrant and Gemini clients."""
    with patch('src.services.rag_service.QdrantClient') as MockQdrantClient:
        
        # Mock QdrantClient
        mock_qdrant_client = MockQdrantClient.return_value
        mock_qdrant_client.collection_exists.return_value = True # Assume collection exists

        # Mock Gemini API for embeddings
        mock_embed_content_async = mocker.patch('src.services.rag_service.genai.embed_content_async', new_callable=AsyncMock, return_value={'embedding': [0.1]*768})

        # Temporarily set environment variables for initialization
        os.environ["QDRANT_URL"] = "http://mock-qdrant:6333"
        os.environ["QDRANT_API_KEY"] = "mock-qdrant-key"
        os.environ["GEMINI_API_KEY"] = "mock-gemini-key"
        os.environ["QDRANT_COLLECTION_NAME"] = "test_collection"

        service = RAGService()
        service.qdrant_client = mock_qdrant_client # Ensure the mocked client is used
        yield service, mock_embed_content_async

        # Clean up environment variables
        del os.environ["QDRANT_URL"]
        del os.environ["QDRANT_API_KEY"]
        del os.environ["GEMINI_API_KEY"]
        del os.environ["QDRANT_COLLECTION_NAME"]


@pytest.mark.asyncio
async def test_generate_query_embedding(rag_service_instance):
    """Test that query embeddings are generated correctly."""
    service, mock_embed_content_async = rag_service_instance
    query = "test query"
    embedding = await service._generate_query_embedding(query)
    assert isinstance(embedding, list)
    assert len(embedding) == 768
    mock_embed_content_async.assert_called_once_with(
        model=service.embedding_model,
        content=query
    )

@pytest.mark.parametrize("hint, expected_filter", [
    (None, None),
    ("Chapter 1", models.Filter(must=[models.FieldCondition(key="chapter", match=models.MatchText(text="Chapter 1"))])),
    ("Section 2.1", models.Filter(must=[models.FieldCondition(key="section", match=models.MatchText(text="Section 2.1"))])),
    ("Unknown hint", None), # Should not generate a filter for unrecognized hints
])
def test_build_qdrant_filter(rag_service_instance, hint, expected_filter):
    """Test that Qdrant filters are built correctly from hints."""
    service, _ = rag_service_instance
    qdrant_filter = service._build_qdrant_filter(hint)
    
    if expected_filter is None:
        assert qdrant_filter is None
    else:
        assert isinstance(qdrant_filter, models.Filter)
        assert len(qdrant_filter.must) == len(expected_filter.must)
        # Compare conditions directly as they are simple in this test
        assert qdrant_filter.must[0].key == expected_filter.must[0].key
        assert qdrant_filter.must[0].match.text == expected_filter.must[0].match.text

@pytest.mark.asyncio
async def test_retrieve_relevant_context_no_hint(rag_service_instance):
    """Test retrieving context without a selected text hint."""
    service, _ = rag_service_instance
    query = "physical AI applications"
    
    # Mock the search result from Qdrant
    mock_search_result = [
        MagicMock(
            payload={"text": "text chunk 1", "source": "src1", "chapter": "ch1", "section": "sec1"},
            score=0.9
        ),
        MagicMock(
            payload={"text": "text chunk 2", "source": "src2", "chapter": "ch2", "section": "sec2"},
            score=0.8
        )
    ]
    service.qdrant_client.search.return_value = mock_search_result

    context = await service.retrieve_relevant_context(query)
    
    assert len(context) == 2
    assert context[0]["text"] == "text chunk 1"
    assert context[0]["score"] == 0.9
    service.qdrant_client.search.assert_called_once()
    assert service.qdrant_client.search.call_args[1]["query_filter"] is None


@pytest.mark.asyncio
async def test_retrieve_relevant_context_with_hint(rag_service_instance):
    """Test retrieving context with a selected text hint."""
    service, _ = rag_service_instance
    query = "robotics and AI"
    hint = "Chapter 3"

    mock_search_result = [
        MagicMock(
            payload={"text": "text chunk 3", "source": "src3", "chapter": "ch3", "section": "sec1"},
            score=0.95
        )
    ]
    service.qdrant_client.search.return_value = mock_search_result

    context = await service.retrieve_relevant_context(query, selected_text_hint=hint)

    assert len(context) == 1
    assert context[0]["text"] == "text chunk 3"
    assert context[0]["chapter"] == "ch3"
    service.qdrant_client.search.assert_called_once()
    assert service.qdrant_client.search.call_args[1]["query_filter"] is not None
    assert service.qdrant_client.search.call_args[1]["query_filter"].must[0].key == "chapter"
    assert service.qdrant_client.search.call_args[1]["query_filter"].must[0].match.text == "Chapter 3"
@pytest.mark.asyncio
async def test_retrieve_relevant_context_no_results(rag_service_instance):
    """Test retrieving context when Qdrant returns no results."""
    service, _ = rag_service_instance
    query = "non-existent topic"
    service.qdrant_client.search.return_value = []

    context = await service.retrieve_relevant_context(query)
    assert len(context) == 0


    