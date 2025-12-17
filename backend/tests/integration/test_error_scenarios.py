import pytest
from httpx import AsyncClient, ASGITransport
from unittest.mock import patch, AsyncMock
from fastapi import status
from sqlalchemy.exc import OperationalError
from google.api_core.exceptions import GoogleAPIError

from src.main import app
from src.database import get_db
from src.exceptions import GeminiAPIException, DatabaseException

# Define a test database URL
TEST_DATABASE_URL = "sqlite+aiosqlite:///:memory:"

# Override the get_db dependency for testing
@pytest.fixture(name="db_session_override")
async def db_session_override_fixture():
    from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
    from src.models import Base

    engine = create_async_engine(TEST_DATABASE_URL, echo=False)
    async_session_maker = async_sessionmaker(
        autocommit=False, autoflush=False, bind=engine, class_=AsyncSession
    )

    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

    async def override_get_db():
        async with async_session_maker() as session:
            yield session

    app.dependency_overrides[get_db] = override_get_db
    yield
    app.dependency_overrides = {} # Clean up overrides


@pytest.mark.asyncio
async def test_chat_endpoint_handles_openai_api_error(db_session_override):
    with patch('src.services.openai_client.GeminiClient.generate_response', side_effect=GoogleAPIError("Mock Gemini API error")):
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
            response = await client.post(
                "/chat",
                json={"message": "Test message"}
            )
        assert response.status_code == status.HTTP_500_INTERNAL_SERVER_ERROR
        assert "Gemini API error" in response.json()["detail"]


@pytest.mark.asyncio
async def test_chat_endpoint_handles_database_operational_error(db_session_override):
    # Mock the AsyncSessionLocal to raise an OperationalError during a commit/flush
    with patch('src.database.AsyncSessionLocal', new_callable=AsyncMock) as MockAsyncSessionLocal:
        mock_session_instance = AsyncMock()
        mock_session_instance.__aenter__.return_value = mock_session_instance
        mock_session_instance.__aexit__.return_value = False # Don't suppress exception
        
        # Simulate an operational error on commit
        mock_session_instance.commit.side_effect = OperationalError(None, None, "Database connection lost")
        MockAsyncSessionLocal.return_value = mock_session_instance

        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
            response = await client.post(
                "/chat",
                json={"message": "Test message"}
            )
        assert response.status_code == status.HTTP_500_INTERNAL_SERVER_ERROR
        assert "Database error: (sqlite3.OperationalError) Database connection lost" in response.json()["detail"]
