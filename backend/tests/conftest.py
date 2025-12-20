import sys
import os
from typing import AsyncGenerator
from dotenv import load_dotenv # Import load_dotenv

# Add the 'backend' directory to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

load_dotenv() # Load environment variables before any other imports that might depend on them

import pytest
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy.orm import sessionmaker, Session as SyncSession
from unittest.mock import AsyncMock
from uuid import UUID, uuid4

from src.models import Base, ChatSession
from src.main import app, get_chat_repository, get_session_manager, get_message_repository, get_gemini_client # Import the FastAPI app and dependency providers
from src.database import get_db # Import the original get_db
from src.repositories.chat_repository import ChatRepository # Import for type hinting mocks and real instances
from src.services.session_manager import SessionManager
from src.services.message_repository import MessageRepository
from src.services.openai_client import GeminiClient

@pytest.fixture(name="session")
async def session_fixture() -> AsyncGenerator[AsyncSession, None]:
    # Use the in-memory SQLite database for testing
    engine = create_async_engine("sqlite+aiosqlite:///:memory:", echo=False)
    
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

    TestingSessionLocal = async_sessionmaker(
        autocommit=False, autoflush=False, bind=engine, class_=AsyncSession, expire_on_commit=False
    )

    async with TestingSessionLocal() as session:
        yield session

    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.drop_all)

@pytest.fixture(name="override_get_db", autouse=True)
async def override_get_db_fixture(session: AsyncSession):
    async def _get_db_override():
        yield session
    app.dependency_overrides[get_db] = _get_db_override
    yield
    app.dependency_overrides.pop(get_db, None) # Clean up only this override

# --- Mocked Dependencies ---
@pytest.fixture
async def mock_chat_repository() -> AsyncMock:
    mock = AsyncMock(spec=ChatRepository)
    yield mock

@pytest.fixture
async def mock_session_manager(mock_chat_repository: AsyncMock) -> AsyncMock:
    mock = AsyncMock(spec=SessionManager)
    # The SessionManager's find_or_create_session will call chat_repository methods
    # so we need to ensure the mock has these methods for scenarios where find_or_create_session is mocked less
    mock_chat_session = ChatSession(id=uuid4()) # Create a real ChatSession instance
    mock.find_or_create_session.return_value = mock_chat_session
    yield mock

@pytest.fixture
async def mock_message_repository() -> AsyncMock:
    mock = AsyncMock(spec=MessageRepository)
    yield mock

@pytest.fixture
async def mock_gemini_client() -> AsyncMock:
    mock = AsyncMock(spec=GeminiClient)
    # Default return for generate_response
    mock.generate_response.return_value = "Mocked AI Response from Gemini Client"
    mock.construct_system_prompt.return_value = "Mocked system prompt"
    yield mock

@pytest.fixture
def override_all_dependencies(
    override_get_db, # Ensure db is overridden for tests that need it
    mock_chat_repository: AsyncMock,
    mock_session_manager: AsyncMock,
    mock_message_repository: AsyncMock,
    mock_gemini_client: AsyncMock
):
    """Overrides FastAPI dependencies with mock implementations."""
    app.dependency_overrides[get_chat_repository] = lambda: mock_chat_repository
    app.dependency_overrides[get_session_manager] = lambda: mock_session_manager
    app.dependency_overrides[get_message_repository] = lambda: mock_message_repository
    app.dependency_overrides[get_gemini_client] = lambda: mock_gemini_client
    yield
    # Clean up is handled by individual mock fixtures now, as they are yielded
    app.dependency_overrides = {} # This will clean all overrides at the end of each test that uses these fixtures.

# --- Real Dependencies ---
@pytest.fixture
async def real_chat_repository(session: AsyncSession) -> ChatRepository:
    """Provides a real ChatRepository instance for testing."""
    return ChatRepository(session)

@pytest.fixture
async def real_session_manager(real_chat_repository: ChatRepository) -> SessionManager:
    """Provides a real SessionManager instance for testing."""
    return SessionManager(real_chat_repository)

@pytest.fixture
async def real_message_repository(session: AsyncSession) -> MessageRepository:
    """Provides a real MessageRepository instance for testing."""
    return MessageRepository(session)

@pytest.fixture
async def real_gemini_client() -> GeminiClient:
    """Provides a real GeminiClient instance for testing."""
    # In integration tests, we might still want to mock external calls of GeminiClient.
    # For now, return a real instance.
    return GeminiClient()

@pytest.fixture
def use_real_dependencies(
    override_get_db, # Ensure db is overridden for tests that need it
    real_chat_repository: ChatRepository,
    real_session_manager: SessionManager,
    real_message_repository: MessageRepository,
    real_gemini_client: GeminiClient
):
    """Overrides FastAPI dependencies with real implementations for integration tests."""
    app.dependency_overrides[get_chat_repository] = lambda: real_chat_repository
    app.dependency_overrides[get_session_manager] = lambda: real_session_manager
    app.dependency_overrides[get_message_repository] = lambda: real_message_repository
    app.dependency_overrides[get_gemini_client] = lambda: real_gemini_client
    yield
    app.dependency_overrides = {} # Clean up all overrides from this fixture

pytest_plugins = ("pytest_asyncio",)