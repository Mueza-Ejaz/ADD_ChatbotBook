import sys
import os
from typing import AsyncGenerator

# Add the 'backend' directory to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import pytest
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy.orm import sessionmaker, Session as SyncSession

from src.models import Base
from src.main import app # Import the FastAPI app
from src.database import get_db # Import the original get_db

@pytest.fixture(name="session")
async def session_fixture() -> AsyncGenerator[AsyncSession, None]:
    # Use the in-memory SQLite database for testing as defined in src/database.py for TESTING=true
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
async def override_get_db_fixture(session: AsyncSession): # session fixture is now async
    async def _get_db_override():
        yield session
    app.dependency_overrides[get_db] = _get_db_override
    yield
    app.dependency_overrides = {} # Clear overrides after test

pytest_plugins = ("pytest_asyncio",)