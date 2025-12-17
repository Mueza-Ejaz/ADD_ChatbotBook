"""
Database initialization and session management.

This module sets up the asynchronous database engine and session factory
for SQLAlchemy, integrating with FastAPI's dependency injection system.
"""

import os
from typing import AsyncGenerator

from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy.orm import declarative_base
from sqlalchemy.ext.asyncio import AsyncEngine # Added this import

# Define global variables to hold the engine and sessionmaker instances.
# This allows them to be initialized once and reused across the application.
_async_engine: AsyncEngine | None = None
_async_session_maker: async_sessionmaker[AsyncSession] | None = None


def get_engine() -> AsyncEngine:
    """
    Returns the asynchronously configured SQLAlchemy engine.

    The engine is created lazily (on first call) to ensure that
    environment variables are loaded before its initialization.
    """
    global _async_engine
    if _async_engine is None:
        database_url = os.getenv("DATABASE_URL")
        if not database_url:
            raise ValueError("DATABASE_URL environment variable is not set.")
        _async_engine = create_async_engine(database_url, echo=True, future=True)
    return _async_engine


def get_session_maker() -> async_sessionmaker[AsyncSession]:
    """
    Returns the asynchronously configured sessionmaker.

    The sessionmaker is created lazily (on first call) using the
    lazily initialized engine.
    """
    global _async_session_maker
    if _async_session_maker is None:
        engine = get_engine()
        _async_session_maker = async_sessionmaker(
            autocommit=False,
            autoflush=False,
            bind=engine,
            class_=AsyncSession,
            expire_on_commit=False,
        )
    return _async_session_maker


Base = declarative_base() # Moved this to after engine/sessionmaker setup for logical flow, although it can be anywhere


async def get_db() -> AsyncGenerator[AsyncSession, None]:
    """
    Dependency that provides an async database session.

    This function provides a SQLAlchemy AsyncSession to FastAPI endpoints.
    The session is automatically closed after the request is finished.
    """
    AsyncSessionLocal = get_session_maker()
    async with AsyncSessionLocal() as session:
        yield session