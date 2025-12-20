"""
Database initialization and session management.

This module sets up the asynchronous database engine and session factory
for SQLAlchemy, integrating with FastAPI's dependency injection system.
"""

import os
from typing import AsyncGenerator

from sqlalchemy.ext.asyncio import (
    create_async_engine,
    AsyncSession,
    async_sessionmaker,
    AsyncEngine,
)
from sqlalchemy.orm import declarative_base

# Import Base from models (ensure all models are imported there)
from src.models import Base

# Global engine & sessionmaker (singleton style)
_async_engine: AsyncEngine | None = None
_async_session_maker: async_sessionmaker[AsyncSession] | None = None


def get_engine() -> AsyncEngine:
    """
    Lazily create and return the async SQLAlchemy engine.
    """
    global _async_engine

    if _async_engine is None:
        database_url = os.getenv("DATABASE_URL")
        if not database_url:
            raise ValueError("DATABASE_URL environment variable is not set.")

        connect_args = {}

        # Neon + asyncpg SSL handling
        if "sslmode=require" in database_url:
            database_url = database_url.replace("?sslmode=require", "")
            database_url = database_url.replace("&sslmode=require", "")
            connect_args["ssl"] = True

        _async_engine = create_async_engine(
            database_url,
            echo=True,          # SQL logs (good for debugging)
            future=True,
            connect_args=connect_args,
        )

    return _async_engine


def get_session_maker() -> async_sessionmaker[AsyncSession]:
    """
    Lazily create and return async sessionmaker.
    """
    global _async_session_maker

    if _async_session_maker is None:
        engine = get_engine()
        _async_session_maker = async_sessionmaker(
            bind=engine,
            class_=AsyncSession,
            autoflush=False,
            autocommit=False,
            expire_on_commit=False,
        )

    return _async_session_maker


async def init_db() -> None:
    """
    Create all database tables.

    Call this once on application startup.
    """
    engine = get_engine()
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)


async def get_db() -> AsyncGenerator[AsyncSession, None]:
    """
    FastAPI dependency that provides an AsyncSession.
    """
    SessionLocal = get_session_maker()
    async with SessionLocal() as session:
        yield session


