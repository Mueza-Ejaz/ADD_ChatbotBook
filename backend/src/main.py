import os
from typing import Annotated

from dotenv import load_dotenv
from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.ext.asyncio import AsyncSession
from pydantic import ValidationError
from loguru import logger

from src.database import get_db, init_db
from src.schemas import ChatRequest, ChatResponse
from src.services.session_manager import SessionManager
from src.services.message_repository import MessageRepository
from src.services.openai_client import GeminiClient
from src.repositories.chat_repository import ChatRepository
from src.exceptions import (
    ChatSessionNotFoundException,
    GeminiAPIException,
    DatabaseException,
)
from src.config.logging import setup_logging

# Load env & logging
load_dotenv()
setup_logging()

app = FastAPI(title="RAG Chat Backend")

# -------------------- STARTUP --------------------
@app.on_event("startup")
async def startup_event():
    logger.info("Starting application...")
    await init_db()
    logger.info("Database initialized successfully")

# -------------------- CORS --------------------
origins = ["http://localhost:3000"]

frontend_url = os.getenv("FRONTEND_URL")
if frontend_url:
    origins.extend([url.strip() for url in frontend_url.split(",")])
    logger.info(f"Frontend URLs: {origins}")

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# -------------------- HEALTH --------------------
@app.get("/health")
async def health_check():
    return {"status": "healthy"}

# -------------------- DEPENDENCIES --------------------
async def get_chat_repository(
    db: Annotated[AsyncSession, Depends(get_db)]
) -> ChatRepository:
    return ChatRepository(db)


async def get_session_manager(
    chat_repo: Annotated[ChatRepository, Depends(get_chat_repository)]
) -> SessionManager:
    return SessionManager(chat_repo)


async def get_message_repository(
    db: Annotated[AsyncSession, Depends(get_db)]
) -> MessageRepository:
    return MessageRepository(db)


async def get_gemini_client() -> GeminiClient:
    return GeminiClient()

# -------------------- CHAT ENDPOINT --------------------
@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    session_manager: Annotated[SessionManager, Depends(get_session_manager)],
    message_repository: Annotated[MessageRepository, Depends(get_message_repository)],
    gemini_client: Annotated[GeminiClient, Depends(get_gemini_client)],
):
    try:
        logger.info(f"Chat request: {request.message}")

        chat_session = await session_manager.find_or_create_session(
            request.session_id
        )

        session_id = chat_session.id

        await message_repository.save_user_message(
            session_id, request.message
        )

        history = await message_repository.get_last_n_messages_for_session(
            session_id, n=10
        )

        ai_input = request.message
        if request.selected_text:
            ai_input = f"Based on this text: {request.selected_text}\n{request.message}"

        system_prompt = gemini_client.construct_system_prompt()
        ai_response = await gemini_client.generate_response(
            system_prompt, ai_input, history
        )

        await message_repository.save_ai_message(session_id, ai_response)

        return ChatResponse(session_id=session_id, response=ai_response)

    except ChatSessionNotFoundException as e:
        raise HTTPException(status_code=404, detail=e.detail)

    except (GeminiAPIException, DatabaseException) as e:
        raise e

    except ValidationError as e:
        raise HTTPException(status_code=422, detail=e.errors())

    except Exception as e:
        logger.exception("Unexpected error")
        raise HTTPException(
            status_code=500,
            detail=str(e),
        )

