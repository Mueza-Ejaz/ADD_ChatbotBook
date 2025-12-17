import os
from typing import Annotated
from dotenv import load_dotenv
from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.ext.asyncio import AsyncSession
from pydantic import ValidationError
from loguru import logger # Import logger

from src.database import get_db
from src.schemas import ChatRequest, ChatResponse
from src.services.session_manager import SessionManager
from src.services.message_repository import MessageRepository
from src.services.openai_client import GeminiClient
from src.models import Message
from src.exceptions import ChatSessionNotFoundException, GeminiAPIException, DatabaseException
from src.config.logging import setup_logging # Import setup_logging

load_dotenv()

setup_logging() # Call setup_logging at the start

app = FastAPI()

# Configure CORS
origins = [
    "http://localhost:3000",  # Frontend development server
]

# Add FRONTEND_URL from environment variables if available
frontend_url = os.getenv("FRONTEND_URL")
if frontend_url:
    origins.extend([url.strip() for url in frontend_url.split(',')])
    logger.info(f"Frontend URL(s) configured: {origins}") # Log this

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/health")
async def health_check():
    logger.info("Health check endpoint called.") # Log health check
    return {"status": "healthy"}


@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    db_session: Annotated[AsyncSession, Depends(get_db)]
):
    try:
        logger.info(f"Chat request received: session_id={request.session_id}, message='{request.message}'")

        session_manager = SessionManager(db_session)
        message_repository = MessageRepository(db_session)
        gemini_client = GeminiClient()

        chat_session = await session_manager.find_or_create_session(request.session_id)
        session_id = chat_session.id
        logger.debug(f"Chat session: {session_id}")

        # Save user message
        await message_repository.save_user_message(session_id, request.message)
        logger.info(f"User message saved for session {session_id}")

        # Retrieve conversation history for context
        conversation_history = await message_repository.get_last_n_messages_for_session(session_id, n=10)
        logger.debug(f"Conversation history for session {session_id}: {len(conversation_history)} messages")

        # Prepare message for AI, incorporating selected_text if present
        ai_message_input = request.message
        if request.selected_text:
            ai_message_input = f"Based on the following text: '{request.selected_text}', {request.message}"
            logger.debug(f"Selected text incorporated. AI input: '{ai_message_input}'")

        # Generate AI response
        system_prompt = gemini_client.construct_system_prompt()
        ai_response_content = await gemini_client.generate_response(system_prompt, ai_message_input, conversation_history)
        logger.info(f"AI response generated for session {session_id}")

        # Save AI response
        await message_repository.save_ai_message(session_id, ai_response_content)
        logger.info(f"AI response saved for session {session_id}")

        return ChatResponse(session_id=session_id, response=ai_response_content)
    except ChatSessionNotFoundException as e:
        logger.warning(f"ChatSessionNotFoundException: {e.detail}")
        raise e
    except GeminiAPIException as e:
        logger.error(f"GeminiAPIException: {e.detail}")
        raise e
    except DatabaseException as e:
        logger.error(f"DatabaseException: {e.detail}")
        raise e
    except ValidationError as e:
        logger.warning(f"ValidationError: {e.errors()}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail=e.errors()
        )
    except Exception as e:
        logger.exception("An unexpected error occurred in chat_endpoint.") # Use logger.exception for full traceback
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"An unexpected error occurred: {str(e)}"
        )