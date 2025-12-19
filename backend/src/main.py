import os
from typing import Annotated, List
from dotenv import load_dotenv
from fastapi import FastAPI, Depends, HTTPException, status, Header
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.ext.asyncio import AsyncSession
from pydantic import ValidationError
from loguru import logger # Import logger

from src.database import get_db
from src.schemas import ChatRequest, ChatResponse
from src.services.session_manager import SessionManager
from src.services.message_repository import MessageRepository
from src.services.openai_client import GeminiClient
from src.services.rag_service import RAGService
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

async def async_get_rag_service() -> RAGService:
    return RAGService()

async def verify_api_key(x_api_key: str = Header(..., alias="X-API-Key")) -> bool:
    """
    Dependency to verify API key for ingestion endpoint.
    """
    ingestion_api_key = os.getenv("INGESTION_API_KEY")
    if not ingestion_api_key:
        logger.error("INGESTION_API_KEY is not set in environment variables.")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Server configuration error: Ingestion API key not set."
        )
    if x_api_key != ingestion_api_key:
        logger.warning("Unauthorized ingestion attempt with invalid API key.")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid API Key"
        )
    return True

@app.get("/health")
async def health_check():
    """
    Checks the health of the application.

    Returns:
        dict: A dictionary with a status of "healthy".
    """
    logger.info("Health check endpoint called.") # Log health check
    return {"status": "healthy"}

@app.post("/ingest")
async def ingest_data(
    authenticated: Annotated[bool, Depends(verify_api_key)],
    force_resync: bool = False, # Parameter to allow forcing a full resync
):
    """
    Triggers the data ingestion process to synchronize book content with Qdrant.
    This endpoint will execute the `sync_book_to_qdrant.py` script as a subprocess.
    """
    logger.info(f"Ingestion endpoint called. Force resync: {force_resync}")

    # Define the path to the sync script relative to the project root
    script_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "tools", "sync_book_to_qdrant.py"))
    
    command = ["python", script_path]
    if force_resync:
        command.append("--force-resync")

    # Pass relevant environment variables to the subprocess
    env = os.environ.copy()
    # Ensure QDRANT_URL, QDRANT_API_KEY, GEMINI_API_KEY are passed to the subprocess
    # They should already be loaded by dotenv at the app startup, but explicit is better.
    env["QDRANT_URL"] = os.getenv("QDRANT_URL")
    env["QDRANT_API_KEY"] = os.getenv("QDRANT_API_KEY")
    env["GEMINI_API_KEY"] = os.getenv("GEMINI_API_KEY")
    env["GOOGLE_API_KEY"] = os.getenv("GOOGLE_API_KEY") # Ensure both are passed

    try:
        # Start the subprocess without waiting for it to complete
        process = await asyncio.create_subprocess_exec(
            *command,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            env=env
        )
        # We don't await process.wait() here because ingestion can be long-running
        # and we don't want to block the API response.
        # However, for demonstration and initial testing, it might be useful to get some output.
        # For production, consider using a background task queue (e.g., Celery) or
        # streaming logs back.

        # For now, just log that the process was started.
        stdout, stderr = await process.communicate()
        
        if process.returncode == 0:
            logger.info(f"Ingestion script started successfully. Stdout: {stdout.decode().strip()}")
            return {"status": "Ingestion process initiated successfully.", "details": stdout.decode().strip()}
        else:
            logger.error(f"Ingestion script failed to start. Stderr: {stderr.decode().strip()}")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Ingestion script failed: {stderr.decode().strip()}"
            )

    except Exception as e:
        logger.exception("Failed to initiate ingestion process.")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to initiate ingestion process: {str(e)}"
        )


@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    db_session: Annotated[AsyncSession, Depends(get_db)],
    rag_service: Annotated[RAGService, Depends(async_get_rag_service)]
) -> ChatResponse:
    """
    Handles chat interactions, managing sessions, messages, and AI responses.

    If no session_id is provided, a new chat session is created.
    The user's message is saved, an AI response is generated based on conversation history
    and optionally selected text, and the AI's response is also saved.

    Args:
        request (ChatRequest): The incoming chat request containing the message,
                               optional session_id, and optional selected_text.
        db_session (Annotated[AsyncSession, Depends(get_db)]): Asynchronous database session.
        rag_service (Annotated[RAGService, Depends(async_get_rag_service)]): RAG service dependency.

    Returns:
        ChatResponse: The AI's response and the session_id.

    Raises:
        ChatSessionNotFoundException: If a provided session_id does not exist.
        GeminiAPIException: If an error occurs during interaction with the Gemini API.
        DatabaseException: If a database-related error occurs.
        HTTPException: For validation errors (422) or unexpected internal server errors (500).
    """
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
        # ai_message_input = request.message
        # if request.selected_text:
        #     ai_message_input = f"Based on the following text: '{request.selected_text}', {request.message}"
        #     logger.debug(f"Selected text incorporated. AI input: '{ai_message_input}'")

        # New logic for RAG
        retrieved_contexts = await rag_service.retrieve_relevant_context(
            query=request.message,
            selected_text_hint=request.selected_text
        )

        context_str = ""
        sources = []
        if retrieved_contexts:
            context_str = "\n\n" + "### Relevant Context:\n" + \
                          "\n-----\n".join([c["text"] for c in retrieved_contexts]) + "\n\n"
            sources = list(set([f"{c['source']} - {c['chapter']} - {c['section']}" for c in retrieved_contexts]))
            logger.debug(f"Retrieved {len(retrieved_contexts)} contexts. Sources: {sources}")
        else:
            logger.info("No relevant contexts retrieved from Qdrant.")

        # Prepare message for AI, incorporating RAG context and selected_text if present
        # The prompt engineering for Gemini needs to be strict to use the context.
        rag_prompt = f"Given the following context, please answer the question. If the answer cannot be found in the context, please state 'I don't have enough information to answer that.' Do not make up an answer.{context_str}\n\nQuestion: {request.message}"
        if request.selected_text:
            rag_prompt = f"Given the following context and specific focus text, please answer the question. If the answer cannot be found in the context or focus text, please state 'I don't have enough information to answer that.' Do not make up an answer.\n\n### Focus Text:\n'{request.selected_text}'{context_str}\n\nQuestion: {request.message}"
            logger.debug(f"Selected text incorporated into RAG prompt.")

        # Generate AI response
        system_prompt = gemini_client.construct_system_prompt()
        # Ensure the prompt passed to generate_response is the RAG-augmented prompt
        ai_response_content = await gemini_client.generate_response(system_prompt, rag_prompt, conversation_history)
        logger.info(f"AI response generated for session {session_id}")

        # The ChatResponse currently only has session_id and response.
        # We will need to update src/schemas.py to include sources.
        # For now, we'll return just session_id and response, but keep sources tracked.

        # Save AI response
        await message_repository.save_ai_message(session_id, ai_response_content)
        logger.info(f"AI response saved for session {session_id}")

        # Modify return statement to include sources (will require schema update)
        # return ChatResponse(session_id=session_id, response=ai_response_content, sources=sources)
        return ChatResponse(session_id=session_id, response=ai_response_content, sources=sources)
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