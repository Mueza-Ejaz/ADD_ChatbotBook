from pydantic import BaseModel, Field, conlist
from typing import Optional
from datetime import datetime, timezone # Import timezone
from uuid import UUID

class ChatRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=1000)
    session_id: Optional[UUID] = None
    selected_text: Optional[str] = Field(None, min_length=10, max_length=2000)

class ChatResponse(BaseModel):
    response: str = Field(..., min_length=1, max_length=2000)
    session_id: UUID
    timestamp: datetime = Field(default_factory=lambda: datetime.now(timezone.utc)) # Use timezone.utc

class HealthResponse(BaseModel):
    status: str = Field(..., min_length=1, max_length=50)
