from datetime import datetime
from uuid import UUID, uuid4
import pytest
from pydantic import ValidationError

from src.schemas import ChatRequest, ChatResponse, HealthResponse

def test_chat_request_valid():
    # Test valid ChatRequest
    request = ChatRequest(message="Hello", session_id=uuid4(), selected_text="Some text")
    assert request.message == "Hello"
    assert isinstance(request.session_id, UUID)
    assert request.selected_text == "Some text"

def test_chat_request_no_session_id_no_selected_text():
    # Test ChatRequest with no optional fields
    request = ChatRequest(message="Hello")
    assert request.message == "Hello"
    assert request.session_id is None
    assert request.selected_text is None

def test_chat_request_empty_message():
    # Test ChatRequest with empty message (should raise ValidationError)
    with pytest.raises(ValidationError):
        ChatRequest(message="")

def test_chat_response_valid():
    # Test valid ChatResponse
    session_id = uuid4()
    timestamp = datetime.utcnow()
    response = ChatResponse(response="AI Reply", session_id=session_id, timestamp=timestamp)
    assert response.response == "AI Reply"
    assert isinstance(response.session_id, UUID)
    assert response.timestamp == timestamp

def test_health_response_valid():
    # Test valid HealthResponse
    response = HealthResponse(status="healthy")
    assert response.status == "healthy"
