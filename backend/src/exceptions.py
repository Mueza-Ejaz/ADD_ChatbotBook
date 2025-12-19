"""
Custom exception classes for the chat backend.

These exceptions inherit from HTTPException to provide standardized
HTTP responses for specific error conditions.
"""

from fastapi import HTTPException, status

class ChatSessionNotFoundException(HTTPException):
    """
    Raised when a requested chat session cannot be found.
    Corresponds to an HTTP 404 Not Found error.
    """
    def __init__(self, session_id: str):
        super().__init__(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Chat session with ID '{session_id}' not found."
        )

class GeminiAPIException(HTTPException):
    """
    Raised when an error occurs during interaction with the Gemini API.
    Corresponds to an HTTP 500 Internal Server Error by default.
    """
    def __init__(self, detail: str, status_code: int = status.HTTP_500_INTERNAL_SERVER_ERROR):
        super().__init__(
            status_code=status_code,
            detail=f"Gemini API error: {detail}"
        )

class DatabaseException(HTTPException):
    """
    Raised when a database-related error occurs.
    Corresponds to an HTTP 500 Internal Server Error by default.
    """
    def __init__(self, detail: str, status_code: int = status.HTTP_500_INTERNAL_SERVER_ERROR):
        super().__init__(
            status_code=status_code,
            detail=f"Database error: {detail}"
        )