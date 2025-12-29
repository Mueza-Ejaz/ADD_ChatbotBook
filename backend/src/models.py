import uuid
from datetime import datetime
from sqlalchemy import Column, DateTime, ForeignKey, Text, Enum
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship, Mapped, declarative_base

Base = declarative_base()

class ChatSession(Base):
    """Represents a chat session between a user and the AI."""
    __tablename__ = "chat_sessions"
    id: uuid.UUID = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    created_at: datetime = Column(DateTime, default=datetime.utcnow)
    updated_at: datetime = Column(DateTime, default=lambda: datetime.utcnow(), onupdate=lambda: datetime.utcnow()) # Use lambda for onupdate
    messages: Mapped[list["Message"]] = relationship("Message", back_populates="session")

class Message(Base):
    """Represents a message within a chat session."""
    __tablename__ = "messages"
    id: uuid.UUID = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id: uuid.UUID = Column(UUID(as_uuid=True), ForeignKey("chat_sessions.id"))
    role: str = Column(Enum("user", "assistant", "system", name="message_role_enum"), nullable=False)
    content: str = Column(Text, nullable=False)
    created_at: datetime = Column(DateTime, default=datetime.utcnow)
    session: Mapped["ChatSession"] = relationship("ChatSession", back_populates="messages")