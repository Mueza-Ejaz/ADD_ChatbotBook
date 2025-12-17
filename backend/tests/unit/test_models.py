import pytest
from datetime import datetime
from uuid import UUID
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, Session

from src.models import Base, ChatSession, Message

# Use an in-memory SQLite database for testing
DATABASE_URL = "sqlite:///:memory:"

@pytest.fixture(name="session")
def session_fixture():
    engine = create_engine(DATABASE_URL, echo=False)
    Base.metadata.create_all(engine)

    TestingSessionLocal = sessionmaker(
        autocommit=False, autoflush=False, bind=engine
    )
    with TestingSessionLocal() as session:
        yield session

    Base.metadata.drop_all(engine)


def test_create_chat_session(session: Session):
    new_session = ChatSession()
    session.add(new_session)
    session.commit()
    session.refresh(new_session)

    assert isinstance(new_session.id, UUID)
    assert new_session.created_at is not None
    assert new_session.updated_at is not None
    assert new_session.created_at == new_session.updated_at

def test_create_message(session: Session):
    chat_session = ChatSession()
    session.add(chat_session)
    session.commit()
    session.refresh(chat_session)

    new_message = Message(
        session_id=chat_session.id,
        role="user",
        content="Hello AI"
    )
    session.add(new_message)
    session.commit()
    session.refresh(new_message)

    assert isinstance(new_message.id, UUID)
    assert new_message.session_id == chat_session.id
    assert new_message.role == "user"
    assert new_message.content == "Hello AI"
    assert new_message.created_at is not None

def test_chat_session_message_relationship(session: Session):
    chat_session = ChatSession()
    session.add(chat_session)
    session.commit()
    session.refresh(chat_session)

    message1 = Message(session_id=chat_session.id, role="user", content="Hi")
    message2 = Message(session_id=chat_session.id, role="assistant", content="Hello")
    session.add_all([message1, message2])
    session.commit()
    
    session.refresh(chat_session, attribute_names=["messages"])
    
    assert len(chat_session.messages) == 2
    assert chat_session.messages[0].content == "Hi"
    assert chat_session.messages[1].content == "Hello"