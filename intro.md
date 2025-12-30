# Physical AI & Humanoid Robotics Textbook

This project is the companion website and backend API for the **"Physical AI & Humanoid Robotics"** textbook. It provides an interactive learning experience combining static book content with an AI-powered chatbot capable of answering questions based on the book's content using **Retrieval-Augmented Generation (RAG)**.

## 🌟 Features

### Frontend (Docusaurus)
- **Static Site Generation**: Fast, SEO-friendly documentation and blog.
- **Book Chapters & Tutorials**: Structured content for easy navigation.
- **Interactive Chatbot UI**: User interface to interact with the AI assistant.
- **Selected Text Integration**: Select text from the book to provide context to the chatbot.

### Backend (FastAPI with RAG)
- **Chat Sessions**: Persistent chat sessions with PostgreSQL database.
- **Google Gemini API Integration**: Generate AI responses using Google's Gemini models.
- **Retrieval-Augmented Generation (RAG)**: Retrieve relevant content to enhance AI responses.
- **Qdrant Vector Database**: Efficient storage and semantic search for document embeddings.
- **Data Ingestion API**: Secure endpoint to sync book content into Qdrant.
- **Asynchronous Operations**: Non-blocking I/O for database and API calls.
- **Structured Logging**: Configurable logging with Loguru.
- **Pydantic Validation**: Robust input/output validation.
- **Comprehensive Error Handling**
- **Database Migrations**: Managed with Alembic.

## 🛠 Technologies Used

### Frontend
- Docusaurus
- React
- TypeScript
- SCSS Modules

### Backend
- FastAPI
- Python
- Google Gemini API
- Qdrant
- SQLAlchemy (Async)
- Alembic
- Poetry
- Loguru
- Pydantic

```bash

├── backend/ # FastAPI backend
│ ├── alembic/ # Migrations
│ ├── src/
│ │ ├── config/ # Config files
│ │ ├── database.py # DB engine & session
│ │ ├── exceptions.py # Custom exceptions
│ │ ├── main.py # FastAPI app
│ │ ├── models.py # ORM models
│ │ ├── repositories/ # DB repos
│ │ ├── schemas.py # Pydantic models
│ │ └── services/ # Business logic & external APIs
│ └── tests/ # Unit & integration tests
├── frontend/ # Docusaurus frontend
│ ├── docs/ # Book chapters/tutorials
│ ├── blog/ # Blog posts
│ ├── src/
│ │ ├── components/ # React components (e.g., Chatbot)
│ │ ├── css/ # Custom CSS
│ │ ├── hooks/
│ │ ├── pages/ # Docusaurus pages
│ │ └── theme/ # Theme overrides
│ └── docusaurus.config.ts # Docusaurus config
├── tools/ # Utility scripts
└── specs/ # Design docs
```

## ⚡ Setup Instructions

### Prerequisites
- Git
- Node.js & npm/yarn
- Python 3.12+
- Poetry (`pip install poetry`)
- PostgreSQL Database
- Qdrant Instance
- Google Gemini API Key

## 1. Clone the Repository

```bash
git clone <repository_url>
cd <repository_name>
```

## 2. Backend Setup
cd backend
poetry install

### - Environment Variables:
Create .env file (based on .env.example or manually)

```bash
DATABASE_URL="postgresql+asyncpg://USER:PASSWORD@HOST:PORT/DB_NAME?sslmode=require"
GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
QDRANT_URL="YOUR_QDRANT_URL"
QDRANT_API_KEY="YOUR_QDRANT_API_KEY"
INGESTION_API_KEY="YOUR_INGESTION_API_KEY"
FRONTEND_URL="http://localhost:3000"
```
- Run Database Migrations: poetry run alembic upgrade head

## 3. Frontend Setup
cd ../frontend

npm install # or yarn install

▶️ Running Locally
Backend

cd backend

poetry run uvicorn src.main:app --reload --host 0.0.0.0 --port 8000

API docs: http://localhost:8000/docs

Frontend:
cd frontend
npm start
Frontend: http://localhost:3000

🔗 API Endpoints

GET /health – Check API health

POST /chat – Chat interactions with RAG

POST /ingest – Sync book content to Qdrant (requires X-API-Key)


### - Data Ingestion Example:

curl -X POST http://localhost:8000/ingest

 -H "X-API-Key: YOUR_INGESTION_API_KEY"
 

✅ Testing & Code Quality:

Backend Tests
cd backend
poetry run pytest

Linting & Formatting:

poetry run ruff check .

poetry run black .

poetry run ruff check . --fix # auto fix linting

🌐 Live Website

Frontend deployed on Vercel: https://add-chatbot-book-hbye.vercel.app/
