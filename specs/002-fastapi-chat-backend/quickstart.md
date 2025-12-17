# Quickstart Guide: Minimalistic FastAPI Chat Backend

**Feature Branch**: `002-fastapi-chat-backend`
**Created**: 2025-12-17

This guide provides instructions to quickly set up and run the Minimalistic FastAPI Chat Backend for local development.

## Prerequisites

-   Python 3.12+ installed.
-   `uv` or `poetry` for package management (recommended). If not, `pip` will work.
-   Git installed.
-   Access to a Neon PostgreSQL instance and its connection string.
-   An OpenAI API Key.

## Setup Instructions

1.  **Clone the Repository**:
    ```bash
    git clone <repository-url>
    cd <your-repository-directory>
    git checkout 002-fastapi-chat-backend
    ```

2.  **Create a Virtual Environment and Install Dependencies**:

    *   **Using `uv` (recommended)**:
        ```bash
        uv venv
        uv pip install -r requirements.txt # (assuming requirements.txt will be generated)
        # OR if pyproject.toml is used:
        # uv pip install -e .
        ```
    *   **Using `poetry`**:
        ```bash
        poetry install
        poetry shell
        ```
    *   **Using `pip`**:
        ```bash
        python -m venv venv
        .\venv\Scripts\activate # On Windows
        source venv/bin/activate # On macOS/Linux
        pip install -r requirements.txt # (assuming requirements.txt will be generated)
        ```

3.  **Configure Environment Variables**:
    Create a `.env` file in the project root with the following content. Replace placeholders with your actual credentials and allowed frontend URLs.

    ```dotenv
    OPENAI_API_KEY="your_openai_api_key_here"
    DATABASE_URL="your_neon_postgresql_connection_string_here"
    CORS_ORIGINS="http://localhost:3000,https://your-frontend-deployment.com" # Example: http://localhost:3000 for Docusaurus frontend
    ```

4.  **Run Database Migrations (Alembic)**:
    Ensure your `DATABASE_URL` is correctly set in `.env`.
    ```bash
    # If using poetry
    poetry run alembic upgrade head
    # If using virtual environment directly
    alembic upgrade head
    ```
    *(Note: Alembic configuration and initial migration scripts will be set up during Milestone 2 implementation.)*

5.  **Start the FastAPI Application**:
    ```bash
    # If using poetry
    poetry run uvicorn src.main:app --reload
    # If using virtual environment directly
    uvicorn src.main:app --reload
    ```
    The application will be accessible at `http://localhost:8000`.

## Testing the Health Endpoint

Once the application is running, open your web browser or use a tool like `curl` to check the health endpoint:

```bash
curl http://localhost:8000/health
```
You should receive a JSON response: `{"status": "healthy"}`.

## Next Steps

-   **Explore API Documentation**: Access the interactive API documentation at `http://localhost:8000/docs` (Swagger UI) or `http://localhost:8000/redoc` (ReDoc) to test the `/chat` endpoint.
-   **Integrate Frontend**: Connect this backend with your Docusaurus frontend (Phase 3).
