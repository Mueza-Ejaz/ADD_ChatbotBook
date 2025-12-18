import pytest
from httpx import AsyncClient
from unittest.mock import AsyncMock, patch, MagicMock
from src.main import app # Assuming main.py is in src directory for testing setup
import os

# Mock environment variables
@pytest.fixture(autouse=True)
def mock_env_vars():
    with patch.dict(os.environ, {
        "QDRANT_URL": "https://d6a06572-5a56-497a-9a2f-bdf94d137ce8.europe-west3-0.gcp.cloud.qdrant.io",
        "QDRANT_API_KEY": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.syw1JVbam6R7ON0iWTuMGpjDNYuiwUmwjwlm4fsUmyo",
        "GEMINI_API_KEY": "AIzaSyB8hcOSYiEjEYgh2n-kiH1NkZAI8QihTbQ",
        "QDRANT_COLLECTION_NAME": "test_collection",
        "INGESTION_API_KEY": "test-ingestion-key", # For ingest endpoint tests
        "FRONTEND_URL": "http://localhost:3000"
    }):
        yield

@pytest.fixture
async def async_client():
    async with AsyncClient(app=app, base_url="http://test") as client:
        yield client

@pytest.fixture
def mock_subprocess_exec():
    with patch('src.main.asyncio.create_subprocess_exec', new_callable=AsyncMock) as mock_exec:
        # Default successful execution
        mock_process = MagicMock()
        mock_process.returncode = 0
        mock_process.communicate = AsyncMock(return_value=(b"Script ran successfully", b""))
        mock_exec.return_value = mock_process
        yield mock_exec

@pytest.mark.asyncio
async def test_ingest_endpoint_success_no_resync(async_client: AsyncClient, mock_subprocess_exec: AsyncMock):
    """
    Test successful ingestion without force_resync.
    """
    response = await async_client.post(
        "/ingest",
        headers={"X-API-Key": "test-ingestion-key"},
        params={"force_resync": False}
    )

    assert response.status_code == 200
    assert "Ingestion process initiated successfully." in response.json()["status"]
    mock_subprocess_exec.assert_awaited_once()
    args, kwargs = mock_subprocess_exec.call_args
    assert "python" in args[0]
    assert "--force-resync" not in args

@pytest.mark.asyncio
async def test_ingest_endpoint_success_with_resync(async_client: AsyncClient, mock_subprocess_exec: AsyncMock):
    """
    Test successful ingestion with force_resync.
    """
    response = await async_client.post(
        "/ingest",
        headers={"X-API-Key": "test-ingestion-key"},
        params={"force_resync": True}
    )

    assert response.status_code == 200
    assert "Ingestion process initiated successfully." in response.json()["status"]
    mock_subprocess_exec.assert_awaited_once()
    args, kwargs = mock_subprocess_exec.call_args
    assert "--force-resync" in args

@pytest.mark.asyncio
async def test_ingest_endpoint_authentication_failure_missing_key(async_client: AsyncClient):
    """
    Test ingestion endpoint fails with missing API key.
    """
    response = await async_client.post("/ingest") # No X-API-Key header

    assert response.status_code == 422 # FastAPI's validation error for missing header

@pytest.mark.asyncio
async def test_ingest_endpoint_authentication_failure_invalid_key(async_client: AsyncClient):
    """
    Test ingestion endpoint fails with an invalid API key.
    """
    response = await async_client.post(
        "/ingest",
        headers={"X-API-Key": "wrong-key"}
    )

    assert response.status_code == 401
    assert "Invalid API Key" in response.json()["detail"]

@pytest.mark.asyncio
async def test_ingest_endpoint_script_execution_failure(async_client: AsyncClient, mock_subprocess_exec: AsyncMock):
    """
    Test ingestion endpoint handles failure of the sync script.
    """
    mock_process = MagicMock()
    mock_process.returncode = 1 # Simulate script failure
    mock_process.communicate = AsyncMock(return_value=(b"Error output", b"Script failed"))
    mock_subprocess_exec.return_value = mock_process

    response = await async_client.post(
        "/ingest",
        headers={"X-API-Key": "test-ingestion-key"}
    )

    assert response.status_code == 500
    assert "Ingestion script failed" in response.json()["detail"]
    assert "Script failed" in response.json()["detail"]
    mock_subprocess_exec.assert_awaited_once()

@pytest.mark.asyncio
async def test_ingest_endpoint_server_config_error(async_client: AsyncClient):
    """
    Test ingestion endpoint handles missing INGESTION_API_KEY environment variable.
    """
    with patch.dict(os.environ, {"INGESTION_API_KEY": ""}, clear=False): # Set to empty string to simulate missing
        response = await async_client.post(
            "/ingest",
            headers={"X-API-Key": "any-key"} # Key doesn't matter, it'll fail before check
        )
        assert response.status_code == 500
        assert "Server configuration error: Ingestion API key not set." in response.json()["detail"]
