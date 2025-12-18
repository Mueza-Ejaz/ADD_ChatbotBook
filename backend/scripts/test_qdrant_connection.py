import os
import asyncio
from qdrant_client import QdrantClient, models

async def test_qdrant_connection():
    QDRANT_URL = os.getenv("QDRANT_URL")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

    if not QDRANT_URL or not QDRANT_API_KEY:
        print("QDRANT_URL and QDRANT_API_KEY environment variables must be set.")
        return

    client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
    )

    collection_name = "test_collection_gemini_cli"

    print(f"Attempting to connect to Qdrant at {QDRANT_URL}...")
    try:
        # Check if collection exists, delete if it does
        if client.collection_exists(collection_name=collection_name):
            print(f"Collection '{collection_name}' already exists, deleting...")
            client.delete_collection(collection_name=collection_name)
            print(f"Collection '{collection_name}' deleted.")

        # Create a dummy collection
        print(f"Creating collection '{collection_name}'...")
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
        )
        print(f"Collection '{collection_name}' created successfully.")

        # List collections to verify
        collections = client.get_collections()
        collection_names = [c.name for c in collections.collections]
        print(f"Collections found: {collection_names}")
        assert collection_name in collection_names, f"Collection '{collection_name}' not found in list."

        # Delete the dummy collection
        print(f"Deleting collection '{collection_name}'...")
        client.delete_collection(collection_name=collection_name)
        print(f"Collection '{collection_name}' deleted successfully.")

        print("\nQdrant connection and basic operations successful!")
    except Exception as e:
        print(f"\nError connecting to Qdrant or performing operations: {e}")

if __name__ == "__main__":
    # Load environment variables for local testing
    from dotenv import load_dotenv
    load_dotenv(os.path.join(os.path.dirname(__file__), '..', '.env'))
    
    asyncio.run(test_qdrant_connection())
