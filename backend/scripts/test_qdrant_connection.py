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

    collection_name = "physical_ai_textbook"

    print(f"Attempting to connect to Qdrant at {QDRANT_URL}...")
    try:
        if not client.collection_exists(collection_name=collection_name):
            print(f"Collection '{collection_name}' does not exist. Please run the sync tool first.")
            return

        print(f"Fetching 1 sample point from collection '{collection_name}'...")
        # Fetch a sample point
        search_result = client.scroll(
            collection_name=collection_name,
            limit=1,
            with_payload=True,
            with_vectors=True
        )

        if search_result and search_result[0]:
            sample_point = search_result[0][0] # scroll returns (list_of_points, next_page_offset)
            print("\nSuccessfully fetched a sample point:")
            print(f"  ID: {sample_point.id}")
            print(f"  Payload: {sample_point.payload}")
            print(f"  Vector size: {len(sample_point.vector) if sample_point.vector else 'N/A'}")
            print(f"  First 5 vector dimensions: {sample_point.vector[:5] if sample_point.vector else 'N/A'}")
            print("\nQdrant connection and sample point verification successful!")
        else:
            print(f"No points found in collection '{collection_name}'. Please ensure data has been synced.")

    except Exception as e:
        print(f"\nError connecting to Qdrant or performing operations: {e}")

if __name__ == "__main__":
    # Load environment variables for local testing
    from dotenv import load_dotenv
    load_dotenv(os.path.join(os.path.dirname(__file__), '..', '.env'))
    
    asyncio.run(test_qdrant_connection())
