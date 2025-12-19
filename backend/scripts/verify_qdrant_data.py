import os
import asyncio
from dotenv import load_dotenv
from qdrant_client import QdrantClient, models

async def verify_qdrant_data(collection_name: str):
    load_dotenv(os.path.join(os.path.dirname(__file__), '..', '.env'))

    QDRANT_URL = os.getenv("QDRANT_URL")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

    if not all([QDRANT_URL, QDRANT_API_KEY]):
        print("Error: Missing QDRANT_URL or QDRANT_API_KEY in environment variables.")
        return

    client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
    )

    if not client.collection_exists(collection_name=collection_name):
        print(f"Collection '{collection_name}' does not exist.")
        return

    print(f"Verifying data in collection '{collection_name}'...")

    try:
        # Get count of points
        count_result = client.count(
            collection_name=collection_name,
            exact=True
        )
        print(f"Number of points in collection: {count_result.count}")

        if count_result.count == 0:
            print("No points found in the collection.")
            return

        # Retrieve a sample point (e.g., the first one found)
        scroll_result, _ = client.scroll(
            collection_name=collection_name,
            limit=1,
            with_payload=True,
            with_vectors=False, # No need to retrieve vectors for verification
        )

        if scroll_result:
            print("\nSample Point Retrieved:")
            sample_point = scroll_result[0]
            print(f"  ID: {sample_point.id}")
            print(f"  Payload (text snippet): {sample_point.payload['text'][:200]}...") # Print first 200 chars
            print(f"  Source: {sample_point.payload['source']}")
            print(f"  Chapter: {sample_point.payload['chapter']}")
            print(f"  Section: {sample_point.payload['section']}")
        else:
            print("Could not retrieve a sample point.")

        # Optionally, perform a simple search to ensure embeddings work
        # For this, we'd need a query embedding, which requires a Gemini call.
        # Skipping for this verification script to keep it simple.

        print("\nQdrant data verification complete.")

    except Exception as e:
        print(f"An error occurred during data verification: {e}")

if __name__ == "__main__":
    COLLECTION_NAME = "physical_ai_textbook"
    asyncio.run(verify_qdrant_data(COLLECTION_NAME))
