# backend/src/services/rag_service.py

import os
from typing import List, Dict, Optional
import google.generativeai as genai
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class RAGService:
    def __init__(self):
        self.qdrant_client = self._initialize_qdrant_client()
        self.embedding_model = "models/text-embedding-004" # Gemini embedding model for gemini-2.5-flash
        genai.configure(api_key=os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY"))

    def _initialize_qdrant_client(self):
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not qdrant_url or not qdrant_api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in environment variables.")

        return QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

    async def _generate_query_embedding(self, query: str) -> List[float]:
        """
        Generates an embedding for a single query string using Gemini's API.
        """
        if not query:
            return []
        try:
            response = await genai.embed_content_async(
                model=self.embedding_model,
                content=query,
                task_type="RETRIEVAL_QUERY" # Specify task type for better embeddings
            )
            return response['embedding']
        except Exception as e:
            print(f"Error generating query embedding with Gemini API: {e}")
            raise

    def _build_qdrant_filter(self, selected_text_hint: Optional[str]) -> Optional[models.Filter]:
        """
        Parses selected_text_hint to construct Qdrant filter clauses.
        For the MVP, this function is a placeholder. If a structured hint is provided
        (e.g., "chapter: Introduction to AI"), future implementations would parse this
        to create Qdrant filters based on payload fields like 'chapter', 'section', 'source'.

        For now, `selected_text_hint` is primarily intended to augment the `query`
        for better embedding generation or as a re-ranking signal if more advanced
        retrieval strategies are implemented.
        """
        # Example of how a filter *could* be built if a structured hint was present:
        # if selected_text_hint and "chapter:" in selected_text_hint.lower():
        #     chapter_name = selected_text_hint.split("chapter:", 1)[1].strip()
        #     return models.Filter(
        #         must=[
        #             models.FieldCondition(
        #                 key="chapter",
        #                 match=models.MatchText(text=chapter_name)
        #             )
        #         ]
        #     )
        return None

    async def retrieve_relevant_context(
        self, query: str, limit: int = 3, selected_text_hint: Optional[str] = None
    ) -> List[Dict]:
        """
        Retrieves relevant text chunks from Qdrant based on a query and an optional hint.
        """
        if not query:
            return []

        query_embedding = await self._generate_query_embedding(query)
        
        # Build filter from hint if available
        qdrant_filter = self._build_qdrant_filter(selected_text_hint)

        search_result = self.qdrant_client.search(
            collection_name=os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_textbook"),
            query_vector=query_embedding,
            query_filter=qdrant_filter,
            limit=limit,
            with_payload=True,
            with_vectors=False,
        )

        relevant_contexts = []
        for hit in search_result:
            payload = hit.payload
            relevant_contexts.append({
                "text": payload.get("text"),
                "source": payload.get("source"),
                "chapter": payload.get("chapter"),
                "section": payload.get("section"),
                "score": hit.score
            })
        return relevant_contexts

