import os
from typing import List, Optional
import google.generativeai as genai
from qdrant_client import QdrantClient, models

class RAGService:
    def __init__(self):
        gemini_api_key = os.getenv("GEMINI_API_KEY")
        if not gemini_api_key:
            raise ValueError("GEMINI_API_KEY environment variable not set.")
        genai.configure(api_key=gemini_api_key)
        self.embedding_model = "models/text-embedding-004" # Using the same model as in sync tool

        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        if not qdrant_url or not qdrant_api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables must be set.")
        self.qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        self.collection_name = "physical_ai_textbook" # Same as in sync tool

    async def _generate_query_embedding(self, query: str) -> List[float]:
        """
        Generates an embedding for a single query string using Gemini's API.
        """
        try:
            response = await genai.embed_content_async(
                model=self.embedding_model,
                content=query
            )
            return response['embedding']
        except Exception as e:
            print(f"Error generating query embedding with Gemini API: {e}")
            raise # Re-raise the exception for proper error handling upstream

    def _build_qdrant_filter(self, selected_text_hint: Optional[str]) -> Optional[models.Filter]:
        """
        Parses selected_text_hint to construct Qdrant filter clauses.
        For example, a hint like "Chapter 1: Introduction" could filter by chapter.
        """
        if not selected_text_hint:
            return None

        # Simple keyword-based parsing for now. Can be expanded with more sophisticated NLP.
        filters = []
        # Example: if hint contains "Chapter X", filter by chapter
        chapter_match = re.search(r"(Chapter\s\d+.*)", selected_text_hint, re.IGNORECASE)
        if chapter_match:
            chapter_name = chapter_match.group(1).strip()
            filters.append(
                models.FieldCondition(
                    key="chapter",
                    match=models.MatchValue(value=chapter_name)
                )
            )

        # Example: if hint contains "Section Y", filter by section
        section_match = re.search(r"(Section\s\d+.*)", selected_text_hint, re.IGNORECASE)
        if section_match:
            section_name = section_match.group(1).strip()
            filters.append(
                models.FieldCondition(
                    key="section",
                    match=models.MatchValue(value=section_name)
                )
            )

        if filters:
            return models.Filter(must=filters)
        return None

    async def retrieve_relevant_context(self, query: str, limit: int = 3, selected_text_hint: Optional[str] = None) -> List[dict]:
        """
        Retrieves relevant text chunks from Qdrant based on the query and an optional filter.
        """
        query_embedding = await self._generate_query_embedding(query)
        qdrant_filter = self._build_qdrant_filter(selected_text_hint)

        search_result = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            query_filter=qdrant_filter,
            limit=limit,
            with_payload=True,
            with_vectors=False, # No need to retrieve vectors in the result
        )

        relevant_contexts = []
        for hit in search_result:
            if hit.payload:
                relevant_contexts.append({
                    "text": hit.payload.get("text"),
                    "source": hit.payload.get("source"),
                    "chapter": hit.payload.get("chapter"),
                    "section": hit.payload.get("section"),
                    "score": hit.score # Include similarity score
                })
        return relevant_contexts

