import os
import re # Added import for regular expressions
from typing import List, Dict, Optional, Any
import google.generativeai as genai
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv

class RAGService:
    def __init__(self, collection_name: str = "physical_ai_textbook"):
        load_dotenv() # Load environment variables
        self.collection_name = collection_name
        self.qdrant_client = self._initialize_qdrant_client()
        self.gemini_api_key = os.getenv("GEMINI_API_KEY")
        genai.configure(api_key=self.gemini_api_key)
        self.embedding_model = "models/text-embedding-004"

    def _initialize_qdrant_client(self) -> QdrantClient:
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        if not qdrant_url or not qdrant_api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in environment variables.")
        return QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

    async def _generate_query_embedding(self, query: str) -> List[float]:
        """
        Generates an embedding for a single query string using Gemini's API.
        """
        if not self.gemini_api_key:
            raise ValueError("GEMINI_API_KEY is not set.")
        
        try:
            response = await genai.embed_content_async(
                model=self.embedding_model,
                content=query
            )
            return response['embedding']
        except Exception as e:
            print(f"Error generating embedding with Gemini API: {e}")
            raise

    def _build_qdrant_filter(self, selected_text_hint: Optional[str]) -> Optional[models.Filter]:
        """
        Parses selected_text_hint and constructs Qdrant filter clauses.
        For now, this is a placeholder. More sophisticated parsing would be needed.
        """
        if selected_text_hint:
            # Simple example: if hint contains "chapter 1", filter by chapter 1
            # In a real application, this would involve NLP to extract entities
            # For this MVP, we'll assume hints might contain 'chapter N' or 'section N.N'
            if "chapter" in selected_text_hint.lower():
                chapter_match = self._extract_chapter_from_hint(selected_text_hint)
                if chapter_match:
                    return models.Filter(
                        must=[
                            models.FieldCondition(
                                key="chapter",
                                match=models.MatchText(text=chapter_match) # Use MatchText for partial matches
                            )
                        ]
                    )
            elif "section" in selected_text_hint.lower():
                section_match = self._extract_section_from_hint(selected_text_hint)
                if section_match:
                    return models.Filter(
                        must=[
                            models.FieldCondition(
                                key="section",
                                match=models.MatchText(text=section_match)
                            )
                        ]
                    )
        return None

    def _extract_chapter_from_hint(self, hint: str) -> Optional[str]:
        match = re.search(r"chapter (\d+)", hint, re.IGNORECASE)
        if match:
            return f"Chapter {match.group(1)}"
        return None

    def _extract_section_from_hint(self, hint: str) -> Optional[str]:
        match = re.search(r"section (\d+(\.\d+)*)", hint, re.IGNORECASE)
        if match:
            return f"Section {match.group(1)}"
        return None

    async def retrieve_relevant_context(
        self, query: str, limit: int = 3, selected_text_hint: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Retrieves relevant text chunks from Qdrant based on the query and an optional text hint.
        """
        query_embedding = await self._generate_query_embedding(query)
        
        qdrant_filter = self._build_qdrant_filter(selected_text_hint)

        search_result = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            query_filter=qdrant_filter,
            limit=limit,
            with_payload=True,
            with_vectors=False # We don't need the vectors back for context retrieval
        )

        context = []
        for hit in search_result:
            context.append(
                {
                    "text": hit.payload["text"],
                    "source": hit.payload["source"],
                    "chapter": hit.payload.get("chapter"),
                    "section": hit.payload.get("section"),
                    "score": hit.score # Include score for debugging/analysis
                }
            )
        return context
