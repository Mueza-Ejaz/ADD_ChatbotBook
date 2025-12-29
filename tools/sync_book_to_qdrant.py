import os
import asyncio
from typing import List, Dict, Optional
import re
import google.generativeai as genai
from qdrant_client import QdrantClient, models # Import Qdrant client

import argparse
from dotenv import load_dotenv
import glob # For finding markdown files
from loguru import logger # Add logger import

# Assume markdown-it-py is available from the backend environment
try:
    from markdown_it import MarkdownIt
except ImportError:
    print("markdown-it-py not found. Please install it: poetry add markdown-it-py")
    exit(1)


class TextChunker:
    """
    Splits Markdown text into coherent chunks, respecting Markdown structure.
    """
    def __init__(self, chunk_size: int = 300, chunk_overlap: int = 50):
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap
        self.md = MarkdownIt()

    def _split_by_markdown_sections(self, text: str) -> List[str]:
        """
        Splits text by Markdown headings, trying to keep sections together.
        """
        # Split by H1, H2, H3 headings and other major blocks
        # This is a heuristic, more advanced parsing can be done with markdown-it-py tokens
        sections = re.split(r'(^#+\s.*$|^```.*?```)', text, flags=re.MULTILINE | re.DOTALL)
        
        # Filter out empty strings and re-add delimiters where appropriate
        processed_sections = []
        for i, section in enumerate(sections):
            if not section.strip():
                continue
            
            # If the section starts with a heading or code block, it's a delimiter
            if section.startswith('#') or section.startswith('```'):
                if i > 0 and processed_sections: # Add to previous section if it's a header
                     processed_sections[-1] += "\n" + section.strip()
                else: # Otherwise, treat as new section
                    processed_sections.append(section.strip())
            else:
                processed_sections.append(section.strip())

        return [s for s in processed_sections if s]


    def chunk_text(self, text: str, source_path: str) -> List[Dict]:
        """
        Chunks the input text into smaller, overlapping segments suitable for embedding.
        Each chunk is augmented with metadata.
        """
        chunks = []
        doc_sections = self._split_by_markdown_sections(text)
        
        current_chapter = "Unknown Chapter"
        current_section = "Unknown Section"

        for section_text in doc_sections:
            lines = section_text.split('\n')
            
            # Update chapter/section based on headings
            if lines:
                first_line = lines[0].strip()
                if first_line.startswith('# '):
                    current_chapter = first_line[2:].strip()
                    current_section = "Introduction" # Reset section for new chapter
                elif first_line.startswith('## '):
                    current_section = first_line[3:].strip()
                elif first_line.startswith('### '):
                    current_section = first_line[4:].strip()

            # Further split by paragraphs for finer granularity if section is too large
            paragraphs = [p.strip() for p in section_text.split('\n\n') if p.strip()]
            combined_paragraphs = ""
            
            for para in paragraphs:
                if not combined_paragraphs:
                    combined_paragraphs = para
                elif len(combined_paragraphs.split()) + len(para.split()) <= self.chunk_size:
                    combined_paragraphs += "\n\n" + para
                else:
                    chunks.append(self._create_chunk_dict(combined_paragraphs, source_path, current_chapter, current_section))
                    combined_paragraphs = para # Start new chunk with current paragraph

            if combined_paragraphs: # Add any remaining text
                chunks.append(self._create_chunk_dict(combined_paragraphs, source_path, current_chapter, current_section))

        # Further refine chunks if they are too long or too short, adding overlap
        final_chunks = []
        for chunk_dict in chunks:
            text = chunk_dict['text']
            words = text.split()
            if len(words) > self.chunk_size * 1.5: # If a chunk is significantly larger, try to split more
                # Simple word-based splitting with overlap for large chunks
                for i in range(0, len(words), self.chunk_size - self.chunk_overlap):
                    sub_chunk_words = words[i:i + self.chunk_size]
                    sub_chunk_text = " ".join(sub_chunk_words)
                    final_chunks.append(self._create_chunk_dict(sub_chunk_text, source_path, chunk_dict['chapter'], chunk_dict['section']))
            elif len(words) > 50: # Avoid very tiny chunks
                final_chunks.append(chunk_dict)
            # else: too small, might be noise, skip for now or merge with neighbors

        if not final_chunks and text.strip(): # Ensure at least one chunk if there's text
            final_chunks.append(self._create_chunk_dict(text, source_path, current_chapter, current_section))

        return final_chunks

    def _create_chunk_dict(self, text: str, source_path: str, chapter: str, section: str) -> Dict:
        return {
            "text": text.strip(),
            "source": source_path,
            "chapter": chapter,
            "section": section,
        }


class EmbeddingGenerator:
    """
    Generates embeddings for text chunks using Gemini's API, with retry logic.
    """
    def __init__(self, model: str = "models/text-embedding-004", api_key: Optional[str] = None):
        self.model = model
        # Configure Gemini API key
        genai.configure(api_key=api_key or os.getenv("GEMINI_API_KEY"))

    # Gemini API for embeddings does not currently have RateLimitError or APIError
    # defined in a way that backoff can catch directly as it does for OpenAI.
    # We'll use a generic Exception for now, or implement custom retry logic if needed.
    # @backoff.on_exception(backoff.expo, (RateLimitError, APIError), max_tries=5, factor=2)
    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generates an embedding for a single text string.
        """
        try:
            response = await genai.embed_content_async(
                model=self.model,
                content=text
            )
            return response['embedding']
        except Exception as e:
            print(f"Error generating embedding with Gemini API: {e}")
            raise # Re-raise to be caught by eventual backoff wrapper if applied externally

    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generates embeddings for a list of text strings in a single API call (batching).
        """
        if not texts:
            return []
        
        logger.debug(f"Attempting to generate embeddings for {len(texts)} texts. First text snippet: '{texts[0][:100]}...'")
        
        try:
            # Gemini's embed_content_async can take a list of contents
            response = await genai.embed_content_async(
                model=self.model,
                content=texts
            )
            return response['embedding']

        except Exception as e:
            print(f"Error generating batch embeddings with Gemini API: {e}")
            raise # Re-raise

class QdrantIngestor:
    """
    Handles connection to Qdrant and upserting of points with embeddings and metadata.
    """
    def __init__(self, collection_name: str, qdrant_url: Optional[str] = None, qdrant_api_key: Optional[str] = None):
        self.collection_name = collection_name
        self.client = QdrantClient(
            url=qdrant_url or os.getenv("QDRANT_URL"),
            api_key=qdrant_api_key or os.getenv("QDRANT_API_KEY"),
        )
        self.vector_size = 768 # For 'models/text-embedding-004' text embeddings
        self.distance_metric = models.Distance.COSINE

    async def ensure_collection_exists(self, force_recreate: bool = False):
        """
        Ensures the Qdrant collection exists, creating it if it doesn't.
        If force_recreate is True, it deletes and recreates the collection.
        """
        if force_recreate and self.client.collection_exists(collection_name=self.collection_name):
            print(f"Force recreating collection '{self.collection_name}', deleting existing...")
            self.client.delete_collection(collection_name=self.collection_name)
            print(f"Collection '{self.collection_name}' deleted.")

        if not self.client.collection_exists(collection_name=self.collection_name):
            print(f"Collection '{self.collection_name}' not found, creating...")
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=self.vector_size,
                    distance=self.distance_metric
                ),
            )
            print(f"Collection '{self.collection_name}' created.")
            # Verify collection configuration
            collection_info = self.client.get_collection(collection_name=self.collection_name).config
            if collection_info.params.vectors.size != self.vector_size:
                logger.error(f"Error: Collection '{self.collection_name}' created with incorrect vector size! Expected {self.vector_size}, got {collection_info.params.vectors.size}")
                # Optionally, raise an exception or attempt to fix
                raise ValueError("Collection created with incorrect vector size.")
            else:
                logger.info(f"Collection '{self.collection_name}' verified with correct vector size: {self.vector_size}.")

        else:
            print(f"Collection '{self.collection_name}' already exists.")

    # @backoff.on_exception(backoff.expo, (Exception), max_tries=5, factor=2) # Generic retry for now
    async def upsert_chunks(self, chunks_with_embeddings: List[Dict]):
        """
        Upserts a list of chunks, each with its embedding and metadata, to Qdrant.
        """
        if not chunks_with_embeddings:
            return

        points = []
        for i, chunk in enumerate(chunks_with_embeddings):
            # Qdrant point ID can be generated or explicit. Using a hash of text for now, or UUID
            # Using MD5 hash of text and source to create a stable ID for updates
            import hashlib
            unique_id_str = f"{chunk['source']}-{chunk['chapter']}-{chunk['section']}-{chunk['text']}"
            point_id = int(hashlib.md5(unique_id_str.encode()).hexdigest(), 16) % (10**18) # Convert to int within range
            
            points.append(
                models.PointStruct(
                    id=point_id,
                    vector=chunk['embedding'],
                    payload={
                        "text": chunk['text'],
                        "source": chunk['source'],
                        "chapter": chunk['chapter'],
                        "section": chunk['section'],
                    },
                )
            )
        
        # Batch upsert
        print(f"Upserting {len(points)} points to collection '{self.collection_name}'...")
        response = self.client.upsert(
            collection_name=self.collection_name,
            wait=True,
            points=points,
        )
        print(f"Upsert response: {response}")
        # TODO: Handle response for errors/success more robustly


async def sync_book_to_qdrant(docs_path: str, collection_name: str, force_resync: bool = False):
    """
    Main function to chunk, embed, and upsert book content to Qdrant.
    """
    load_dotenv(os.path.join(os.path.dirname(__file__), '..', 'backend', '.env'))

    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY") 
    QDRANT_URL = os.getenv("QDRANT_URL")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

    if not all([GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY]):
        print("Error: Missing GEMINI_API_KEY/GOOGLE_API_KEY, QDRANT_URL, or QDRANT_API_KEY in environment variables.")
        return
    
    logger.info(f"Loaded GEMINI_API_KEY: {GEMINI_API_KEY[:4]}...{GEMINI_API_KEY[-4:]}")

    chunker = TextChunker()
    embedding_generator = EmbeddingGenerator(api_key=GEMINI_API_KEY)
    qdrant_ingestor = QdrantIngestor(
        collection_name=collection_name,
        qdrant_url=QDRANT_URL,
        qdrant_api_key=QDRANT_API_KEY
    )

    await qdrant_ingestor.ensure_collection_exists(force_recreate=force_resync)

    markdown_files = glob.glob(os.path.join(docs_path, '**', '*.md'), recursive=True)
    if not markdown_files:
        print(f"No markdown files found in {docs_path}")
        return

    all_chunks = []
    print(f"Found {len(markdown_files)} markdown files. Chunking content...")
    for file_path in markdown_files:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Normalize source path to be relative to project root or docs_path
        relative_source_path = os.path.relpath(file_path, os.getcwd()) # Use getcwd for consistency
        chunks = chunker.chunk_text(content, relative_source_path)
        all_chunks.extend(chunks)
    
    print(f"Generated {len(all_chunks)} text chunks.")

    if not all_chunks:
        print("No chunks generated, skipping embedding and ingestion.")
        return

    # Generate embeddings in batches
    BATCH_SIZE = 100 # Gemini API might have limits, or it's just more efficient
    chunks_with_embeddings = []
    for i in range(0, len(all_chunks), BATCH_SIZE):
        batch_chunks = all_chunks[i:i + BATCH_SIZE]
        texts_to_embed = [chunk['text'] for chunk in batch_chunks]
        print(f"Generating embeddings for batch {i//BATCH_SIZE + 1} ({len(texts_to_embed)} texts)...")
        embeddings_batch = await embedding_generator.generate_embeddings_batch(texts_to_embed)
        
        for j, chunk in enumerate(batch_chunks):
            chunk['embedding'] = embeddings_batch[j]
            chunks_with_embeddings.append(chunk)

    print(f"Generated embeddings for {len(chunks_with_embeddings)} chunks. Upserting to Qdrant...")
    await qdrant_ingestor.upsert_chunks(chunks_with_embeddings)
    print("Book content synchronization complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Synchronize book content to Qdrant vector database.")
    parser.add_argument("--docs-path", type=str, default="frontend/docs/",
                        help="Path to the Docusaurus 'docs' directory containing Markdown files.")
    parser.add_argument("--collection-name", type=str, default="physical_ai_textbook",
                        help="Name of the Qdrant collection.")
    parser.add_argument("--force-resync", action="store_true",
                        help="If set, deletes existing collection and recreates it before syncing.")
    
    args = parser.parse_args()

    asyncio.run(sync_book_to_qdrant(args.docs_path, args.collection_name, args.force_resync))

    