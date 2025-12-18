# Research for Phase 5: RAG with Qdrant

This document addresses the "NEEDS CLARIFICATION" identified during the planning phase for Phase 5, focusing on Retrieval-Augmented Generation (RAG) with Qdrant.

## 1. Effective Text Chunking Strategies for Markdown

**Decision:** Employ a recursive character text splitter, prioritizing Markdown-specific separators such as headings, code blocks, and lists, before falling back to more generic separators like paragraphs and sentences. Incorporate a fixed-size chunking with overlap (e.g., ~300 words with 10-20% overlap) to maintain context across chunk boundaries.

**Rationale:** Markdown documents have inherent structural elements (headings, code blocks, lists) that denote semantic boundaries. Splitting along these lines first helps preserve the coherence of information. Recursive splitting allows for more granular control, breaking down larger sections into manageable chunks while respecting structure. Overlap is crucial to prevent loss of context at chunk edges during retrieval.

**Alternatives Considered:**
*   **Simple fixed-size chunking**: Less semantically aware, could split logical units of text.
*   **Paragraph-based chunking**: Better than fixed-size but might still split across small headings or code examples.
*   **Library-specific Markdown chunkers**: While potentially useful, a custom recursive approach offers more control over specific Docusaurus content structures.

## 2. OpenAI Embeddings API Integration Best Practices (Rate Limiting)

**Decision:** Implement an exponential backoff and retry mechanism for all OpenAI API calls, specifically targeting rate limit errors (HTTP 429). Additionally, for large-scale ingestion (like the sync tool), introduce a configurable delay (e.g., 0.1-0.5 seconds) between API requests to proactively reduce the chance of hitting rate limits. Consider batching embedding requests where possible to reduce overall API calls.

**Rationale:** OpenAI APIs have rate limits that can vary. Exponential backoff and retries are standard practice for handling transient errors and rate limits gracefully, preventing application crashes and ensuring eventual success. A proactive delay helps smooth out the request rate, especially during initial data ingestion. Batching reduces the number of network round trips and API overhead.

**Alternatives Considered:**
*   **Ignoring rate limits**: Leads to frequent failures and a poor user experience.
*   **Fixed delay only**: Might still hit rate limits if the fixed delay is too short or under variable load.
*   **Complex queueing systems**: Overkill for the initial phase, backoff/retry is sufficient.

## 3. Qdrant Client Python Library Usage (Specific Filter Syntax)

**Decision:** Use Qdrant's `FieldCondition` and `MatchValue` filters within a `Filter` clause to implement metadata-based searching. For complex scenarios involving `selected_text_hint`, parse the hint to extract potential `source`, `chapter`, or `section` identifiers. Combine these with logical `Must` and `Should` conditions as needed.

**Example Filter Structure for `selected_text_hint`:**

If `selected_text_hint` implies a specific `chapter` and `section`:

```python
from qdrant_client.http.models import Filter, FieldCondition, MatchValue, models

# Example filter construction
qdrant_filter = Filter(
    must=[
        FieldCondition(
            key="chapter",
            match=MatchValue(value="Introduction to Physical AI")
        ),
        FieldCondition(
            key="section",
            match=MatchValue(value="Historical Context")
        )
    ]
)
```

For cases where a hint might suggest multiple relevant chapters or sections, `Should` conditions can be used:

```python
qdrant_filter_should = Filter(
    should=[
        FieldCondition(key="chapter", match=MatchValue(value="Chapter 1")),
        FieldCondition(key="chapter", match=MatchValue(value="Chapter 2")),
    ],
    must=[
        # Add other global filters here if needed
    ]
)
```

**Rationale:** Qdrant's filtering capabilities are robust and allow for precise control over the search space. Using `FieldCondition` with `MatchValue` is the direct and idiomatic way to filter by exact metadata field values. Logical operators (`must`, `should`, ``not`) enable complex query construction.

**Alternatives Considered:**
*   **Post-retrieval filtering**: Less efficient as it requires retrieving more data than necessary from Qdrant and filtering in application code.
*   **Storing concatenated metadata for search**: Reduces granularity and flexibility for filtering.

## 4. Prompt Engineering for RAG (Optimal Phrasing for Strict System Prompt)

**Decision:** Adopt a clear, concise, and restrictive system prompt. Emphasize the persona, the constraint to *only* use provided context, and a directive for what to do if the answer is not found.

**Optimal System Prompt Template:**

```
"You are an expert assistant specialized in the 'Physical AI & Humanoid Robotics' textbook. Your sole purpose is to answer the user's question accurately and concisely, *strictly* based on the context provided below.

Context from the 'Physical AI & Humanoid Robotics' textbook:
---
{retrieved_passages_concatenated}
---

If the answer to the question cannot be found within the provided context, you *must* respond with: 'I am sorry, but I cannot find the answer to your question in the provided textbook content. Please try rephrasing your question or provide more details.' Do not use any outside knowledge.
"
```

**Rationale:** This prompt template:
*   Establishes a clear persona ("expert assistant specialized in...")
*   Explicitly states the constraint ("*strictly* based on the context provided below")
*   Provides a precise fallback response when information is absent ("I am sorry, but I cannot find...")
*   Reduces the likelihood of hallucination by strictly forbidding external knowledge.

**Alternatives Considered:**
*   **Less strict prompts**: Increases the risk of the LLM generating answers outside the provided context.
*   **No fallback instruction**: LLM might hallucinate an answer or respond generically when the context is insufficient.
*   **Overly verbose prompts**: Can consume more tokens and potentially dilute the core instructions.