# API Contracts: Content Embedding Pipeline

**Feature**: 002-content-embedding-pipeline
**Date**: 2025-12-17

## Overview

This document defines the internal function contracts for the embedding pipeline. The current implementation is a standalone script; these contracts define the function signatures and behaviors.

---

## Function: get_all_urls

**Purpose**: Return all book content URLs with metadata.

**Signature**:
```python
def get_all_urls() -> list[dict]:
    """
    Fetch all book content URLs from the deployed site.

    Returns:
        List of dicts with 'url', 'chapter', and 'section' metadata.
    """
```

**Return Schema**:
```python
[
    {
        "url": str,      # Full URL (e.g., "https://...hackathon1.vercel.app/docs/intro")
        "chapter": str,  # Chapter name (e.g., "Introduction", "Chapter 1")
        "section": str,  # Section title (e.g., "Start Reading")
    },
    # ... more entries
]
```

**Example Response**:
```python
[
    {"url": "https://speckitplus-hackathon1.vercel.app/docs/intro", "chapter": "Introduction", "section": "Start Reading"},
    {"url": "https://speckitplus-hackathon1.vercel.app/docs/chapter-1-introduction", "chapter": "Chapter 1", "section": "Introduction to Physical AI"},
]
```

**Error Handling**: Returns empty list if no URLs configured.

---

## Function: extract_text_from_url

**Purpose**: Fetch webpage and extract clean text content.

**Signature**:
```python
def extract_text_from_url(url: str) -> str:
    """
    Fetch and extract readable text content from a URL.

    Args:
        url: The webpage URL to extract text from.

    Returns:
        Extracted text content as a string. Empty string on error.
    """
```

**Input**:
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| url | string | Yes | Full HTTP(S) URL to fetch |

**Output**:
- Success: Cleaned text content (newline-separated paragraphs)
- Failure: Empty string `""`

**Behavior**:
1. Fetch URL with 30-second timeout
2. Parse HTML with BeautifulSoup
3. Remove `<script>`, `<style>`, `<nav>`, `<header>`, `<footer>`, `<aside>`
4. Find main content (`<main>`, `<article>`, `.markdown`, `.content`, or `<body>`)
5. Extract text with newline separators
6. Strip whitespace from each line

**Error Handling**:
- HTTP errors: Log error, return empty string
- Parse errors: Log error, return empty string
- Timeout: Log error, return empty string

---

## Function: chunk_text

**Purpose**: Split text into overlapping chunks for embedding.

**Signature**:
```python
def chunk_text(
    text: str,
    chunk_size: int = 500,
    overlap: int = 100
) -> list[str]:
    """
    Split text into overlapping chunks suitable for embedding.

    Args:
        text: The text to chunk.
        chunk_size: Target number of words per chunk (default: 500).
        overlap: Number of words to overlap between chunks (default: 100).

    Returns:
        List of text chunks. Empty list if input is empty.
    """
```

**Input**:
| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| text | string | Yes | - | Text to chunk |
| chunk_size | int | No | 500 | Words per chunk |
| overlap | int | No | 100 | Overlap between chunks |

**Output**:
- List of string chunks
- Each chunk is approximately `chunk_size` words
- Consecutive chunks share `overlap` words

**Behavior**:
1. Split text by whitespace into words
2. If total words ≤ chunk_size, return single chunk
3. Create sliding window with step = chunk_size - overlap
4. Return list of joined word chunks

**Example**:
```python
text = "word1 word2 word3 ... word1000"  # 1000 words
chunks = chunk_text(text, chunk_size=500, overlap=100)
# Returns 3 chunks:
#   - words 0-499 (500 words)
#   - words 400-899 (500 words, overlaps 100 with first)
#   - words 800-999 (200 words, overlaps 100 with second)
```

---

## Function: embed

**Purpose**: Generate vector embeddings using Cohere API.

**Signature**:
```python
def embed(texts: list[str]) -> list[list[float]]:
    """
    Generate embeddings for text chunks using Cohere.

    Args:
        texts: List of text strings to embed.

    Returns:
        List of embedding vectors (each 1024 floats). Empty list on error.
    """
```

**Input**:
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| texts | list[str] | Yes | Text strings to embed |

**Output**:
- List of embedding vectors
- Each vector is 1024 float values
- Order matches input texts

**Cohere API Call**:
```python
response = cohere_client.embed(
    texts=texts,
    model="embed-english-v3.0",
    input_type="search_document",
)
embeddings = response.embeddings
```

**Error Handling**:
- API error: Log error, return empty list
- Rate limit: Log error, return empty list

---

## Function: create_collection

**Purpose**: Create or verify Qdrant collection exists.

**Signature**:
```python
def create_collection() -> bool:
    """
    Create the Qdrant collection for storing embeddings.

    Returns:
        True if collection was created or already exists, False on error.
    """
```

**Output**:
- `True`: Collection ready for use
- `False`: Error occurred

**Behavior**:
1. Check if collection `rag_embedding` exists
2. If exists, return True
3. If not, create with:
   - Vector size: 1024
   - Distance: Cosine
4. Return True on success, False on error

---

## Function: save_chunk_to_qdrant

**Purpose**: Store embedded chunks with metadata in Qdrant.

**Signature**:
```python
def save_chunk_to_qdrant(
    chunks: list[str],
    embeddings: list[list[float]],
    metadata: dict,
) -> bool:
    """
    Save embedded chunks to Qdrant with metadata.

    Args:
        chunks: List of text chunks.
        embeddings: Corresponding embedding vectors.
        metadata: Dict with 'chapter', 'section', 'url' keys.

    Returns:
        True if successful, False on error.
    """
```

**Input**:
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| chunks | list[str] | Yes | Text chunks |
| embeddings | list[list[float]] | Yes | Matching vectors |
| metadata | dict | Yes | Source metadata |

**Metadata Schema**:
```python
{
    "chapter": str,   # e.g., "Chapter 1"
    "section": str,   # e.g., "Introduction to Physical AI"
    "url": str,       # Source URL
}
```

**Behavior**:
1. Validate chunks and embeddings have same length
2. For each (chunk, embedding) pair:
   - Generate point ID from MD5 hash of chunk text
   - Create PointStruct with vector and payload
3. Upsert all points to Qdrant collection

**Point Payload**:
```python
{
    "text": chunk,
    "chapter": metadata["chapter"],
    "section": metadata["section"],
    "source_url": metadata["url"],
    "chunk_index": i,
}
```

**Error Handling**:
- Mismatched lengths: Log error, return False
- Qdrant error: Log error, return False

---

## Function: main

**Purpose**: Orchestrate the full embedding pipeline.

**Signature**:
```python
def main() -> None:
    """
    Main execution: Extract content, generate embeddings, and store in Qdrant.
    """
```

**Execution Flow**:
1. Create Qdrant collection (abort if fails)
2. Get all book URLs
3. For each URL:
   a. Extract text
   b. Chunk text
   c. Generate embeddings
   d. Save to Qdrant
4. Print summary

**Output**: Console progress and summary statistics.

---

## External Dependencies

### Cohere API

**Endpoint**: `cohere.Client.embed()`
**Model**: `embed-english-v3.0`
**Authentication**: `COHERE_API_KEY` environment variable

### Qdrant Cloud

**Endpoint**: Configured via `QDRANT_URL` environment variable
**Authentication**: `QDRANT_API_KEY` environment variable
**Collection**: `rag_embedding`

---

## Environment Variables

| Variable | Required | Description |
|----------|----------|-------------|
| COHERE_API_KEY | Yes | Cohere API key for embeddings |
| QDRANT_URL | Yes | Qdrant Cloud cluster URL |
| QDRANT_API_KEY | Yes | Qdrant Cloud API key |
