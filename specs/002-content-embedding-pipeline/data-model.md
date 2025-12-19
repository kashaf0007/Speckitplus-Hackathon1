# Data Model: Content Embedding Pipeline

**Feature**: 002-content-embedding-pipeline
**Date**: 2025-12-17

## Overview

This document defines the data entities used by the content embedding pipeline. The primary storage is Qdrant Cloud vector database, with data structures defined as Python dictionaries/dataclasses.

---

## Entity: ContentSource

Represents a source of book content (webpage URL).

**Purpose**: Track ingestion sources and their processing status.

**Attributes**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| url | string | Yes | Full URL of the content page |
| chapter | string | Yes | Chapter identifier (e.g., "Chapter 1") |
| section | string | Yes | Section title (e.g., "Introduction to Physical AI") |
| path | string | Yes | URL path component (e.g., "/docs/chapter-1-introduction") |

**Python Representation**:
```python
# In-memory representation (dict)
content_source = {
    "url": "https://speckitplus-hackathon1.vercel.app/docs/chapter-1-introduction",
    "chapter": "Chapter 1",
    "section": "Introduction to Physical AI",
    "path": "/docs/chapter-1-introduction",
}
```

**Notes**:
- Not persisted separately; derived from hardcoded book structure
- Future: Could be stored in database for dynamic source management

---

## Entity: ContentChunk

A segment of extracted text ready for embedding.

**Purpose**: Represent a single unit of text that will be embedded and stored.

**Attributes**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| text | string | Yes | The chunk text content |
| source_url | string | Yes | URL this chunk was extracted from |
| chapter | string | Yes | Chapter identifier |
| section | string | Yes | Section title |
| chunk_index | integer | Yes | Position within source (0-indexed) |
| word_count | integer | No | Number of words in chunk |

**Python Representation**:
```python
content_chunk = {
    "text": "Physical AI represents the convergence of...",
    "source_url": "https://speckitplus-hackathon1.vercel.app/docs/chapter-1-introduction",
    "chapter": "Chapter 1",
    "section": "Introduction to Physical AI",
    "chunk_index": 0,
    "word_count": 487,
}
```

**Notes**:
- Chunks are transient; created during processing and immediately embedded
- Target size: 500 words with 100-word overlap

---

## Entity: EmbeddingRecord (Qdrant Point)

The vector representation stored in Qdrant for retrieval.

**Purpose**: Enable semantic search over book content.

**Storage**: Qdrant Cloud collection `rag_embedding`

**Attributes**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | integer | Yes | Unique point ID (MD5 hash of text) |
| vector | float[1024] | Yes | Cohere embedding vector |
| payload.text | string | Yes | Original chunk text |
| payload.chapter | string | Yes | Chapter identifier |
| payload.section | string | Yes | Section title |
| payload.source_url | string | Yes | Source page URL |
| payload.chunk_index | integer | Yes | Position within source |

**Qdrant Point Structure**:
```python
from qdrant_client.models import PointStruct

point = PointStruct(
    id=4823947293847239,  # MD5-derived integer
    vector=[0.123, -0.456, ...],  # 1024 floats
    payload={
        "text": "Physical AI represents the convergence of...",
        "chapter": "Chapter 1",
        "section": "Introduction to Physical AI",
        "source_url": "https://speckitplus-hackathon1.vercel.app/docs/chapter-1-introduction",
        "chunk_index": 0,
    },
)
```

**ID Generation**:
```python
import hashlib

content_hash = hashlib.md5(chunk_text.encode()).hexdigest()
point_id = int(content_hash[:16], 16) % (2**63)  # Positive int64
```

---

## Qdrant Collection Configuration

**Collection Name**: `rag_embedding`

**Vector Configuration**:
```python
from qdrant_client.models import Distance, VectorParams

vectors_config = VectorParams(
    size=1024,           # Cohere embed-english-v3.0 dimension
    distance=Distance.COSINE,  # Cosine similarity for semantic search
)
```

**Payload Indexing** (optional optimization):
```python
# Enable filtering by chapter
payload_schema = {
    "chapter": PayloadSchemaType.KEYWORD,
    "section": PayloadSchemaType.KEYWORD,
}
```

---

## Entity Relationships

```
┌─────────────────┐
│  ContentSource  │
│    (webpage)    │
└────────┬────────┘
         │ 1:N (extracts to)
         ▼
┌─────────────────┐
│  ContentChunk   │
│   (text seg)    │
└────────┬────────┘
         │ 1:1 (embeds to)
         ▼
┌─────────────────┐
│ EmbeddingRecord │
│  (Qdrant Point) │
└─────────────────┘
```

**Cardinality**:
- One ContentSource → Many ContentChunks (based on content length)
- One ContentChunk → One EmbeddingRecord (1:1 mapping)

---

## Data Flow

1. **Input**: List of ContentSource URLs
2. **Extract**: Fetch URL → Parse HTML → Clean text
3. **Chunk**: Split text into ContentChunks (500 words, 100 overlap)
4. **Embed**: Generate vector for each chunk via Cohere API
5. **Store**: Upsert EmbeddingRecord to Qdrant

```
[URLs] → [Raw HTML] → [Clean Text] → [Chunks] → [Vectors] → [Qdrant]
```

---

## Validation Rules

| Entity | Rule | Error Handling |
|--------|------|----------------|
| ContentSource | URL must be valid HTTP(S) | Skip with error log |
| ContentSource | URL must return 200 status | Skip with error log |
| ContentChunk | Text must not be empty | Skip source |
| ContentChunk | Word count should be 400-600 | Warning only |
| EmbeddingRecord | Vector must be 1024 dimensions | Fatal error |
| EmbeddingRecord | ID must be unique | Upsert overwrites |
