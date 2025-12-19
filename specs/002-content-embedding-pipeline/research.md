# Research: Content Embedding Pipeline

**Feature**: 002-content-embedding-pipeline
**Date**: 2025-12-17
**Status**: Complete

## Research Summary

This document captures technology decisions and best practices for implementing the content embedding pipeline.

---

## Decision 1: Embedding Model Selection

**Decision**: Use Cohere `embed-english-v3.0` model

**Rationale**:
- Constitution mandates Cohere for embeddings (Principle VI)
- `embed-english-v3.0` provides 1024-dimensional vectors with excellent semantic quality
- Supports `search_document` and `search_query` input types for optimal RAG performance
- Free tier provides sufficient capacity for book-sized content

**Alternatives Considered**:
| Alternative | Reason Rejected |
|-------------|-----------------|
| OpenAI text-embedding-3 | Constitution requires Cohere |
| Cohere embed-multilingual-v3 | Book is in English; English-specific model is more efficient |
| Local embedding models | Adds infrastructure complexity; Cohere API is simpler |

---

## Decision 2: Vector Database Configuration

**Decision**: Qdrant Cloud with collection `rag_embedding`, cosine distance

**Rationale**:
- Constitution mandates Qdrant Cloud (Principle VI)
- Cosine distance is standard for text embeddings and works well with normalized vectors
- Free tier supports up to 1GB storage, sufficient for single book
- Cloud-hosted eliminates infrastructure management

**Configuration**:
```python
VectorParams(
    size=1024,  # Cohere embed-english-v3.0 dimension
    distance=Distance.COSINE,
)
```

**Alternatives Considered**:
| Alternative | Reason Rejected |
|-------------|-----------------|
| Pinecone | Constitution requires Qdrant |
| Self-hosted Qdrant | Adds operational overhead |
| Euclidean distance | Cosine is more appropriate for semantic similarity |

---

## Decision 3: Chunking Strategy

**Decision**: Word-based chunking with 500 words per chunk, 100-word overlap

**Rationale**:
- 500 words ≈ 600-700 tokens, safely under Cohere's 512-token optimal window
- 100-word overlap preserves context across chunk boundaries
- Simple word-based splitting is robust for web-extracted text
- Spec requirement FR-004 specifies 500-1000 tokens with 100-token overlap

**Implementation**:
```python
CHUNK_SIZE = 500  # words
CHUNK_OVERLAP = 100  # words
```

**Alternatives Considered**:
| Alternative | Reason Rejected |
|-------------|-----------------|
| Sentence-based chunking | More complex; web content may have irregular sentence boundaries |
| Token-based chunking | Requires tokenizer; word-based is simpler and sufficient |
| Larger chunks (1000 words) | May exceed optimal embedding window; smaller is safer |

---

## Decision 4: Web Content Extraction

**Decision**: Use httpx + BeautifulSoup4 with lxml parser

**Rationale**:
- httpx provides modern async-capable HTTP client with good defaults
- BeautifulSoup4 handles real-world HTML robustly
- lxml parser is fast and handles malformed HTML well
- Target site (Vercel-hosted Docusaurus) has clean, predictable structure

**Content Selection Strategy**:
1. Remove non-content elements: `<script>`, `<style>`, `<nav>`, `<header>`, `<footer>`, `<aside>`
2. Find main content: `<main>`, `<article>`, `.markdown`, `.content`, or `<body>` fallback
3. Extract text with newline separators, strip whitespace

**Alternatives Considered**:
| Alternative | Reason Rejected |
|-------------|-----------------|
| Selenium/Playwright | Overkill for static site; adds complexity |
| requests library | httpx is more modern with better async support |
| html.parser | lxml is faster and handles edge cases better |

---

## Decision 5: Metadata Schema

**Decision**: Store comprehensive metadata with each embedding

**Payload Structure**:
```python
{
    "text": str,           # Original chunk text (for retrieval display)
    "chapter": str,        # e.g., "Chapter 1"
    "section": str,        # e.g., "Introduction to Physical AI"
    "source_url": str,     # Full URL of source page
    "chunk_index": int,    # Position within source (for ordering)
}
```

**Rationale**:
- Spec requirement FR-005 mandates chapter, section, page/URL metadata
- Original text stored for retrieval without re-fetching
- chunk_index enables reconstructing document order if needed
- Supports filtering queries by chapter

**Alternatives Considered**:
| Alternative | Reason Rejected |
|-------------|-----------------|
| Minimal metadata (URL only) | Loses chapter/section info needed for filtering |
| Store text separately | Adds complexity; Qdrant payload is sufficient |
| Include page numbers | Web content doesn't have page numbers; URL is sufficient |

---

## Decision 6: Duplicate Detection

**Decision**: Content-based MD5 hash for point ID generation

**Rationale**:
- MD5 of chunk text provides deterministic ID for deduplication
- Qdrant upsert with same ID automatically handles re-indexing
- No separate deduplication logic needed
- Spec requirement FR-010 satisfied

**Implementation**:
```python
content_hash = hashlib.md5(chunk.encode()).hexdigest()
point_id = int(content_hash[:16], 16) % (2**63)
```

**Alternatives Considered**:
| Alternative | Reason Rejected |
|-------------|-----------------|
| UUID generation | Doesn't enable deduplication |
| SHA-256 | Overkill for dedup; MD5 is sufficient and faster |
| SimHash for near-duplicates | Adds complexity; exact dedup is sufficient for MVP |

---

## Decision 7: Error Handling Strategy

**Decision**: Fail-fast with clear error messages, skip problematic content

**Rationale**:
- Pipeline should not halt completely on single page failure
- Clear error messages help debugging
- Spec requirement SC-007 requires actionable error messages
- Partial success is acceptable for initial ingestion

**Behavior**:
- Invalid URL: Log error, skip URL, continue with others
- Embedding API failure: Log error, skip chunk, continue
- Qdrant failure: Log error, report partial failure
- No content extracted: Log warning, skip, continue

**Alternatives Considered**:
| Alternative | Reason Rejected |
|-------------|-----------------|
| Strict fail-all | One bad page shouldn't block entire book |
| Silent retry | User needs visibility into failures |
| Queue for retry | Adds complexity; manual re-run is simpler for MVP |

---

## Book Content URLs

**Source**: https://speckitplus-hackathon1.vercel.app/

| Path | Chapter | Section |
|------|---------|---------|
| /docs/intro | Introduction | Start Reading |
| /docs/chapter-1-introduction | Chapter 1 | Introduction to Physical AI |
| /docs/chapter-2-ros2 | Chapter 2 | ROS 2 Fundamentals |
| /docs/chapter-3-simulation | Chapter 3 | Simulation (Gazebo & Unity) |
| /docs/chapter-4-isaac | Chapter 4 | NVIDIA Isaac |
| /docs/chapter-5-vla | Chapter 5 | Vision-Language-Action |
| /docs/chapter-6-capstone | Chapter 6 | Capstone Project |

---

## Open Questions (Resolved)

| Question | Resolution |
|----------|------------|
| Which Cohere model? | embed-english-v3.0 (1024 dims) |
| Chunk size? | 500 words with 100-word overlap |
| How to handle duplicates? | MD5 hash-based point IDs |
| What metadata to store? | text, chapter, section, source_url, chunk_index |
