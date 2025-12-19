# Data Model: Retrieval Validation Assistant

**Feature**: 003-retrieval-validation
**Date**: 2025-12-18
**Purpose**: Define all data entities, their relationships, validation rules, and state transitions.

## Overview

The Retrieval Validation Assistant operates on three primary data entities:
1. **Validation Request** (Input)
2. **Text Chunk** (Input, from Qdrant)
3. **Validation Result** (Output)

Additional internal entities:
4. **Relevance Assessment** (per-chunk scoring)
5. **Evidence Quote** (extracted text with attribution)
6. **Quality Rating** (aggregate assessment)

## Core Entities

### 1. Validation Request (Input)

**Purpose**: Encapsulates all information needed to validate a retrieval result.

**Fields**:

| Field | Type | Required | Description | Validation Rules |
|-------|------|----------|-------------|------------------|
| `query` | `str` | Yes | User's question or information request | Min length: 3 chars, Max length: 500 chars |
| `chunks` | `list[TextChunk]` | Yes | Retrieved text chunks from Qdrant | Min items: 0, Max items: 50 |
| `relevance_threshold` | `float` | No | Minimum score for relevance (default: 0.3) | Range: 0.0-1.0 |
| `request_id` | `str` | No | Optional ID for tracing/logging | UUID format recommended |

**Validation Rules**:
- Query must not be empty or whitespace-only
- Chunks list may be empty (valid edge case: no retrieval results)
- Relevance threshold must be between 0.0 and 1.0 (inclusive)

**Example**:
```python
{
    "query": "What are the key features of ROS 2?",
    "chunks": [
        {"chunk_id": "ch001", "text": "ROS 2 provides..."},
        {"chunk_id": "ch002", "text": "Key features include..."}
    ],
    "relevance_threshold": 0.3,
    "request_id": "req-123-abc"
}
```

---

### 2. Text Chunk (Input, from Qdrant)

**Purpose**: Represents a single retrieved text segment from the vector database.

**Fields**:

| Field | Type | Required | Description | Validation Rules |
|-------|------|----------|-------------|------------------|
| `chunk_id` | `str` | Yes | Unique identifier from Qdrant | Non-empty string |
| `text` | `str` | Yes | The actual content of the chunk | Min length: 1 char, Max length: 5000 chars |
| `metadata` | `dict` | No | Optional metadata (chapter, section, source URL) | Key-value pairs |
| `score` | `float` | No | Original Qdrant similarity score (if available) | Range: 0.0-1.0 |

**Validation Rules**:
- chunk_id must be unique within a validation request
- text must not be empty
- metadata keys must be strings
- score is informational only (not used for validation logic)

**Example**:
```python
{
    "chunk_id": "rag_embedding-1234",
    "text": "ROS 2 is a set of software libraries and tools...",
    "metadata": {
        "chapter": "Chapter 2",
        "section": "ROS 2 Fundamentals",
        "source_url": "https://speckitplus-hackathon1.vercel.app/docs/chapter-2-ros2"
    },
    "score": 0.87
}
```

---

### 3. Validation Result (Output)

**Purpose**: Complete validation outcome with all assessments and evidence.

**Fields**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `relevant_chunks` | `list[RelevanceAssessment]` | Yes | Chunks deemed relevant to the query |
| `answer_present` | `bool` | Yes | Whether answer is explicitly in retrieved data |
| `evidence` | `list[EvidenceQuote]` | No | Exact quotes supporting the answer (empty if answer_present=False) |
| `retrieval_quality` | `str` | Yes | Overall quality rating: "Good", "Partial", or "Poor" |
| `quality_reasoning` | `str` | Yes | Human-readable explanation of the quality rating |
| `processing_time_ms` | `float` | Yes | Time taken for validation (for observability) |

**Validation Rules**:
- If `answer_present` is False, `evidence` must be empty
- `retrieval_quality` must be one of: "Good", "Partial", "Poor"
- `processing_time_ms` must be positive

**State Transitions**:
1. **Initialization**: Empty validation result created
2. **Relevance Assessment**: `relevant_chunks` populated
3. **Answer Detection**: `answer_present` set based on relevant chunks
4. **Evidence Extraction**: `evidence` populated if answer present
5. **Quality Rating**: `retrieval_quality` and `quality_reasoning` calculated
6. **Finalization**: `processing_time_ms` recorded, result returned

**Example**:
```python
{
    "relevant_chunks": [
        {
            "chunk_id": "ch001",
            "relevance_score": 0.85,
            "summary": "Describes ROS 2 core features..."
        }
    ],
    "answer_present": True,
    "evidence": [
        {
            "chunk_id": "ch001",
            "quote": "ROS 2 provides real-time capabilities, improved security, and cross-platform support.",
            "sentence_index": 2
        }
    ],
    "retrieval_quality": "Good",
    "quality_reasoning": "2 highly relevant chunks found (avg score: 0.82), answer fully present",
    "processing_time_ms": 342.5
}
```

---

### 4. Relevance Assessment (Internal/Output)

**Purpose**: Per-chunk relevance scoring result.

**Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `chunk_id` | `str` | Reference to the assessed chunk |
| `relevance_score` | `float` | Normalized relevance score from Cohere rerank (0.0-1.0) |
| `is_relevant` | `bool` | True if score >= threshold |
| `summary` | `str` | Brief description of chunk content (first 100 chars) |

**Validation Rules**:
- `relevance_score` must be between 0.0 and 1.0
- `is_relevant` must match: `relevance_score >= relevance_threshold`
- `summary` max length: 150 chars

**Example**:
```python
{
    "chunk_id": "rag_embedding-1234",
    "relevance_score": 0.85,
    "is_relevant": True,
    "summary": "ROS 2 is a set of software libraries and tools that help you build robot applications..."
}
```

---

### 5. Evidence Quote (Internal/Output)

**Purpose**: Extracted text supporting the answer, with attribution.

**Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `chunk_id` | `str` | Source chunk ID for traceability |
| `quote` | `str` | Exact text extracted from chunk (verbatim, no paraphrasing) |
| `sentence_index` | `int` | Position of sentence within chunk (0-indexed) |
| `context_before` | `str` (optional) | Previous sentence for context |
| `context_after` | `str` (optional) | Next sentence for context |

**Validation Rules**:
- `quote` must exist verbatim in the referenced chunk
- `quote` must not be empty
- `sentence_index` must be non-negative
- Context fields are optional but recommended

**Example**:
```python
{
    "chunk_id": "rag_embedding-1234",
    "quote": "ROS 2 provides real-time capabilities, improved security, and cross-platform support.",
    "sentence_index": 2,
    "context_before": "ROS 2 is the next generation of ROS.",
    "context_after": "It also supports multiple programming languages."
}
```

---

### 6. Quality Rating (Internal/Output)

**Purpose**: Aggregate assessment of retrieval quality.

**Rating Enum**:
```python
class RetrievalQuality(str, Enum):
    GOOD = "Good"
    PARTIAL = "Partial"
    POOR = "Poor"
```

**Rating Logic**:

| Rating | Criteria |
|--------|----------|
| **Good** | ≥2 relevant chunks AND answer_present=True AND avg_relevance_score ≥ 0.6 |
| **Partial** | 1 relevant chunk OR answer_present=False (but >0 relevant chunks) OR avg_relevance_score 0.3-0.6 |
| **Poor** | 0 relevant chunks OR avg_relevance_score < 0.3 |

**Reasoning Template**:
- Good: "X relevant chunks found (avg score: Y), answer fully present"
- Partial: "Only X relevant chunk(s) found, answer may be incomplete"
- Poor: "No relevant chunks found, answer not present"

---

## Entity Relationships

```
ValidationRequest
    └── contains [0..50] TextChunks
        └── assessed by RelevanceAssessment
            └── if relevant, may yield EvidenceQuote

ValidationResult
    ├── aggregates [0..N] RelevanceAssessments → relevant_chunks
    ├── includes [0..N] EvidenceQuotes → evidence
    └── derives QualityRating → retrieval_quality
```

**Relationship Rules**:
1. Each TextChunk produces exactly one RelevanceAssessment
2. Only relevant TextChunks (is_relevant=True) are included in ValidationResult.relevant_chunks
3. EvidenceQuotes only created from relevant chunks where answer is present
4. QualityRating calculated after all RelevanceAssessments complete

---

## Pydantic Model Definitions (Implementation Preview)

```python
from pydantic import BaseModel, Field, field_validator
from typing import Optional
from enum import Enum

class TextChunk(BaseModel):
    chunk_id: str = Field(..., min_length=1)
    text: str = Field(..., min_length=1, max_length=5000)
    metadata: Optional[dict] = None
    score: Optional[float] = Field(None, ge=0.0, le=1.0)

class ValidationRequest(BaseModel):
    query: str = Field(..., min_length=3, max_length=500)
    chunks: list[TextChunk] = Field(default_factory=list, max_length=50)
    relevance_threshold: float = Field(default=0.3, ge=0.0, le=1.0)
    request_id: Optional[str] = None

    @field_validator('query')
    @classmethod
    def query_not_empty(cls, v: str) -> str:
        if not v.strip():
            raise ValueError('Query must not be empty or whitespace-only')
        return v

class RelevanceAssessment(BaseModel):
    chunk_id: str
    relevance_score: float = Field(..., ge=0.0, le=1.0)
    is_relevant: bool
    summary: str = Field(..., max_length=150)

class EvidenceQuote(BaseModel):
    chunk_id: str
    quote: str = Field(..., min_length=1)
    sentence_index: int = Field(..., ge=0)
    context_before: Optional[str] = None
    context_after: Optional[str] = None

class RetrievalQuality(str, Enum):
    GOOD = "Good"
    PARTIAL = "Partial"
    POOR = "Poor"

class ValidationResult(BaseModel):
    relevant_chunks: list[RelevanceAssessment]
    answer_present: bool
    evidence: list[EvidenceQuote] = Field(default_factory=list)
    retrieval_quality: RetrievalQuality
    quality_reasoning: str
    processing_time_ms: float = Field(..., gt=0)

    @field_validator('evidence')
    @classmethod
    def evidence_requires_answer(cls, v: list[EvidenceQuote], info) -> list[EvidenceQuote]:
        if not info.data.get('answer_present') and v:
            raise ValueError('Evidence must be empty when answer_present is False')
        return v
```

---

## Validation Edge Cases

| Edge Case | Handling |
|-----------|----------|
| Empty chunks list | Valid input → quality="Poor", answer_present=False |
| All chunks irrelevant | Valid result → quality="Poor", answer_present=False, return standard message |
| Query too vague | Proceed normally (Cohere rerank handles ambiguity), may result in quality="Partial" |
| Chunk text is corrupted/malformed | Skip chunk, log warning, continue with remaining chunks |
| Duplicate chunk IDs | Raise validation error (violates uniqueness constraint) |
| Query matches chunk exactly | Valid case → likely quality="Good" with high relevance scores |

---

## Performance Considerations

**Model Serialization**:
- All models use Pydantic for efficient JSON serialization
- Typical ValidationResult size: 1-5KB (depends on # of relevant chunks and evidence quotes)
- In-memory processing, no database I/O

**Validation Overhead**:
- Pydantic validation adds ~1-2ms per request (negligible)
- Benefit: Catches invalid data before processing

**Data Flow Efficiency**:
```
Input (JSON) → Pydantic (ValidationRequest) → Processing → Pydantic (ValidationResult) → Output (JSON)
```

Total serialization overhead: ~5ms (well within 2000ms budget from SC-002)
