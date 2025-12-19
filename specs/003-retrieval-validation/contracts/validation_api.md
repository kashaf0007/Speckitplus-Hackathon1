# API Contract: Retrieval Validation

**Feature**: 003-retrieval-validation
**Date**: 2025-12-18
**Purpose**: Define the public API contract for the retrieval validation module.

## Overview

The Retrieval Validation module exposes a single primary function for validating retrieval results. This is a **library API** (Python function call), not an HTTP/REST API.

## Public API

### Function: `validate_retrieval`

**Purpose**: Validate that retrieved chunks are relevant to a query and contain the answer.

**Signature**:
```python
def validate_retrieval(
    query: str,
    chunks: list[dict],
    relevance_threshold: float = 0.3,
    request_id: Optional[str] = None
) -> dict:
    """
    Validate retrieval results for a RAG query.

    Args:
        query: User's question or information request (3-500 chars)
        chunks: List of retrieved chunks from Qdrant, each with:
            - chunk_id (str): Unique identifier
            - text (str): Chunk content
            - metadata (dict, optional): Additional context
            - score (float, optional): Original similarity score
        relevance_threshold: Minimum relevance score (0.0-1.0, default: 0.3)
        request_id: Optional ID for tracing/logging

    Returns:
        dict: Validation result with structure:
            {
                "relevant_chunks": [
                    {
                        "chunk_id": str,
                        "relevance_score": float,
                        "is_relevant": bool,
                        "summary": str
                    }
                ],
                "answer_present": bool,
                "evidence": [
                    {
                        "chunk_id": str,
                        "quote": str,
                        "sentence_index": int,
                        "context_before": str (optional),
                        "context_after": str (optional)
                    }
                ],
                "retrieval_quality": str,  # "Good", "Partial", or "Poor"
                "quality_reasoning": str,
                "processing_time_ms": float
            }

    Raises:
        ValidationError: If input data is invalid (e.g., empty query, malformed chunks)
        CohereAPIError: If Cohere rerank API fails
        TimeoutError: If processing exceeds timeout (default: 5s)

    Example:
        >>> result = validate_retrieval(
        ...     query="What is ROS 2?",
        ...     chunks=[
        ...         {"chunk_id": "ch1", "text": "ROS 2 is...", "metadata": {}},
        ...         {"chunk_id": "ch2", "text": "Unrelated content", "metadata": {}}
        ...     ]
        ... )
        >>> result["answer_present"]
        True
        >>> result["retrieval_quality"]
        "Good"
    """
```

---

## Input Contract

### Query Validation

| Rule | Validation | Error if Violated |
|------|------------|-------------------|
| Query must not be empty | `len(query.strip()) >= 3` | `ValidationError("Query must be at least 3 characters")` |
| Query must not exceed limit | `len(query) <= 500` | `ValidationError("Query must not exceed 500 characters")` |
| Query must be string | `isinstance(query, str)` | `TypeError("Query must be a string")` |

### Chunks Validation

| Rule | Validation | Error if Violated |
|------|------------|-------------------|
| Chunks must be list | `isinstance(chunks, list)` | `TypeError("Chunks must be a list")` |
| Max 50 chunks | `len(chunks) <= 50` | `ValidationError("Cannot process more than 50 chunks")` |
| Each chunk must have chunk_id | `"chunk_id" in chunk and chunk["chunk_id"]` | `ValidationError(f"Chunk at index {i} missing chunk_id")` |
| Each chunk must have text | `"text" in chunk and chunk["text"]` | `ValidationError(f"Chunk {chunk_id} missing text")` |
| Chunk IDs must be unique | `len(chunk_ids) == len(set(chunk_ids))` | `ValidationError("Duplicate chunk IDs found")` |

### Threshold Validation

| Rule | Validation | Error if Violated |
|------|------------|-------------------|
| Threshold in range [0, 1] | `0.0 <= relevance_threshold <= 1.0` | `ValidationError("Relevance threshold must be between 0.0 and 1.0")` |

---

## Output Contract

### Success Response

**Structure**:
```json
{
    "relevant_chunks": [
        {
            "chunk_id": "rag_embedding-1234",
            "relevance_score": 0.85,
            "is_relevant": true,
            "summary": "ROS 2 is a set of software libraries..."
        }
    ],
    "answer_present": true,
    "evidence": [
        {
            "chunk_id": "rag_embedding-1234",
            "quote": "ROS 2 provides real-time capabilities, improved security, and cross-platform support.",
            "sentence_index": 2,
            "context_before": "ROS 2 is the next generation of ROS.",
            "context_after": "It also supports multiple programming languages."
        }
    ],
    "retrieval_quality": "Good",
    "quality_reasoning": "2 relevant chunks found (avg score: 0.82), answer fully present",
    "processing_time_ms": 342.5
}
```

**Field Guarantees**:
- `relevant_chunks`: Always present, may be empty list
- `answer_present`: Always boolean (never null)
- `evidence`: Empty list if `answer_present` is false
- `retrieval_quality`: Always one of: "Good", "Partial", "Poor"
- `quality_reasoning`: Always non-empty string explaining rating
- `processing_time_ms`: Always positive float

### Error Response

**Exception Types**:

1. **ValidationError** (input data issues):
```python
ValidationError: Query must be at least 3 characters
```

2. **CohereAPIError** (external service failure):
```python
CohereAPIError: Cohere rerank API returned 429 (rate limit exceeded)
```

3. **TimeoutError** (processing too slow):
```python
TimeoutError: Validation processing exceeded 5000ms timeout
```

**Error Handling Contract**:
- All errors include descriptive messages
- Errors are raised immediately (fail-fast)
- No partial results returned on error
- Errors are logged before being raised

---

## Behavior Guarantees

### Determinism

**Guarantee**: Same input always produces same output (within same Cohere rerank model version).

**Rationale**:
- No random sampling or probabilistic algorithms
- Cohere rerank is deterministic given same model version
- Sentence tokenization is rule-based (not ML-based)

**Caveat**: If Cohere updates rerank model, scores may change slightly, but relative ordering remains stable.

### No Side Effects

**Guarantee**: Function does not modify input data or external state (except logging).

**Side Effects Limited To**:
- Logging validation decisions (read-only observability)
- API calls to Cohere (external service, read-only from perspective of validation logic)

**No Side Effects**:
- No database writes
- No file system modifications
- No global state changes

### Performance Guarantees

| Metric | Guarantee | Measured By |
|--------|-----------|-------------|
| Processing time | <2000ms for 20 chunks | `processing_time_ms` field |
| Memory usage | <100MB peak | Internal profiling (not exposed in API) |
| API calls | 1 Cohere rerank call per request | Cohere API logs |

**Failure Mode**: If processing exceeds 5000ms (absolute timeout), raises `TimeoutError`.

---

## Integration Examples

### Example 1: Basic Usage

```python
from retrieval_validation import validate_retrieval

# After Qdrant retrieval
retrieved_chunks = qdrant_client.search(
    collection_name="rag_embedding",
    query_vector=embedding,
    limit=10
)

# Convert to validation format
chunks = [
    {
        "chunk_id": hit.id,
        "text": hit.payload["text"],
        "metadata": hit.payload.get("metadata", {}),
        "score": hit.score
    }
    for hit in retrieved_chunks
]

# Validate
result = validate_retrieval(query="What is ROS 2?", chunks=chunks)

# Check result
if not result["answer_present"]:
    return "No relevant information found in retrieved data."

# Proceed to answer generation with validated chunks
relevant_chunk_texts = [
    chunk["text"]
    for chunk in chunks
    if chunk["chunk_id"] in [r["chunk_id"] for r in result["relevant_chunks"]]
]
```

### Example 2: Error Handling

```python
try:
    result = validate_retrieval(query, chunks)
except ValidationError as e:
    logger.error(f"Invalid input: {e}")
    return {"error": "Invalid query or chunks", "details": str(e)}
except CohereAPIError as e:
    logger.error(f"Cohere API failed: {e}")
    return {"error": "Validation service unavailable", "details": "Retry later"}
except TimeoutError as e:
    logger.error(f"Validation timeout: {e}")
    return {"error": "Validation took too long", "details": "Try with fewer chunks"}
```

### Example 3: Custom Threshold

```python
# More strict relevance requirement
result = validate_retrieval(
    query="What is ROS 2?",
    chunks=chunks,
    relevance_threshold=0.5  # Higher threshold = fewer relevant chunks
)

# Use for high-precision use cases (medical, legal, etc.)
if result["retrieval_quality"] == "Good":
    # High confidence answer
    proceed_to_generation()
else:
    # Low confidence, request more specific query
    return "Please provide more specific details"
```

---

## Versioning & Compatibility

**Current Version**: 1.0.0 (initial release)

**Semantic Versioning**:
- **Major**: Breaking changes to input/output contract
- **Minor**: Backward-compatible additions (new optional fields)
- **Patch**: Bug fixes, performance improvements

**Compatibility Promise**:
- Input contract stable: Existing code won't break
- Output contract may add new optional fields (minor version bump)
- Existing fields won't be removed or renamed (major version bump required)

**Deprecation Policy**:
- 6 months notice before removing any feature
- Deprecation warnings logged in code
- Migration guide provided

---

## Testing Contract

**Unit Test Coverage**:
- All validation rules tested with valid/invalid inputs
- Edge cases (empty chunks, all irrelevant, etc.)
- Error conditions (malformed data, API failures)

**Contract Test Coverage**:
- Input validation tests (ensure errors raised correctly)
- Output structure tests (ensure all required fields present)
- Behavior tests (determinism, no side effects)

**Integration Test Coverage**:
- End-to-end with real Cohere API
- Performance tests (verify <2s guarantee)
- Error handling tests (API failures, timeouts)

**Test Data**:
- Sample chunks provided in `tests/fixtures/sample_chunks.py`
- Cover diverse scenarios (relevant/irrelevant, short/long, etc.)
