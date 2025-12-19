# Quickstart Guide: Retrieval Validation Assistant

**Feature**: 003-retrieval-validation
**Date**: 2025-12-18
**Purpose**: Get started quickly with the retrieval validation component.

## Overview

The Retrieval Validation Assistant is a Python library that validates whether retrieved chunks from Qdrant are relevant to a user query and contain the answer. It prevents hallucinations in RAG systems by ensuring answers are grounded in retrieved data.

**5-Minute Integration**:
1. Install dependencies
2. Import the validator
3. Call `validate_retrieval()` after Qdrant search
4. Use the validation result to decide whether to proceed with answer generation

---

## Installation

### Prerequisites
- Python 3.11+
- Existing Backend project with Qdrant + Cohere setup

### Add Dependencies

Update `Backend/pyproject.toml`:

```toml
[project]
dependencies = [
    # Existing dependencies
    "cohere>=5.13.0",
    "qdrant-client>=1.12.0",
    # Add these:
    "pydantic>=2.0",
    "nltk>=3.9",
]
```

### Install NLTK Data

```bash
cd Backend
source .venv/bin/activate  # or .venv\Scripts\activate on Windows
python -c "import nltk; nltk.download('punkt')"
```

---

## Basic Usage

### Step 1: Import the Validator

```python
from retrieval_validation import validate_retrieval
```

### Step 2: Integrate After Qdrant Retrieval

```python
# Your existing retrieval code
retrieved_chunks = qdrant_client.search(
    collection_name="rag_embedding",
    query_vector=embedding,
    limit=10
)

# NEW: Convert to validation format
chunks = [
    {
        "chunk_id": hit.id,
        "text": hit.payload["text"],
        "metadata": hit.payload.get("metadata", {}),
        "score": hit.score
    }
    for hit in retrieved_chunks
]

# NEW: Validate retrieval
result = validate_retrieval(query="What is ROS 2?", chunks=chunks)
```

### Step 3: Use Validation Result

```python
# Check if answer is present
if not result["answer_present"]:
    return "No relevant information found in retrieved data."

# Check retrieval quality
if result["retrieval_quality"] == "Poor":
    return "Unable to find relevant information. Please rephrase your query."

# Proceed to answer generation with validated chunks
relevant_chunk_ids = [r["chunk_id"] for r in result["relevant_chunks"]]
relevant_texts = [
    chunk["text"] for chunk in chunks if chunk["chunk_id"] in relevant_chunk_ids
]

# Your answer generation code here
answer = generate_answer(query, relevant_texts)

# Optionally include evidence for transparency
evidence_quotes = [e["quote"] for e in result["evidence"]]
return {
    "answer": answer,
    "evidence": evidence_quotes,
    "quality": result["retrieval_quality"]
}
```

---

## Complete Integration Example

### Before: Without Validation (Prone to Hallucinations)

```python
def answer_query(query: str) -> str:
    # Embed query
    embedding = cohere_client.embed(texts=[query]).embeddings[0]

    # Retrieve chunks
    results = qdrant_client.search(
        collection_name="rag_embedding",
        query_vector=embedding,
        limit=10
    )

    # Generate answer directly (RISKY: may hallucinate if chunks irrelevant)
    context = "\n\n".join([hit.payload["text"] for hit in results])
    answer = generate_answer(query, context)

    return answer
```

### After: With Validation (Hallucination-Proof)

```python
from retrieval_validation import validate_retrieval

def answer_query(query: str) -> dict:
    # Embed query
    embedding = cohere_client.embed(texts=[query]).embeddings[0]

    # Retrieve chunks
    results = qdrant_client.search(
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
        for hit in results
    ]

    # Validate retrieval (NEW)
    validation = validate_retrieval(query=query, chunks=chunks)

    # Handle validation failures (NEW)
    if not validation["answer_present"]:
        return {
            "answer": "No relevant information found in retrieved data.",
            "quality": validation["retrieval_quality"],
            "evidence": []
        }

    if validation["retrieval_quality"] == "Poor":
        return {
            "answer": "Unable to find sufficient relevant information. Please provide more details in your query.",
            "quality": "Poor",
            "evidence": []
        }

    # Use only validated relevant chunks (NEW)
    relevant_chunk_ids = [r["chunk_id"] for r in validation["relevant_chunks"]]
    relevant_chunks = [c for c in chunks if c["chunk_id"] in relevant_chunk_ids]
    context = "\n\n".join([chunk["text"] for chunk in relevant_chunks])

    # Generate answer with validated context
    answer = generate_answer(query, context)

    # Return with evidence (NEW)
    return {
        "answer": answer,
        "quality": validation["retrieval_quality"],
        "evidence": [e["quote"] for e in validation["evidence"]],
        "processing_time_ms": validation["processing_time_ms"]
    }
```

---

## Configuration Options

### Adjusting Relevance Threshold

**Default**: 0.3 (balanced precision/recall)

```python
# More strict (fewer chunks marked relevant)
result = validate_retrieval(
    query=query,
    chunks=chunks,
    relevance_threshold=0.5  # Higher threshold
)

# More lenient (more chunks marked relevant)
result = validate_retrieval(
    query=query,
    chunks=chunks,
    relevance_threshold=0.2  # Lower threshold
)
```

**Recommended Thresholds**:
- **0.2**: High recall (include borderline relevant chunks), risk of some noise
- **0.3**: Balanced (default, good for most use cases)
- **0.5**: High precision (only very relevant chunks), may miss some valid results

### Adding Request IDs for Tracing

```python
import uuid

request_id = str(uuid.uuid4())
result = validate_retrieval(
    query=query,
    chunks=chunks,
    request_id=request_id
)

# Logs will include request_id for tracing
logger.info(f"Validation complete for request {request_id}: quality={result['retrieval_quality']}")
```

---

## Error Handling

### Handle Common Errors

```python
from retrieval_validation import validate_retrieval
from retrieval_validation.exceptions import ValidationError, CohereAPIError
import logging

logger = logging.getLogger(__name__)

def safe_validate(query: str, chunks: list[dict]) -> dict:
    try:
        return validate_retrieval(query, chunks)

    except ValidationError as e:
        logger.error(f"Validation input error: {e}")
        return {
            "error": "Invalid query or chunks",
            "answer_present": False,
            "retrieval_quality": "Poor",
            "evidence": []
        }

    except CohereAPIError as e:
        logger.error(f"Cohere API failed: {e}")
        # Fallback: use basic relevance from Qdrant scores
        return fallback_validation(query, chunks)

    except TimeoutError as e:
        logger.error(f"Validation timeout: {e}")
        return {
            "error": "Processing timeout",
            "answer_present": False,
            "retrieval_quality": "Poor",
            "evidence": []
        }

    except Exception as e:
        logger.exception(f"Unexpected validation error: {e}")
        # Fail open: proceed without validation (log for investigation)
        return {
            "error": "Validation unavailable",
            "answer_present": True,  # Assume present to allow generation
            "retrieval_quality": "Unknown",
            "evidence": []
        }

def fallback_validation(query: str, chunks: list[dict]) -> dict:
    """Simple fallback when Cohere unavailable (uses Qdrant scores)."""
    relevant = [c for c in chunks if c.get("score", 0) >= 0.7]
    return {
        "relevant_chunks": [{"chunk_id": c["chunk_id"], "is_relevant": True} for c in relevant],
        "answer_present": len(relevant) > 0,
        "retrieval_quality": "Good" if len(relevant) >= 2 else "Partial",
        "evidence": [],  # Cannot extract evidence without Cohere
        "processing_time_ms": 0.0
    }
```

---

## Testing Your Integration

### Unit Test Example

```python
import pytest
from retrieval_validation import validate_retrieval

def test_validate_retrieval_with_relevant_chunks():
    """Test validation with relevant chunks."""
    query = "What is ROS 2?"
    chunks = [
        {
            "chunk_id": "ch1",
            "text": "ROS 2 is a set of software libraries and tools for building robot applications.",
            "metadata": {"chapter": "Chapter 2"}
        },
        {
            "chunk_id": "ch2",
            "text": "Completely unrelated content about cooking.",
            "metadata": {"chapter": "Chapter 10"}
        }
    ]

    result = validate_retrieval(query, chunks)

    # Assertions
    assert result["answer_present"] is True
    assert len(result["relevant_chunks"]) >= 1
    assert result["relevant_chunks"][0]["chunk_id"] == "ch1"
    assert result["retrieval_quality"] in ["Good", "Partial"]
    assert len(result["evidence"]) > 0

def test_validate_retrieval_no_relevant_chunks():
    """Test validation with no relevant chunks."""
    query = "What is quantum computing?"
    chunks = [
        {
            "chunk_id": "ch1",
            "text": "ROS 2 is a robot operating system.",
            "metadata": {}
        }
    ]

    result = validate_retrieval(query, chunks)

    # Assertions
    assert result["answer_present"] is False
    assert len(result["relevant_chunks"]) == 0
    assert result["retrieval_quality"] == "Poor"
    assert len(result["evidence"]) == 0
```

### Integration Test Example

```python
import pytest
from your_app import answer_query  # Your integrated function

def test_end_to_end_with_validation():
    """Test full query flow with validation."""
    query = "What are the key features of ROS 2?"

    result = answer_query(query)

    # Should get answer with evidence
    assert "answer" in result
    assert result["answer"] != "No relevant information found in retrieved data."
    assert len(result["evidence"]) > 0
    assert result["quality"] in ["Good", "Partial"]

def test_negative_query_handled():
    """Test query with no answer in database."""
    query = "What is the meaning of life?"  # Not in book content

    result = answer_query(query)

    # Should gracefully handle no answer
    assert "No relevant information found" in result["answer"]
    assert result["quality"] == "Poor"
    assert len(result["evidence"]) == 0
```

---

## Performance Tips

### 1. Batch Validation (Multiple Queries)

```python
# Don't do this (multiple API calls):
for query in queries:
    result = validate_retrieval(query, chunks)  # Expensive!

# Do this instead (batch processing):
results = []
for query in queries:
    result = validate_retrieval(query, chunks)  # Each call batches internally
    results.append(result)

# Note: Cohere rerank batches chunks within each call, but cannot batch across queries yet
```

### 2. Cache Validation Results

```python
from functools import lru_cache
import hashlib

def cache_key(query: str, chunks: list[dict]) -> str:
    """Generate cache key from query + chunk IDs."""
    chunk_ids = sorted([c["chunk_id"] for c in chunks])
    return hashlib.md5(f"{query}:{chunk_ids}".encode()).hexdigest()

validation_cache = {}

def cached_validate(query: str, chunks: list[dict]) -> dict:
    key = cache_key(query, chunks)
    if key not in validation_cache:
        validation_cache[key] = validate_retrieval(query, chunks)
    return validation_cache[key]
```

### 3. Limit Chunk Count

```python
# Don't validate 100 chunks (slow + expensive):
all_chunks = qdrant_client.search(..., limit=100)

# Do validate top 10-20 (faster + cheaper):
top_chunks = qdrant_client.search(..., limit=10)  # Pre-filter in Qdrant
result = validate_retrieval(query, top_chunks)
```

---

## Troubleshooting

### Issue: "ValidationError: Query must be at least 3 characters"

**Cause**: Query is too short or empty.

**Fix**:
```python
if len(query.strip()) < 3:
    return "Please provide a more detailed query."
result = validate_retrieval(query, chunks)
```

### Issue: "CohereAPIError: Rate limit exceeded"

**Cause**: Too many Cohere API calls.

**Fix**: Implement rate limiting or upgrade Cohere plan.
```python
import time
from tenacity import retry, wait_exponential, stop_after_attempt

@retry(wait=wait_exponential(multiplier=1, min=2, max=10), stop=stop_after_attempt(3))
def validate_with_retry(query, chunks):
    return validate_retrieval(query, chunks)
```

### Issue: "Validation takes too long (>2s)"

**Cause**: Too many chunks or slow Cohere API.

**Fix**: Reduce chunk count or increase timeout.
```python
# Reduce chunk count
chunks = chunks[:10]  # Top 10 only

# Or implement custom timeout handling (future feature)
```

### Issue: "Evidence quotes don't match chunks"

**Cause**: Bug in evidence extraction (should not happen).

**Fix**: Report issue with example data. As workaround:
```python
# Verify evidence manually
for evidence in result["evidence"]:
    chunk = next(c for c in chunks if c["chunk_id"] == evidence["chunk_id"])
    assert evidence["quote"] in chunk["text"], "Evidence mismatch!"
```

---

## Next Steps

1. **Read the full documentation**:
   - [Data Model](./data-model.md) - Understand entities and relationships
   - [API Contract](./contracts/validation_api.md) - Detailed API reference
   - [Research Decisions](./research.md) - Technology choices and rationale

2. **Explore advanced usage**:
   - Custom relevance thresholds for domain-specific needs
   - Evidence extraction customization (future)
   - Quality rating tuning (future)

3. **Monitor in production**:
   - Log `processing_time_ms` to track performance
   - Track `retrieval_quality` distribution to measure retrieval system health
   - Monitor Cohere API usage/costs

4. **Provide feedback**:
   - Report bugs or feature requests
   - Share use cases and integration patterns
   - Contribute improvements

---

## Summary

**Minimal Integration (3 lines)**:
```python
from retrieval_validation import validate_retrieval

result = validate_retrieval(query, chunks)
if not result["answer_present"]:
    return "No relevant information found in retrieved data."
```

**You're now protected against hallucinations!** 🎉
