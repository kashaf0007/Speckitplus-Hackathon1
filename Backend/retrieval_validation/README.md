# Retrieval Validation Assistant

A library for validating retrieval results in RAG (Retrieval-Augmented Generation) systems. Prevents hallucinations by ensuring answers are grounded in retrieved data.

## Features

- ✅ **Relevance Validation**: Uses Cohere rerank API for semantic relevance scoring
- ✅ **Answer Presence Detection**: Validates that retrieved chunks actually contain the answer
- ✅ **Evidence Extraction**: Extracts verbatim quotes with attribution for traceability
- ✅ **Quality Assessment**: Provides three-tier rating (Good/Partial/Poor) for observability
- ✅ **Error Handling**: Graceful fallbacks when APIs unavailable
- ✅ **Performance**: <2s processing for 20 chunks

## Installation

```bash
cd Backend
pip install -e .
python -c "import nltk; nltk.download('punkt')"
```

## Quick Start

```python
from retrieval_validation import validate_retrieval

# After Qdrant retrieval
chunks = [
    {"chunk_id": "ch1", "text": "ROS 2 is...", "metadata": {}},
    {"chunk_id": "ch2", "text": "More info...", "metadata": {}}
]

# Validate retrieval
result = validate_retrieval(
    query="What is ROS 2?",
    chunks=chunks
)

# Check result
if not result["answer_present"]:
    return "No relevant information found in retrieved data."

# Proceed with answer generation using validated chunks
relevant_texts = [
    chunk["text"] for chunk in chunks
    if chunk["chunk_id"] in [r["chunk_id"] for r in result["relevant_chunks"]]
]
```

## API Reference

### Main Functions

#### `validate_retrieval(query, chunks, relevance_threshold=0.3, request_id=None, cohere_client=None, timeout_ms=5000.0)`

Main validation function that returns comprehensive results.

**Returns**: Dict with keys:
- `relevant_chunks`: List of relevant chunk assessments
- `answer_present`: Boolean indicating if answer is in chunks
- `evidence`: List of evidence quotes with attribution
- `retrieval_quality`: Quality rating ("Good", "Partial", or "Poor")
- `quality_reasoning`: Explanation of the rating
- `processing_time_ms`: Processing time in milliseconds

#### `validate_retrieval_simple(query, chunks)`

Simplified validation returning only answer presence (bool).

#### `get_relevant_chunk_texts(query, chunks)`

Get texts of only relevant chunks.

#### `validate_and_format_context(query, chunks, include_evidence=True)`

Validate and format context for answer generation.

## Architecture

```
retrieval_validation/
├── __init__.py           # Public API exports
├── models.py             # Pydantic data models
├── exceptions.py         # Custom exceptions
├── logger.py             # Structured logging
├── relevance.py          # Cohere rerank integration
├── answer_detector.py    # Answer presence detection
├── evidence.py           # Evidence extraction (NLTK)
├── quality.py            # Quality assessment
└── validator.py          # Main orchestration
```

## Testing

```bash
cd Backend

# Run unit tests
pytest tests/unit/test_retrieval_validation/

# Run integration tests
pytest tests/integration/test_retrieval_validation_integration.py

# Run all tests
pytest tests/
```

## Error Handling

The module defines three custom exceptions:

- `ValidationError`: Invalid input data
- `CohereAPIError`: Cohere API failure
- `TimeoutError`: Processing timeout exceeded

All validation functions include graceful error handling with fallback mechanisms.

## Performance

- **Processing Time**: <2s for 20 chunks (target: SC-002)
- **API Calls**: Single batched Cohere rerank call
- **Memory**: <100MB peak usage
- **Timeout**: 5s absolute maximum

## Quality Ratings

- **Good**: ≥2 relevant chunks, answer present, avg relevance ≥0.6
- **Partial**: 1 relevant chunk OR answer partially present OR moderate scores
- **Poor**: 0 relevant chunks OR avg relevance <0.3

## Examples

See `tests/integration/test_retrieval_validation_integration.py` for comprehensive examples.

## License

Part of the Book RAG Chatbot project.

## Version

1.0.0 - Initial release
