"""
Retrieval Validation Assistant

A library for validating retrieval results in RAG (Retrieval-Augmented Generation) systems.
Prevents hallucinations by ensuring answers are grounded in retrieved data.

Public API:
    validate_retrieval: Main validation function that returns comprehensive results
    validate_retrieval_simple: Simplified validation returning only answer presence (bool)
    get_relevant_chunk_texts: Get texts of only relevant chunks
    validate_and_format_context: Validate and format context for answer generation

Data Models:
    ValidationRequest: Input validation request model
    ValidationResult: Complete validation outcome model
    TextChunk: Retrieved text chunk model
    RelevanceAssessment: Per-chunk relevance result
    EvidenceQuote: Extracted evidence with attribution
    RetrievalQuality: Quality rating enum (Good/Partial/Poor)

Exceptions:
    ValidationError: Raised for invalid input data
    CohereAPIError: Raised when Cohere API fails
    TimeoutError: Raised when processing exceeds timeout

Example Usage:
    >>> from retrieval_validation import validate_retrieval
    >>>
    >>> # After Qdrant retrieval
    >>> chunks = [
    ...     {"chunk_id": "ch1", "text": "ROS 2 is...", "metadata": {}},
    ...     {"chunk_id": "ch2", "text": "More info...", "metadata": {}}
    ... ]
    >>>
    >>> # Validate retrieval
    >>> result = validate_retrieval(
    ...     query="What is ROS 2?",
    ...     chunks=chunks
    ... )
    >>>
    >>> # Check result
    >>> if not result["answer_present"]:
    ...     return "No relevant information found in retrieved data."
    >>>
    >>> # Proceed with answer generation using validated chunks
    >>> relevant_texts = [
    ...     chunk["text"] for chunk in chunks
    ...     if chunk["chunk_id"] in [r["chunk_id"] for r in result["relevant_chunks"]]
    ... ]
"""

__version__ = "1.0.0"

# Import public API
from .validator import (
    validate_retrieval,
    validate_retrieval_simple,
    get_relevant_chunk_texts,
    validate_and_format_context
)

from .models import (
    ValidationRequest,
    ValidationResult,
    TextChunk,
    RelevanceAssessment,
    EvidenceQuote,
    RetrievalQuality
)

from .exceptions import (
    ValidationError,
    CohereAPIError,
    TimeoutError
)

__all__ = [
    # Main API functions
    "validate_retrieval",
    "validate_retrieval_simple",
    "get_relevant_chunk_texts",
    "validate_and_format_context",
    # Data models
    "ValidationRequest",
    "ValidationResult",
    "TextChunk",
    "RelevanceAssessment",
    "EvidenceQuote",
    "RetrievalQuality",
    # Exceptions
    "ValidationError",
    "CohereAPIError",
    "TimeoutError",
]
