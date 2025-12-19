"""
Main validator orchestration

Integrates all validation components (relevance, answer detection, evidence, quality)
into the public validate_retrieval() API.
"""

import time
from typing import Optional
import cohere

from .models import (
    ValidationRequest,
    ValidationResult,
    TextChunk,
    RetrievalQuality
)
from .relevance import assess_relevance, fallback_relevance_assessment, filter_relevant_chunks
from .answer_detector import detect_answer_presence_with_full_chunks
from .evidence import extract_evidence
from .quality import assess_retrieval_quality
from .exceptions import ValidationError, CohereAPIError, TimeoutError
from .logger import (
    logger,
    log_validation_start,
    log_validation_complete,
    log_error
)


def validate_retrieval(
    query: str,
    chunks: list[dict],
    relevance_threshold: float = 0.3,
    request_id: Optional[str] = None,
    cohere_client: Optional[cohere.Client] = None,
    timeout_ms: float = 5000.0
) -> dict:
    """
    Validate retrieval results for a RAG query.

    This is the main public API for the retrieval validation module.
    It orchestrates all validation steps and returns a comprehensive result.

    Args:
        query: User's question or information request (3-500 chars)
        chunks: List of retrieved chunks from Qdrant, each with:
            - chunk_id (str): Unique identifier
            - text (str): Chunk content
            - metadata (dict, optional): Additional context
            - score (float, optional): Original similarity score
        relevance_threshold: Minimum relevance score (0.0-1.0, default: 0.3)
        request_id: Optional ID for tracing/logging
        cohere_client: Optional Cohere client (creates new if None)
        timeout_ms: Maximum processing time in milliseconds (default: 5000)

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
        ValidationError: If input data is invalid
        CohereAPIError: If Cohere rerank API fails
        TimeoutError: If processing exceeds timeout

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
    # Start timing
    start_time = time.time()

    # Validate inputs using Pydantic
    try:
        # Convert chunk dicts to TextChunk models
        chunk_models = [TextChunk(**chunk) for chunk in chunks]

        # Create validation request
        request = ValidationRequest(
            query=query,
            chunks=chunk_models,
            relevance_threshold=relevance_threshold,
            request_id=request_id
        )
    except Exception as e:
        error_msg = f"Invalid input data: {str(e)}"
        log_error("ValidationError", error_msg, request_id)
        raise ValidationError(error_msg)

    # Log validation start
    log_validation_start(request_id, query, len(chunks))

    try:
        # Check timeout
        def check_timeout():
            elapsed = (time.time() - start_time) * 1000
            if elapsed > timeout_ms:
                raise TimeoutError(
                    f"Validation processing exceeded {timeout_ms}ms timeout"
                )

        # Step 1: Assess relevance using Cohere rerank
        try:
            all_assessments = assess_relevance(
                query=request.query,
                chunks=request.chunks,
                relevance_threshold=request.relevance_threshold,
                cohere_client=cohere_client
            )
        except CohereAPIError as e:
            logger.warning(f"Cohere API failed, using fallback: {e}")
            # Fallback to Qdrant scores
            all_assessments = fallback_relevance_assessment(
                chunks=request.chunks,
                relevance_threshold=max(0.7, request.relevance_threshold + 0.1)
            )

        check_timeout()

        # Get relevant chunks only
        relevant_assessments = filter_relevant_chunks(all_assessments)

        # Step 2: Detect answer presence
        # Build chunk texts dictionary for full-text analysis
        chunk_texts = {chunk.chunk_id: chunk.text for chunk in request.chunks}

        answer_present, answer_reasoning = detect_answer_presence_with_full_chunks(
            query=request.query,
            relevant_assessments=relevant_assessments,
            chunk_texts=chunk_texts
        )

        check_timeout()

        # Step 3: Extract evidence (only if answer is present)
        if answer_present and relevant_assessments:
            evidence_list = extract_evidence(
                query=request.query,
                relevant_assessments=relevant_assessments,
                chunk_texts=chunk_texts,
                max_quotes_per_chunk=3,
                include_context=True
            )
        else:
            evidence_list = []

        check_timeout()

        # Step 4: Assess retrieval quality
        quality, quality_reasoning = assess_retrieval_quality(
            all_assessments=all_assessments,
            answer_present=answer_present
        )

        # Calculate processing time
        processing_time = (time.time() - start_time) * 1000  # Convert to ms

        # Build validation result
        result = ValidationResult(
            relevant_chunks=relevant_assessments,
            answer_present=answer_present,
            evidence=evidence_list,
            retrieval_quality=quality,
            quality_reasoning=quality_reasoning,
            processing_time_ms=processing_time
        )

        # Log completion
        log_validation_complete(
            request_id,
            processing_time,
            quality.value,
            answer_present
        )

        # Convert to dict for return
        return result.model_dump()

    except TimeoutError:
        # Re-raise timeout errors
        raise

    except CohereAPIError:
        # Re-raise Cohere errors
        raise

    except Exception as e:
        # Catch unexpected errors
        error_msg = f"Unexpected error during validation: {str(e)}"
        log_error("UnexpectedError", error_msg, request_id)
        raise ValidationError(error_msg)


def validate_retrieval_simple(
    query: str,
    chunks: list[dict]
) -> bool:
    """
    Simplified validation that returns only answer presence.

    Convenience function for cases where you only need to know
    if the answer is present, without detailed validation results.

    Args:
        query: User's question
        chunks: List of retrieved chunks

    Returns:
        bool: True if answer is present, False otherwise

    Example:
        >>> if validate_retrieval_simple("What is ROS 2?", chunks):
        ...     print("Answer found, proceed with generation")
        ... else:
        ...     print("No relevant information found")
    """
    try:
        result = validate_retrieval(query, chunks)
        return result["answer_present"]
    except Exception:
        # On error, fail open (assume answer might be present)
        logger.exception("Validation failed, defaulting to answer_present=True")
        return True


def get_relevant_chunk_texts(
    query: str,
    chunks: list[dict]
) -> list[str]:
    """
    Get only the texts of relevant chunks.

    Convenience function to filter chunks before answer generation.

    Args:
        query: User's question
        chunks: List of retrieved chunks

    Returns:
        List of text strings from relevant chunks only

    Example:
        >>> relevant_texts = get_relevant_chunk_texts("What is ROS 2?", chunks)
        >>> context = "\\n\\n".join(relevant_texts)
        >>> answer = generate_answer(query, context)
    """
    try:
        result = validate_retrieval(query, chunks)
        relevant_chunk_ids = {
            chunk["chunk_id"] for chunk in result["relevant_chunks"]
        }

        # Filter original chunks to get full texts
        relevant_texts = [
            chunk["text"] for chunk in chunks
            if chunk["chunk_id"] in relevant_chunk_ids
        ]

        return relevant_texts

    except Exception:
        # On error, return all chunk texts
        logger.exception("Validation failed, returning all chunks")
        return [chunk["text"] for chunk in chunks]


def validate_and_format_context(
    query: str,
    chunks: list[dict],
    include_evidence: bool = True
) -> tuple[str, bool]:
    """
    Validate retrieval and format context for answer generation.

    Combines validation with context preparation in one step.

    Args:
        query: User's question
        chunks: List of retrieved chunks
        include_evidence: Whether to include evidence quotes in context

    Returns:
        Tuple of (formatted_context: str, answer_present: bool)

    Example:
        >>> context, answer_present = validate_and_format_context(
        ...     "What is ROS 2?",
        ...     chunks
        ... )
        >>> if answer_present:
        ...     answer = generate_answer(query, context)
        ... else:
        ...     return "No relevant information found"
    """
    try:
        result = validate_retrieval(query, chunks)

        if not result["answer_present"]:
            return "", False

        # Get relevant chunk texts
        relevant_texts = get_relevant_chunk_texts(query, chunks)

        # Optionally include evidence quotes
        if include_evidence and result["evidence"]:
            evidence_section = "\n\n**Key Evidence:**\n" + "\n".join(
                f"- {evidence['quote']}"
                for evidence in result["evidence"][:3]  # Top 3 quotes
            )
            context = "\n\n".join(relevant_texts) + evidence_section
        else:
            context = "\n\n".join(relevant_texts)

        return context, True

    except Exception:
        # On error, return all chunks without validation
        logger.exception("Validation failed, returning unvalidated context")
        context = "\n\n".join(chunk["text"] for chunk in chunks)
        return context, True
