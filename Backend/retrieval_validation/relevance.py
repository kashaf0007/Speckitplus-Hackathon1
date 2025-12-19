"""
Relevance assessment using Cohere rerank API

This module handles chunk relevance scoring to determine which retrieved chunks
are actually relevant to the user's query.
"""

from typing import Optional
import cohere
from cohere import Client

from .models import TextChunk, RelevanceAssessment
from .exceptions import CohereAPIError
from .logger import logger, log_relevance_assessment, log_error


def get_cohere_client(api_key: Optional[str] = None) -> Client:
    """
    Get Cohere client instance.

    Args:
        api_key: Cohere API key (if None, uses COHERE_API_KEY env var)

    Returns:
        Cohere client instance

    Raises:
        CohereAPIError: If API key is missing or invalid
    """
    try:
        if api_key:
            return cohere.Client(api_key)
        else:
            # Uses COHERE_API_KEY environment variable
            return cohere.Client()
    except Exception as e:
        raise CohereAPIError(f"Failed to initialize Cohere client: {str(e)}")


def assess_relevance(
    query: str,
    chunks: list[TextChunk],
    relevance_threshold: float = 0.3,
    cohere_client: Optional[Client] = None
) -> list[RelevanceAssessment]:
    """
    Assess relevance of chunks to the query using Cohere rerank.

    This function uses Cohere's rerank API to score each chunk's relevance to the query.
    Chunks scoring above the threshold are marked as relevant.

    Args:
        query: User's question or information request
        chunks: List of retrieved text chunks
        relevance_threshold: Minimum score for marking relevant (0.0-1.0)
        cohere_client: Optional Cohere client (creates new if None)

    Returns:
        List of relevance assessments for each chunk

    Raises:
        CohereAPIError: If Cohere API call fails

    Example:
        >>> assessments = assess_relevance(
        ...     query="What is ROS 2?",
        ...     chunks=[chunk1, chunk2],
        ...     relevance_threshold=0.3
        ... )
        >>> relevant = [a for a in assessments if a.is_relevant]
    """
    # Handle empty chunks
    if not chunks:
        logger.debug("No chunks to assess")
        return []

    # Get Cohere client
    if cohere_client is None:
        cohere_client = get_cohere_client()

    # Prepare documents for rerank
    documents = [chunk.text for chunk in chunks]

    try:
        # Call Cohere rerank API (batch processing)
        logger.debug(f"Calling Cohere rerank for {len(chunks)} chunks")
        response = cohere_client.rerank(
            model="rerank-english-v3.0",
            query=query,
            documents=documents,
            top_n=len(documents),  # Return all documents with scores
            return_documents=False  # We already have the documents
        )

        # Create relevance assessments
        assessments = []
        for result in response.results:
            chunk = chunks[result.index]
            relevance_score = result.relevance_score
            is_relevant = relevance_score >= relevance_threshold

            # Create summary (first 150 chars of text)
            summary = chunk.text[:147] + "..." if len(chunk.text) > 150 else chunk.text

            assessment = RelevanceAssessment(
                chunk_id=chunk.chunk_id,
                relevance_score=relevance_score,
                is_relevant=is_relevant,
                summary=summary
            )
            assessments.append(assessment)

            # Log assessment
            log_relevance_assessment(chunk.chunk_id, relevance_score, is_relevant)

        # Sort by relevance score (highest first)
        assessments.sort(key=lambda x: x.relevance_score, reverse=True)

        logger.debug(
            f"Relevance assessment complete: "
            f"{sum(1 for a in assessments if a.is_relevant)}/{len(assessments)} relevant"
        )

        return assessments

    except cohere.CohereAPIError as e:
        error_msg = f"Cohere rerank API error: {str(e)}"
        log_error("CohereAPIError", error_msg)
        raise CohereAPIError(error_msg)
    except cohere.CohereConnectionError as e:
        error_msg = f"Cohere connection error: {str(e)}"
        log_error("CohereConnectionError", error_msg)
        raise CohereAPIError(error_msg)
    except Exception as e:
        error_msg = f"Unexpected error during relevance assessment: {str(e)}"
        log_error("UnexpectedError", error_msg)
        raise CohereAPIError(error_msg)


def filter_relevant_chunks(
    assessments: list[RelevanceAssessment]
) -> list[RelevanceAssessment]:
    """
    Filter to only relevant chunks.

    Args:
        assessments: List of all relevance assessments

    Returns:
        List containing only relevant assessments (is_relevant=True)

    Example:
        >>> relevant = filter_relevant_chunks(all_assessments)
        >>> print(f"Found {len(relevant)} relevant chunks")
    """
    return [a for a in assessments if a.is_relevant]


def calculate_average_relevance_score(
    assessments: list[RelevanceAssessment]
) -> float:
    """
    Calculate average relevance score across all assessments.

    Args:
        assessments: List of relevance assessments

    Returns:
        Average relevance score (0.0-1.0), or 0.0 if no assessments

    Example:
        >>> avg_score = calculate_average_relevance_score(assessments)
        >>> print(f"Average relevance: {avg_score:.2f}")
    """
    if not assessments:
        return 0.0

    total_score = sum(a.relevance_score for a in assessments)
    return total_score / len(assessments)


def fallback_relevance_assessment(
    chunks: list[TextChunk],
    relevance_threshold: float = 0.7
) -> list[RelevanceAssessment]:
    """
    Fallback relevance assessment using original Qdrant scores.

    Used when Cohere rerank API is unavailable. Less accurate than rerank,
    but provides basic relevance filtering based on Qdrant similarity scores.

    Args:
        chunks: List of retrieved text chunks with scores
        relevance_threshold: Minimum Qdrant score for marking relevant

    Returns:
        List of relevance assessments based on Qdrant scores

    Note:
        This is a fallback mechanism. Cohere rerank is significantly more
        accurate for relevance assessment.
    """
    logger.warning("Using fallback relevance assessment (Qdrant scores only)")

    assessments = []
    for chunk in chunks:
        # Use Qdrant score if available, otherwise assume low relevance
        relevance_score = chunk.score if chunk.score is not None else 0.5
        is_relevant = relevance_score >= relevance_threshold

        # Create summary
        summary = chunk.text[:147] + "..." if len(chunk.text) > 150 else chunk.text

        assessment = RelevanceAssessment(
            chunk_id=chunk.chunk_id,
            relevance_score=relevance_score,
            is_relevant=is_relevant,
            summary=summary
        )
        assessments.append(assessment)

        log_relevance_assessment(chunk.chunk_id, relevance_score, is_relevant)

    # Sort by relevance score (highest first)
    assessments.sort(key=lambda x: x.relevance_score, reverse=True)

    return assessments
