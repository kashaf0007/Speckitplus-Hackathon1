"""
Logging configuration for Retrieval Validation Assistant

This module provides structured logging for all validation operations.
"""

import logging
import sys
from typing import Optional


def get_logger(name: str, level: Optional[int] = None) -> logging.Logger:
    """
    Get a configured logger for retrieval validation.

    Args:
        name: Logger name (typically __name__ of the calling module)
        level: Logging level (default: INFO)

    Returns:
        Configured logger instance

    Example:
        >>> logger = get_logger(__name__)
        >>> logger.info("Validation started", extra={"request_id": "req-123"})
    """
    logger = logging.getLogger(name)

    # Set level
    if level is None:
        level = logging.INFO
    logger.setLevel(level)

    # Avoid adding multiple handlers if logger already configured
    if not logger.handlers:
        # Create console handler with formatting
        handler = logging.StreamHandler(sys.stdout)
        handler.setLevel(level)

        # Create formatter
        formatter = logging.Formatter(
            fmt='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        handler.setFormatter(formatter)

        # Add handler to logger
        logger.addHandler(handler)

    return logger


# Module-level logger for retrieval_validation
logger = get_logger('retrieval_validation')


def log_validation_start(request_id: Optional[str], query: str, chunk_count: int):
    """Log the start of a validation request"""
    logger.info(
        f"Validation started: request_id={request_id}, "
        f"query_length={len(query)}, chunk_count={chunk_count}"
    )


def log_validation_complete(request_id: Optional[str], processing_time_ms: float,
                           quality: str, answer_present: bool):
    """Log the completion of a validation request"""
    logger.info(
        f"Validation complete: request_id={request_id}, "
        f"processing_time_ms={processing_time_ms:.2f}, "
        f"quality={quality}, answer_present={answer_present}"
    )


def log_relevance_assessment(chunk_id: str, relevance_score: float, is_relevant: bool):
    """Log relevance assessment for a chunk"""
    logger.debug(
        f"Relevance assessed: chunk_id={chunk_id}, "
        f"score={relevance_score:.3f}, is_relevant={is_relevant}"
    )


def log_answer_detection(answer_present: bool, relevant_chunk_count: int):
    """Log answer presence detection result"""
    logger.debug(
        f"Answer detection: answer_present={answer_present}, "
        f"relevant_chunks={relevant_chunk_count}"
    )


def log_evidence_extraction(chunk_id: str, quote_count: int):
    """Log evidence extraction for a chunk"""
    logger.debug(
        f"Evidence extracted: chunk_id={chunk_id}, quote_count={quote_count}"
    )


def log_quality_rating(quality: str, avg_score: float, relevant_count: int):
    """Log quality rating calculation"""
    logger.debug(
        f"Quality rated: quality={quality}, "
        f"avg_score={avg_score:.3f}, relevant_count={relevant_count}"
    )


def log_error(error_type: str, message: str, request_id: Optional[str] = None):
    """Log validation errors"""
    logger.error(
        f"Validation error: type={error_type}, "
        f"message={message}, request_id={request_id}"
    )


def log_warning(warning_type: str, message: str, request_id: Optional[str] = None):
    """Log validation warnings"""
    logger.warning(
        f"Validation warning: type={warning_type}, "
        f"message={message}, request_id={request_id}"
    )
