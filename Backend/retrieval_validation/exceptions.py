"""
Custom exceptions for Retrieval Validation Assistant

This module defines all custom exceptions used in the validation pipeline.
"""


class ValidationError(Exception):
    """
    Raised when input data is invalid.

    Examples:
        - Empty or whitespace-only query
        - Malformed chunks (missing chunk_id or text)
        - Duplicate chunk IDs
        - Relevance threshold out of range [0.0, 1.0]
        - More than 50 chunks provided
    """
    pass


class CohereAPIError(Exception):
    """
    Raised when Cohere rerank API fails.

    Examples:
        - API rate limit exceeded (429)
        - API authentication failure (401)
        - API service unavailable (503)
        - Network timeout
        - Invalid API response
    """
    pass


class TimeoutError(Exception):
    """
    Raised when validation processing exceeds timeout.

    The absolute timeout is 5000ms (5 seconds). This ensures validation
    completes within a reasonable time and doesn't block downstream systems.
    """
    pass
