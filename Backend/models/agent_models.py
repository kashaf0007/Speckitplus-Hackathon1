"""
Pydantic models for RAG agent request/response handling.

These models define the API contract for the /ask endpoint and internal
data structures for agent orchestration.
"""

from typing import List, Optional
from pydantic import BaseModel, Field, validator


class AskRequest(BaseModel):
    """
    Request model for /ask endpoint.

    Represents a user question with optional selected text context.
    """
    question: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="User's question about the book content"
    )
    selected_text: Optional[str] = Field(
        None,
        max_length=10000,
        description="Optional user-highlighted passage that overrides retrieval"
    )

    @validator('question')
    def question_not_empty(cls, v):
        """Ensure question is not just whitespace."""
        if not v or not v.strip():
            raise ValueError('Question must not be empty or whitespace only')
        return v.strip()

    @validator('selected_text')
    def selected_text_strip(cls, v):
        """Strip whitespace from selected text if provided."""
        if v:
            return v.strip()
        return v


class ChunkReference(BaseModel):
    """
    Detailed information about a content chunk used in the answer.

    Contains the chunk text and metadata for source attribution.
    """
    chunk_id: str = Field(
        ...,
        description="Unique identifier (from Qdrant or 'selected_text')"
    )
    text: str = Field(
        ...,
        max_length=2000,
        description="Full text of the chunk (truncated if longer)"
    )
    page: Optional[int] = Field(
        None,
        ge=1,
        description="Page number in the book (null for selected text or web content)"
    )
    chapter: Optional[str] = Field(
        None,
        description="Chapter name/number (null if not available)"
    )
    section: Optional[str] = Field(
        None,
        description="Section name (null if not available)"
    )

    @validator('chunk_id', 'text')
    def not_empty(cls, v):
        """Ensure required fields are not empty."""
        if not v or not v.strip():
            raise ValueError('Field must not be empty')
        return v.strip()


class AskResponse(BaseModel):
    """
    Response model for /ask endpoint.

    Contains the agent's answer with sources and grounding information.
    """
    answer: str = Field(
        ...,
        description="Agent's generated answer (1-5 sentences) or refusal message"
    )
    sources: List[str] = Field(
        ...,
        description="List of source identifiers (chunk IDs or 'selected_text')"
    )
    matched_chunks: List[ChunkReference] = Field(
        ...,
        description="Full chunk details with text and metadata"
    )
    grounded: bool = Field(
        ...,
        description="Whether the answer is grounded in book content"
    )
    retrieval_quality: Optional[str] = Field(
        None,
        description="Quality assessment: 'Good', 'Partial', 'Poor', or null if selected text"
    )

    @validator('answer')
    def answer_not_empty(cls, v):
        """Ensure answer is not empty."""
        if not v or not v.strip():
            raise ValueError('Answer must not be empty')
        return v.strip()

    @validator('retrieval_quality')
    def valid_quality(cls, v):
        """Ensure retrieval quality is valid if provided."""
        if v and v not in ['Good', 'Partial', 'Poor']:
            raise ValueError('Retrieval quality must be Good, Partial, or Poor')
        return v
