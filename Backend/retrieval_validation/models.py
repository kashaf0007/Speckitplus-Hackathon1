"""
Pydantic data models for Retrieval Validation Assistant

This module defines all data entities used in the validation pipeline:
- TextChunk: Retrieved text segments from Qdrant
- ValidationRequest: Input for validation process
- RelevanceAssessment: Per-chunk relevance scoring result
- EvidenceQuote: Extracted evidence with attribution
- RetrievalQuality: Quality rating enum (Good/Partial/Poor)
- ValidationResult: Complete validation outcome
"""

from enum import Enum
from typing import Optional

from pydantic import BaseModel, Field, field_validator


class TextChunk(BaseModel):
    """
    A single text chunk retrieved from the vector database.

    Attributes:
        chunk_id: Unique identifier from Qdrant
        text: The actual content of the chunk (1-5000 chars)
        metadata: Optional metadata (chapter, section, source URL, etc.)
        score: Original Qdrant similarity score (0.0-1.0)
    """
    chunk_id: str = Field(..., min_length=1)
    text: str = Field(..., min_length=1, max_length=5000)
    metadata: Optional[dict] = None
    score: Optional[float] = Field(None, ge=0.0, le=1.0)


class ValidationRequest(BaseModel):
    """
    Input request for retrieval validation.

    Attributes:
        query: User's question or information request (3-500 chars)
        chunks: List of retrieved chunks from Qdrant (max 50)
        relevance_threshold: Minimum relevance score for marking relevant (0.0-1.0)
        request_id: Optional ID for tracing/logging
    """
    query: str = Field(..., min_length=3, max_length=500)
    chunks: list[TextChunk] = Field(default_factory=list, max_length=50)
    relevance_threshold: float = Field(default=0.3, ge=0.0, le=1.0)
    request_id: Optional[str] = None

    @field_validator('query')
    @classmethod
    def query_not_empty(cls, v: str) -> str:
        """Validate query is not empty or whitespace-only"""
        if not v.strip():
            raise ValueError('Query must not be empty or whitespace-only')
        return v


class RelevanceAssessment(BaseModel):
    """
    Per-chunk relevance scoring result.

    Attributes:
        chunk_id: Reference to the assessed chunk
        relevance_score: Normalized relevance score from Cohere rerank (0.0-1.0)
        is_relevant: True if score >= threshold
        summary: Brief description of chunk content (first 100 chars)
    """
    chunk_id: str
    relevance_score: float = Field(..., ge=0.0, le=1.0)
    is_relevant: bool
    summary: str = Field(..., max_length=150)


class EvidenceQuote(BaseModel):
    """
    Extracted text supporting the answer, with attribution.

    Attributes:
        chunk_id: Source chunk ID for traceability
        quote: Exact text extracted from chunk (verbatim, no paraphrasing)
        sentence_index: Position of sentence within chunk (0-indexed)
        context_before: Previous sentence for context (optional)
        context_after: Next sentence for context (optional)
    """
    chunk_id: str
    quote: str = Field(..., min_length=1)
    sentence_index: int = Field(..., ge=0)
    context_before: Optional[str] = None
    context_after: Optional[str] = None


class RetrievalQuality(str, Enum):
    """
    Overall quality rating for retrieval results.

    Values:
        GOOD: >=2 relevant chunks, answer fully present, avg score >= 0.6
        PARTIAL: 1 relevant chunk, or answer partially present, or avg score 0.3-0.6
        POOR: 0 relevant chunks, or avg score < 0.3
    """
    GOOD = "Good"
    PARTIAL = "Partial"
    POOR = "Poor"


class ValidationResult(BaseModel):
    """
    Complete validation outcome with all assessments and evidence.

    Attributes:
        relevant_chunks: Chunks deemed relevant to the query
        answer_present: Whether answer is explicitly in retrieved data
        evidence: Exact quotes supporting the answer (empty if answer_present=False)
        retrieval_quality: Overall quality rating (Good/Partial/Poor)
        quality_reasoning: Human-readable explanation of the quality rating
        processing_time_ms: Time taken for validation (for observability)
    """
    relevant_chunks: list[RelevanceAssessment]
    answer_present: bool
    evidence: list[EvidenceQuote] = Field(default_factory=list)
    retrieval_quality: RetrievalQuality
    quality_reasoning: str
    processing_time_ms: float = Field(..., gt=0)

    @field_validator('evidence')
    @classmethod
    def evidence_requires_answer(cls, v: list[EvidenceQuote], info) -> list[EvidenceQuote]:
        """Validate that evidence is only provided when answer is present"""
        if not info.data.get('answer_present') and v:
            raise ValueError('Evidence must be empty when answer_present is False')
        return v
