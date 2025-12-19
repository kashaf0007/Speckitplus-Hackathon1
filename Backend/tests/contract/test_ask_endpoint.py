"""
Contract tests for POST /ask endpoint.

Validates request/response schemas match the OpenAPI specification
in contracts/ask_endpoint.yaml.
"""

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from models.agent_models import AskRequest, AskResponse, ChunkReference


class TestAskEndpointContract:
    """Contract tests for /ask endpoint schema validation."""

    def test_ask_request_schema_valid(self):
        """Test that valid AskRequest passes schema validation."""
        # Valid request with question only
        request = AskRequest(question="What is Chapter 3 about?")
        assert request.question == "What is Chapter 3 about?"
        assert request.selected_text is None

        # Valid request with selected text
        request_with_text = AskRequest(
            question="Explain this passage",
            selected_text="The author describes photosynthesis in detail..."
        )
        assert request_with_text.question == "Explain this passage"
        assert request_with_text.selected_text == "The author describes photosynthesis in detail..."

    def test_ask_request_schema_invalid_empty_question(self):
        """Test that empty question fails validation."""
        with pytest.raises(ValidationError) as exc_info:
            AskRequest(question="")

        # Pydantic V2 uses "string_too_short" for min_length validation
        assert "string_too_short" in str(exc_info.value) or "at least 1 character" in str(exc_info.value)

    def test_ask_request_schema_invalid_whitespace_question(self):
        """Test that whitespace-only question fails validation."""
        with pytest.raises(ValidationError) as exc_info:
            AskRequest(question="   ")

        assert "Question must not be empty" in str(exc_info.value)

    def test_ask_request_schema_question_max_length(self):
        """Test that question exceeding 1000 chars fails validation."""
        long_question = "x" * 1001
        with pytest.raises(ValidationError) as exc_info:
            AskRequest(question=long_question)

        assert "1000" in str(exc_info.value)

    def test_ask_request_schema_selected_text_max_length(self):
        """Test that selected_text exceeding 10000 chars fails validation."""
        long_text = "x" * 10001
        with pytest.raises(ValidationError) as exc_info:
            AskRequest(question="Valid question", selected_text=long_text)

        assert "10000" in str(exc_info.value)

    def test_ask_response_schema_valid_success(self):
        """Test that valid success AskResponse passes schema validation."""
        chunk = ChunkReference(
            chunk_id="chunk_123",
            text="The protagonist faces challenges in Chapter 3.",
            page=45,
            chapter="Chapter 3",
            section="Rising Action"
        )

        response = AskResponse(
            answer="Chapter 3 explores themes of resilience.",
            sources=["chunk_123"],
            matched_chunks=[chunk],
            grounded=True,
            retrieval_quality="Good"
        )

        assert response.answer == "Chapter 3 explores themes of resilience."
        assert response.sources == ["chunk_123"]
        assert len(response.matched_chunks) == 1
        assert response.grounded is True
        assert response.retrieval_quality == "Good"

    def test_ask_response_schema_valid_refusal(self):
        """Test that valid refusal AskResponse passes schema validation."""
        response = AskResponse(
            answer="This information is not available in the book.",
            sources=[],
            matched_chunks=[],
            grounded=False,
            retrieval_quality="Poor"
        )

        assert response.answer == "This information is not available in the book."
        assert response.sources == []
        assert response.matched_chunks == []
        assert response.grounded is False
        assert response.retrieval_quality == "Poor"

    def test_ask_response_schema_valid_selected_text(self):
        """Test that AskResponse with selected text source passes validation."""
        chunk = ChunkReference(
            chunk_id="selected_text",
            text="The author describes photosynthesis...",
            page=None,
            chapter=None,
            section=None
        )

        response = AskResponse(
            answer="This passage describes photosynthesis.",
            sources=["selected_text"],
            matched_chunks=[chunk],
            grounded=True,
            retrieval_quality=None  # Null for selected text
        )

        assert response.sources == ["selected_text"]
        assert response.retrieval_quality is None

    def test_ask_response_schema_invalid_empty_answer(self):
        """Test that empty answer fails validation."""
        with pytest.raises(ValidationError) as exc_info:
            AskResponse(
                answer="",
                sources=[],
                matched_chunks=[],
                grounded=False,
                retrieval_quality=None
            )

        assert "Answer must not be empty" in str(exc_info.value)

    def test_ask_response_schema_invalid_retrieval_quality(self):
        """Test that invalid retrieval_quality fails validation."""
        chunk = ChunkReference(
            chunk_id="chunk_123",
            text="Sample text",
            page=1,
            chapter="Ch1",
            section="Intro"
        )

        with pytest.raises(ValidationError) as exc_info:
            AskResponse(
                answer="Valid answer",
                sources=["chunk_123"],
                matched_chunks=[chunk],
                grounded=True,
                retrieval_quality="Invalid"  # Must be Good, Partial, or Poor
            )

        assert "Good, Partial, or Poor" in str(exc_info.value)

    def test_chunk_reference_schema_valid(self):
        """Test that valid ChunkReference passes schema validation."""
        chunk = ChunkReference(
            chunk_id="chunk_456",
            text="The scientific method involves observation and analysis.",
            page=78,
            chapter="Chapter 5",
            section="Methodology"
        )

        assert chunk.chunk_id == "chunk_456"
        assert chunk.text == "The scientific method involves observation and analysis."
        assert chunk.page == 78
        assert chunk.chapter == "Chapter 5"
        assert chunk.section == "Methodology"

    def test_chunk_reference_schema_invalid_empty_chunk_id(self):
        """Test that empty chunk_id fails validation."""
        with pytest.raises(ValidationError) as exc_info:
            ChunkReference(
                chunk_id="",
                text="Valid text",
                page=1,
                chapter="Ch1",
                section="Intro"
            )

        assert "Field must not be empty" in str(exc_info.value)

    def test_chunk_reference_schema_invalid_empty_text(self):
        """Test that empty text fails validation."""
        with pytest.raises(ValidationError) as exc_info:
            ChunkReference(
                chunk_id="chunk_123",
                text="",
                page=1,
                chapter="Ch1",
                section="Intro"
            )

        assert "Field must not be empty" in str(exc_info.value)

    def test_chunk_reference_schema_invalid_negative_page(self):
        """Test that negative page number fails validation."""
        with pytest.raises(ValidationError) as exc_info:
            ChunkReference(
                chunk_id="chunk_123",
                text="Valid text",
                page=-1,
                chapter="Ch1",
                section="Intro"
            )

        assert "greater than or equal to 1" in str(exc_info.value)

    def test_chunk_reference_schema_optional_metadata(self):
        """Test that ChunkReference with null metadata is valid."""
        chunk = ChunkReference(
            chunk_id="selected_text",
            text="User-selected passage",
            page=None,
            chapter=None,
            section=None
        )

        assert chunk.page is None
        assert chunk.chapter is None
        assert chunk.section is None


# Note: Endpoint integration tests will be added after API endpoint is implemented (T025)
# These tests validate the schema only, not the actual HTTP endpoint behavior.
