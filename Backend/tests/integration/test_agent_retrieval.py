"""
Integration tests for agent retrieval flow.

Tests the complete flow: embedding → retrieval → validation → generation
with mocked Qdrant to ensure no hallucinations.
"""

import pytest
from unittest.mock import Mock, AsyncMock, patch
import asyncio

from agent_rag import AgentOrchestrator, NoAnswerFoundError, AgentContext
from models.agent_models import AskRequest, AskResponse, ChunkReference


class TestAgentRetrievalFlow:
    """Integration tests for standard query flow with retrieval."""

    @pytest.fixture
    def orchestrator(self):
        """Create AgentOrchestrator instance for testing."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel') as mock_model:
                    mock_model.return_value.generate_content.return_value = Mock(
                        text="ROS 2 is a robotics framework for building robot applications."
                    )
                    return AgentOrchestrator()

    @pytest.fixture
    def mock_cohere_client(self):
        """Mock Cohere client for embedding."""
        client = Mock()
        client.embed = Mock(return_value=Mock(embeddings=[[0.1] * 1024]))
        return client

    @pytest.fixture
    def mock_qdrant_client(self):
        """Mock Qdrant client with predefined chunks."""
        client = Mock()

        # Mock search result with book chunks
        mock_hit = Mock()
        mock_hit.id = "chunk_123"
        mock_hit.payload = {
            "text": "Chapter 3 explores themes of resilience through the protagonist's journey.",
            "page": 45,
            "chapter": "Chapter 3",
            "section": "Rising Action"
        }

        client.search = Mock(return_value=[mock_hit])
        return client

    @pytest.fixture
    def mock_qdrant_multi_chunk(self):
        """Mock Qdrant client with multiple chunks."""
        client = Mock()

        chunks = []
        for i in range(5):
            mock_hit = Mock()
            mock_hit.id = f"chunk_{i}"
            mock_hit.payload = {
                "text": f"Content chunk {i} about robotics and automation systems.",
                "page": 10 + i,
                "chapter": f"Chapter {i + 1}",
                "section": f"Section {i + 1}"
            }
            chunks.append(mock_hit)

        client.search = Mock(return_value=chunks)
        return client

    @pytest.fixture
    def mock_validator(self):
        """Mock validation function."""
        def validator(question, chunks):
            return {
                "answer_present": True,
                "retrieval_quality": "Good",
                "relevant_chunks": ["chunk_123"]
            }
        return validator

    @pytest.fixture
    def mock_validator_no_answer(self):
        """Mock validation function that says answer not present."""
        def validator(question, chunks):
            return {
                "answer_present": False,
                "retrieval_quality": "Poor",
                "relevant_chunks": []
            }
        return validator

    # ==================== CONTEXT PREPARATION TESTS ====================

    def test_prepare_context_with_retrieval_success(
        self, orchestrator, mock_cohere_client, mock_qdrant_client, mock_validator
    ):
        """Test context preparation with successful retrieval."""
        request = AskRequest(question="What happens in Chapter 3?")

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=mock_validator
        )

        assert context.context_type == "retrieval"
        assert len(context.source_metadata) > 0
        assert context.source_metadata[0]["chunk_id"] == "chunk_123"
        assert context.validation_result["answer_present"] is True

    def test_prepare_context_with_empty_retrieval(
        self, orchestrator, mock_cohere_client
    ):
        """Test context preparation when no chunks retrieved."""
        request = AskRequest(question="What happens in Chapter 3?")

        mock_qdrant_empty = Mock()
        mock_qdrant_empty.search = Mock(return_value=[])

        with pytest.raises(NoAnswerFoundError):
            orchestrator.prepare_context(
                request,
                cohere_client=mock_cohere_client,
                qdrant_client=mock_qdrant_empty,
                validator=None
            )

    def test_prepare_context_validation_answer_not_present(
        self, orchestrator, mock_cohere_client, mock_qdrant_client, mock_validator_no_answer
    ):
        """Test context preparation when validation says answer not present."""
        request = AskRequest(question="What's the weather today?")

        with pytest.raises(NoAnswerFoundError):
            orchestrator.prepare_context(
                request,
                cohere_client=mock_cohere_client,
                qdrant_client=mock_qdrant_client,
                validator=mock_validator_no_answer
            )

    def test_prepare_context_multiple_chunks(
        self, orchestrator, mock_cohere_client, mock_qdrant_multi_chunk, mock_validator
    ):
        """Test context preparation assembles multiple chunks correctly."""
        request = AskRequest(question="Tell me about robotics")

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_multi_chunk,
            validator=mock_validator
        )

        # Should have all 5 chunks
        assert len(context.source_metadata) == 5

        # Context text should include all chunks
        for i in range(5):
            assert f"chunk_{i}" in [m["chunk_id"] for m in context.source_metadata]

    def test_prepare_context_chunk_metadata_extraction(
        self, orchestrator, mock_cohere_client, mock_qdrant_client, mock_validator
    ):
        """Test that chunk metadata is correctly extracted."""
        request = AskRequest(question="What happens in Chapter 3?")

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=mock_validator
        )

        chunk_meta = context.source_metadata[0]
        assert chunk_meta["page"] == 45
        assert chunk_meta["chapter"] == "Chapter 3"
        assert chunk_meta["section"] == "Rising Action"
        assert "resilience" in chunk_meta["text"]

    # ==================== SELECTED TEXT OVERRIDE TESTS ====================

    def test_selected_text_skips_retrieval(self, orchestrator):
        """Test that selected text completely bypasses retrieval."""
        request = AskRequest(
            question="Explain this passage",
            selected_text="The author describes photosynthesis in detail."
        )

        # These should NOT be called
        mock_cohere = Mock()
        mock_qdrant = Mock()
        mock_validator = Mock()

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere,
            qdrant_client=mock_qdrant,
            validator=mock_validator
        )

        # Verify no external calls were made
        mock_cohere.embed.assert_not_called()
        mock_qdrant.search.assert_not_called()
        mock_validator.assert_not_called()

        # Context should be from selected text
        assert context.context_type == "selected_text"
        assert context.context_text == "The author describes photosynthesis in detail."

    def test_selected_text_source_metadata_format(self, orchestrator):
        """Test that selected text has correct source metadata."""
        request = AskRequest(
            question="What does this mean?",
            selected_text="User selected content here."
        )

        context = orchestrator.prepare_context(request)

        assert len(context.source_metadata) == 1
        assert context.source_metadata[0]["chunk_id"] == "selected_text"
        assert context.validation_result is None

    def test_selected_text_no_validation_required(self, orchestrator):
        """Test that selected text doesn't go through validation."""
        request = AskRequest(
            question="Explain this",
            selected_text="Some highlighted content from the book."
        )

        # Create a validator that would fail if called
        def failing_validator(q, c):
            raise Exception("Validator should not be called!")

        context = orchestrator.prepare_context(
            request,
            cohere_client=None,
            qdrant_client=None,
            validator=failing_validator
        )

        # Should succeed without calling validator
        assert context.context_type == "selected_text"

    # ==================== ERROR HANDLING TESTS ====================

    def test_cohere_embed_error_handling(self, orchestrator, mock_qdrant_client):
        """Test error handling when Cohere embedding fails."""
        request = AskRequest(question="What is ROS 2?")

        mock_cohere_error = Mock()
        mock_cohere_error.embed = Mock(side_effect=Exception("Cohere API error"))

        with pytest.raises(Exception) as exc_info:
            orchestrator.prepare_context(
                request,
                cohere_client=mock_cohere_error,
                qdrant_client=mock_qdrant_client,
                validator=None
            )

        assert "Cohere" in str(exc_info.value)

    def test_qdrant_search_error_handling(self, orchestrator, mock_cohere_client):
        """Test error handling when Qdrant search fails."""
        request = AskRequest(question="What is ROS 2?")

        mock_qdrant_error = Mock()
        mock_qdrant_error.search = Mock(side_effect=Exception("Qdrant connection error"))

        with pytest.raises(Exception) as exc_info:
            orchestrator.prepare_context(
                request,
                cohere_client=mock_cohere_client,
                qdrant_client=mock_qdrant_error,
                validator=None
            )

        assert "Qdrant" in str(exc_info.value) or "connection" in str(exc_info.value).lower()

    def test_validation_error_continues_processing(
        self, orchestrator, mock_cohere_client, mock_qdrant_client
    ):
        """Test that validation errors don't crash the system."""
        request = AskRequest(question="What is ROS 2?")

        def failing_validator(q, c):
            raise Exception("Validation service unavailable")

        # Should NOT raise - validation errors are logged but don't fail the request
        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=failing_validator
        )

        # Context should still be created
        assert context.context_type == "retrieval"
        assert context.validation_result is None  # Validation failed but continued

    def test_missing_clients_raises_error(self, orchestrator):
        """Test that missing clients raise appropriate error."""
        request = AskRequest(question="What is ROS 2?")

        with pytest.raises(ValueError) as exc_info:
            orchestrator.prepare_context(
                request,
                cohere_client=None,
                qdrant_client=None,
                validator=None
            )

        assert "Cohere" in str(exc_info.value) and "Qdrant" in str(exc_info.value)

    # ==================== ANSWER GENERATION TESTS ====================

    def test_generate_answer_success(self, orchestrator):
        """Test successful answer generation."""
        context = AgentContext(
            context_text="ROS 2 is a robotics middleware framework.",
            context_type="retrieval",
            source_metadata=[{
                "chunk_id": "chunk_1",
                "text": "ROS 2 is a robotics middleware framework.",
                "page": 1,
                "chapter": "Chapter 1",
                "section": "Introduction"
            }],
            validation_result={"answer_present": True, "retrieval_quality": "Good"}
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="What is ROS 2?",
            context=context
        ))

        assert "answer" in result
        assert result["sources"] == ["chunk_1"]
        assert len(result["matched_chunks"]) == 1
        assert result["grounded"] is True

    def test_generate_answer_includes_chunk_references(self, orchestrator):
        """Test that generated answer includes proper chunk references."""
        context = AgentContext(
            context_text="Node A communicates with Node B via topics.",
            context_type="retrieval",
            source_metadata=[{
                "chunk_id": "chunk_comm",
                "text": "Node A communicates with Node B via topics.",
                "page": 25,
                "chapter": "Chapter 2",
                "section": "Communication"
            }],
            validation_result={"answer_present": True, "retrieval_quality": "Good"}
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="How do nodes communicate?",
            context=context
        ))

        chunk_ref = result["matched_chunks"][0]
        assert isinstance(chunk_ref, ChunkReference)
        assert chunk_ref.chunk_id == "chunk_comm"
        assert chunk_ref.page == 25
        assert chunk_ref.chapter == "Chapter 2"

    def test_generate_answer_selected_text_source(self, orchestrator):
        """Test answer generation from selected text."""
        context = AgentContext(
            context_text="User highlighted passage about sensors.",
            context_type="selected_text",
            source_metadata=[{
                "chunk_id": "selected_text",
                "text": "User highlighted passage about sensors."
            }],
            validation_result=None
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="What does this say?",
            context=context
        ))

        assert result["sources"] == ["selected_text"]
        assert result["retrieval_quality"] is None

    # ==================== REFUSAL RESPONSE TESTS ====================

    def test_create_refusal_response_format(self, orchestrator):
        """Test refusal response has correct format."""
        response = orchestrator.create_refusal_response()

        assert response["answer"] == "This information is not available in the book."
        assert response["sources"] == []
        assert response["matched_chunks"] == []
        assert response["grounded"] is False
        assert response["retrieval_quality"] == "Poor"

    def test_create_refusal_response_custom_quality(self, orchestrator):
        """Test refusal response with custom quality."""
        response = orchestrator.create_refusal_response(retrieval_quality="Partial")

        assert response["retrieval_quality"] == "Partial"

    # ==================== EDGE CASE TESTS ====================

    def test_empty_chunk_text_in_payload(self, orchestrator, mock_cohere_client, mock_validator):
        """Test handling of chunk with empty text."""
        request = AskRequest(question="What is ROS 2?")

        mock_qdrant = Mock()
        mock_hit = Mock()
        mock_hit.id = "chunk_empty"
        mock_hit.payload = {
            "text": "",
            "page": 1,
            "chapter": "Chapter 1",
            "section": "Intro"
        }
        mock_qdrant.search = Mock(return_value=[mock_hit])

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant,
            validator=mock_validator
        )

        # Should still create context
        assert context.context_type == "retrieval"
        assert len(context.source_metadata) == 1
        assert context.source_metadata[0]["text"] == ""

    def test_missing_optional_metadata(self, orchestrator, mock_cohere_client, mock_validator):
        """Test handling of chunks missing optional metadata."""
        request = AskRequest(question="What is ROS 2?")

        mock_qdrant = Mock()
        mock_hit = Mock()
        mock_hit.id = "chunk_minimal"
        mock_hit.payload = {
            "text": "ROS 2 content here."
            # Missing page, chapter, section
        }
        mock_qdrant.search = Mock(return_value=[mock_hit])

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant,
            validator=mock_validator
        )

        chunk_meta = context.source_metadata[0]
        assert chunk_meta["page"] is None
        assert chunk_meta["chapter"] is None
        assert chunk_meta["section"] is None

    def test_very_long_chunk_text_truncation(self, orchestrator, mock_cohere_client, mock_validator):
        """Test that very long chunk text is truncated."""
        request = AskRequest(question="What is ROS 2?")

        long_text = "A" * 5000  # Very long text

        mock_qdrant = Mock()
        mock_hit = Mock()
        mock_hit.id = "chunk_long"
        mock_hit.payload = {
            "text": long_text,
            "page": 1,
            "chapter": "Chapter 1",
            "section": "Intro"
        }
        mock_qdrant.search = Mock(return_value=[mock_hit])

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant,
            validator=mock_validator
        )

        # Text in metadata should be truncated to 2000 chars
        assert len(context.source_metadata[0]["text"]) <= 2000


class TestSelectedTextQueryFlow:
    """Integration tests for selected text query flow (User Story 2)."""

    @pytest.fixture
    def orchestrator(self):
        """Create AgentOrchestrator instance for testing."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel') as mock_model:
                    mock_model.return_value.generate_content.return_value = Mock(
                        text="Based on the selected passage, the author discusses..."
                    )
                    return AgentOrchestrator()

    def test_selected_text_priority_over_retrieval(self, orchestrator):
        """Test that selected text takes absolute priority over retrieval."""
        request = AskRequest(
            question="What does this mean?",
            selected_text="User selected this specific passage."
        )

        # Mock clients that should NOT be called
        mock_cohere = Mock()
        mock_qdrant = Mock()

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere,
            qdrant_client=mock_qdrant,
            validator=None
        )

        # No embedding or search should happen
        mock_cohere.embed.assert_not_called()
        mock_qdrant.search.assert_not_called()

        # Context should be from selected text
        assert context.context_type == "selected_text"

    def test_selected_text_preserves_content(self, orchestrator):
        """Test that selected text content is preserved exactly."""
        original_text = "  Whitespace preserved  \n\nMultiple paragraphs  "
        request = AskRequest(
            question="Explain this",
            selected_text=original_text
        )

        context = orchestrator.prepare_context(request)

        # Text should be stripped by validator but content preserved
        assert "Whitespace preserved" in context.context_text
        assert "Multiple paragraphs" in context.context_text

    @pytest.mark.parametrize("selected_text_length", [10, 100, 1000, 5000, 9999])
    def test_selected_text_various_lengths(self, orchestrator, selected_text_length):
        """Test selected text handling with various lengths."""
        text = "X" * selected_text_length
        request = AskRequest(
            question="What is this?",
            selected_text=text
        )

        context = orchestrator.prepare_context(request)

        assert context.context_type == "selected_text"
        # Text should match (possibly stripped)
        assert len(context.context_text) == selected_text_length

    def test_selected_text_answer_generation(self, orchestrator):
        """Test answer generation from selected text context."""
        context = AgentContext(
            context_text="The photosynthesis process converts light energy.",
            context_type="selected_text",
            source_metadata=[{
                "chunk_id": "selected_text",
                "text": "The photosynthesis process converts light energy."
            }],
            validation_result=None
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="What does photosynthesis do?",
            context=context
        ))

        assert result["sources"] == ["selected_text"]
        assert result["retrieval_quality"] is None
        # Answer should be grounded
        assert result["grounded"] is True


class TestRefusalBehavior:
    """Integration tests for refusal behavior (User Story 3)."""

    @pytest.fixture
    def orchestrator(self):
        """Create AgentOrchestrator instance for testing."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel') as mock_model:
                    mock_model.return_value.generate_content.return_value = Mock(
                        text="This information is not available in the book."
                    )
                    return AgentOrchestrator()

    @pytest.fixture
    def mock_cohere_client(self):
        """Mock Cohere client."""
        client = Mock()
        client.embed = Mock(return_value=Mock(embeddings=[[0.1] * 1024]))
        return client

    @pytest.fixture
    def mock_qdrant_with_irrelevant_chunks(self):
        """Mock Qdrant returning irrelevant chunks."""
        client = Mock()
        mock_hit = Mock()
        mock_hit.id = "chunk_irrelevant"
        mock_hit.payload = {
            "text": "ROS 2 provides real-time support.",
            "page": 1,
            "chapter": "Chapter 1",
            "section": "Introduction"
        }
        client.search = Mock(return_value=[mock_hit])
        return client

    def test_refusal_when_validation_fails(
        self, orchestrator, mock_cohere_client, mock_qdrant_with_irrelevant_chunks
    ):
        """Test that validation failure leads to NoAnswerFoundError."""
        request = AskRequest(question="What's the weather today?")

        def validator_no_answer(q, c):
            return {"answer_present": False, "retrieval_quality": "Poor"}

        with pytest.raises(NoAnswerFoundError):
            orchestrator.prepare_context(
                request,
                cohere_client=mock_cohere_client,
                qdrant_client=mock_qdrant_with_irrelevant_chunks,
                validator=validator_no_answer
            )

    def test_refusal_when_no_chunks_found(self, orchestrator, mock_cohere_client):
        """Test refusal when Qdrant returns no chunks."""
        request = AskRequest(question="Tell me about something not in the book")

        mock_qdrant_empty = Mock()
        mock_qdrant_empty.search = Mock(return_value=[])

        with pytest.raises(NoAnswerFoundError):
            orchestrator.prepare_context(
                request,
                cohere_client=mock_cohere_client,
                qdrant_client=mock_qdrant_empty,
                validator=None
            )

    def test_refusal_message_exact_format(self, orchestrator):
        """Test that refusal message matches exact specification."""
        response = orchestrator.create_refusal_response()

        expected_message = "This information is not available in the book."
        assert response["answer"] == expected_message
        assert len(response["answer"]) == len(expected_message)

    def test_refusal_response_structure(self, orchestrator):
        """Test complete structure of refusal response."""
        response = orchestrator.create_refusal_response()

        # Verify all required fields
        assert "answer" in response
        assert "sources" in response
        assert "matched_chunks" in response
        assert "grounded" in response
        assert "retrieval_quality" in response

        # Verify values
        assert response["sources"] == []
        assert response["matched_chunks"] == []
        assert response["grounded"] is False

    def test_multiple_refusals_consistent(self, orchestrator):
        """Test that multiple refusals produce identical responses."""
        responses = [orchestrator.create_refusal_response() for _ in range(10)]

        for response in responses:
            assert response["answer"] == "This information is not available in the book."
            assert response["sources"] == []
            assert response["grounded"] is False


class TestEdgeCases:
    """
    T053: Additional edge case tests for agent retrieval.

    Tests edge cases including:
    - Empty Qdrant results
    - Contradictory chunks
    - Malformed validation response
    - Unicode handling
    - Special characters
    """

    @pytest.fixture
    def orchestrator(self):
        """Create AgentOrchestrator instance for testing."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel') as mock_model:
                    mock_model.return_value.generate_content.return_value = Mock(
                        text="The book presents different perspectives on this topic."
                    )
                    return AgentOrchestrator()

    @pytest.fixture
    def mock_cohere_client(self):
        """Mock Cohere client."""
        client = Mock()
        client.embed = Mock(return_value=Mock(embeddings=[[0.1] * 1024]))
        return client

    # ==================== EMPTY QDRANT RESULTS ====================

    def test_empty_qdrant_results_raises_no_answer_error(self, orchestrator, mock_cohere_client):
        """Test that empty Qdrant results raise NoAnswerFoundError."""
        request = AskRequest(question="What is mentioned about quantum physics?")

        mock_qdrant = Mock()
        mock_qdrant.search = Mock(return_value=[])

        with pytest.raises(NoAnswerFoundError, match="No relevant content found"):
            orchestrator.prepare_context(
                request,
                cohere_client=mock_cohere_client,
                qdrant_client=mock_qdrant,
                validator=None
            )

    def test_qdrant_returns_none_handled(self, orchestrator, mock_cohere_client):
        """Test handling when Qdrant returns None instead of empty list."""
        request = AskRequest(question="Test question")

        mock_qdrant = Mock()
        mock_qdrant.search = Mock(return_value=None)

        # Should handle None gracefully (treat as empty)
        with pytest.raises((NoAnswerFoundError, TypeError)):
            orchestrator.prepare_context(
                request,
                cohere_client=mock_cohere_client,
                qdrant_client=mock_qdrant,
                validator=None
            )

    # ==================== CONTRADICTORY CHUNKS ====================

    def test_contradictory_chunks_both_included(self, orchestrator, mock_cohere_client):
        """Test that contradictory chunks are both included in context."""
        request = AskRequest(question="What year was the discovery made?")

        mock_qdrant = Mock()
        chunks = [
            Mock(
                id="chunk_1950",
                payload={
                    "text": "The discovery was made in 1950.",
                    "page": 10,
                    "chapter": "Chapter 2",
                    "section": "History"
                }
            ),
            Mock(
                id="chunk_1955",
                payload={
                    "text": "The discovery was made in 1955.",
                    "page": 25,
                    "chapter": "Chapter 3",
                    "section": "Revision"
                }
            )
        ]
        mock_qdrant.search = Mock(return_value=chunks)

        def validator(q, c):
            return {"answer_present": True, "retrieval_quality": "Partial"}

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant,
            validator=validator
        )

        # Both chunks should be included
        assert len(context.source_metadata) == 2
        chunk_ids = [m["chunk_id"] for m in context.source_metadata]
        assert "chunk_1950" in chunk_ids
        assert "chunk_1955" in chunk_ids

        # Both texts should be in context
        assert "1950" in context.context_text
        assert "1955" in context.context_text

    def test_contradictory_chunks_multiple_sources(self, orchestrator, mock_cohere_client):
        """Test that contradictory info from multiple chapters is preserved."""
        request = AskRequest(question="What is the population?")

        mock_qdrant = Mock()
        chunks = [
            Mock(id="ch1", payload={"text": "Population was 1 million.", "page": 5, "chapter": "Ch1", "section": "S1"}),
            Mock(id="ch2", payload={"text": "Population reached 2 million.", "page": 50, "chapter": "Ch5", "section": "S2"}),
            Mock(id="ch3", payload={"text": "Population dropped to 500,000.", "page": 100, "chapter": "Ch10", "section": "S3"})
        ]
        mock_qdrant.search = Mock(return_value=chunks)

        def validator(q, c):
            return {"answer_present": True, "retrieval_quality": "Good"}

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant,
            validator=validator
        )

        # All three perspectives should be in context
        assert len(context.source_metadata) == 3

    # ==================== MALFORMED VALIDATION RESPONSE ====================

    def test_malformed_validation_missing_answer_present(self, orchestrator, mock_cohere_client):
        """Test handling of validation response missing answer_present field."""
        request = AskRequest(question="What is ROS 2?")

        mock_qdrant = Mock()
        mock_qdrant.search = Mock(return_value=[
            Mock(id="c1", payload={"text": "ROS 2 content", "page": 1, "chapter": "Ch1", "section": "S1"})
        ])

        def malformed_validator(q, c):
            return {"retrieval_quality": "Good"}  # Missing answer_present

        # Should raise NoAnswerFoundError because answer_present defaults to False
        with pytest.raises(NoAnswerFoundError):
            orchestrator.prepare_context(
                request,
                cohere_client=mock_cohere_client,
                qdrant_client=mock_qdrant,
                validator=malformed_validator
            )

    def test_malformed_validation_none_response(self, orchestrator, mock_cohere_client):
        """Test handling when validator returns None."""
        request = AskRequest(question="What is ROS 2?")

        mock_qdrant = Mock()
        mock_qdrant.search = Mock(return_value=[
            Mock(id="c1", payload={"text": "ROS 2 content", "page": 1, "chapter": "Ch1", "section": "S1"})
        ])

        def none_validator(q, c):
            return None

        # Should handle gracefully and raise NoAnswerFoundError
        with pytest.raises((NoAnswerFoundError, TypeError, AttributeError)):
            orchestrator.prepare_context(
                request,
                cohere_client=mock_cohere_client,
                qdrant_client=mock_qdrant,
                validator=none_validator
            )

    def test_malformed_validation_wrong_type(self, orchestrator, mock_cohere_client):
        """Test handling when validator returns wrong type."""
        request = AskRequest(question="What is ROS 2?")

        mock_qdrant = Mock()
        mock_qdrant.search = Mock(return_value=[
            Mock(id="c1", payload={"text": "ROS 2 content", "page": 1, "chapter": "Ch1", "section": "S1"})
        ])

        def string_validator(q, c):
            return "This is not a dict"

        # Should continue processing (validation error logged but not blocking)
        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant,
            validator=string_validator
        )

        # Should continue with None validation_result due to error handling
        assert context.validation_result is None

    # ==================== UNICODE AND SPECIAL CHARACTERS ====================

    def test_unicode_in_question(self, orchestrator, mock_cohere_client):
        """Test handling of Unicode characters in question."""
        request = AskRequest(question="What is the meaning of \u00e9t\u00e9 (summer) in French literature?")

        mock_qdrant = Mock()
        mock_qdrant.search = Mock(return_value=[
            Mock(id="c1", payload={"text": "French text with \u00e9t\u00e9", "page": 1, "chapter": "Ch1", "section": "S1"})
        ])

        def validator(q, c):
            return {"answer_present": True, "retrieval_quality": "Good"}

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant,
            validator=validator
        )

        assert context is not None
        assert context.context_type == "retrieval"

    def test_unicode_in_selected_text(self, orchestrator):
        """Test handling of Unicode in selected text."""
        unicode_text = "Japanese: \u65e5\u672c\u8a9e, Chinese: \u4e2d\u6587, Arabic: \u0639\u0631\u0628\u064a"
        request = AskRequest(
            question="What languages are mentioned?",
            selected_text=unicode_text
        )

        context = orchestrator.prepare_context(request)

        assert context.context_type == "selected_text"
        assert "\u65e5\u672c\u8a9e" in context.context_text

    def test_special_characters_in_chunk(self, orchestrator, mock_cohere_client):
        """Test handling of special characters in chunk text."""
        request = AskRequest(question="What are the formulas?")

        mock_qdrant = Mock()
        mock_qdrant.search = Mock(return_value=[
            Mock(
                id="c1",
                payload={
                    "text": "Formula: E = mc\u00b2, temperature: 100\u00b0C, percent: 50%",
                    "page": 1,
                    "chapter": "Ch1",
                    "section": "S1"
                }
            )
        ])

        def validator(q, c):
            return {"answer_present": True, "retrieval_quality": "Good"}

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant,
            validator=validator
        )

        assert "mc\u00b2" in context.context_text
        assert "100\u00b0C" in context.context_text

    def test_newlines_and_tabs_in_chunk(self, orchestrator, mock_cohere_client):
        """Test handling of newlines and tabs in chunk text."""
        request = AskRequest(question="What is the list?")

        mock_qdrant = Mock()
        mock_qdrant.search = Mock(return_value=[
            Mock(
                id="c1",
                payload={
                    "text": "List:\n\t1. First item\n\t2. Second item\n\t3. Third item",
                    "page": 1,
                    "chapter": "Ch1",
                    "section": "S1"
                }
            )
        ])

        def validator(q, c):
            return {"answer_present": True, "retrieval_quality": "Good"}

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant,
            validator=validator
        )

        assert "First item" in context.context_text
        assert "Second item" in context.context_text

    # ==================== BOUNDARY CONDITIONS ====================

    def test_maximum_chunks_retrieved(self, orchestrator, mock_cohere_client):
        """Test handling of maximum number of chunks (10)."""
        request = AskRequest(question="What is everything?")

        mock_qdrant = Mock()
        chunks = [
            Mock(
                id=f"chunk_{i}",
                payload={"text": f"Content {i}", "page": i, "chapter": f"Ch{i}", "section": f"S{i}"}
            )
            for i in range(10)
        ]
        mock_qdrant.search = Mock(return_value=chunks)

        def validator(q, c):
            return {"answer_present": True, "retrieval_quality": "Good"}

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant,
            validator=validator
        )

        assert len(context.source_metadata) == 10

    def test_single_chunk_retrieved(self, orchestrator, mock_cohere_client):
        """Test handling of single chunk retrieval."""
        request = AskRequest(question="What is the single fact?")

        mock_qdrant = Mock()
        mock_qdrant.search = Mock(return_value=[
            Mock(id="only_chunk", payload={"text": "Single fact here.", "page": 1, "chapter": "Ch1", "section": "S1"})
        ])

        def validator(q, c):
            return {"answer_present": True, "retrieval_quality": "Good"}

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant,
            validator=validator
        )

        assert len(context.source_metadata) == 1

    def test_chunk_with_very_high_similarity_score(self, orchestrator, mock_cohere_client):
        """Test handling of chunk with perfect similarity score."""
        request = AskRequest(question="What is ROS 2?")

        mock_qdrant = Mock()
        mock_hit = Mock()
        mock_hit.id = "perfect_match"
        mock_hit.score = 1.0  # Perfect score
        mock_hit.payload = {"text": "ROS 2 is robotics middleware.", "page": 1, "chapter": "Ch1", "section": "S1"}
        mock_qdrant.search = Mock(return_value=[mock_hit])

        def validator(q, c):
            return {"answer_present": True, "retrieval_quality": "Good"}

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant,
            validator=validator
        )

        assert context.source_metadata[0]["chunk_id"] == "perfect_match"

    def test_chunk_with_very_low_similarity_score(self, orchestrator, mock_cohere_client):
        """Test that low-scoring chunks are still processed (filtering is Qdrant's job)."""
        request = AskRequest(question="What is ROS 2?")

        mock_qdrant = Mock()
        mock_hit = Mock()
        mock_hit.id = "low_score"
        mock_hit.score = 0.1  # Low score
        mock_hit.payload = {"text": "Some content.", "page": 1, "chapter": "Ch1", "section": "S1"}
        mock_qdrant.search = Mock(return_value=[mock_hit])

        def validator(q, c):
            return {"answer_present": True, "retrieval_quality": "Partial"}

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant,
            validator=validator
        )

        # Low-scoring chunk should still be included (filtering handled by Qdrant)
        assert len(context.source_metadata) == 1

    # ==================== CONCURRENT ACCESS ====================

    def test_multiple_sequential_requests(self, orchestrator, mock_cohere_client):
        """Test that multiple sequential requests work correctly."""
        mock_qdrant = Mock()
        mock_qdrant.search = Mock(return_value=[
            Mock(id="c1", payload={"text": "Content", "page": 1, "chapter": "Ch1", "section": "S1"})
        ])

        def validator(q, c):
            return {"answer_present": True, "retrieval_quality": "Good"}

        # Process multiple requests sequentially
        for i in range(5):
            request = AskRequest(question=f"Question number {i}?")
            context = orchestrator.prepare_context(
                request,
                cohere_client=mock_cohere_client,
                qdrant_client=mock_qdrant,
                validator=validator
            )
            assert context is not None
            assert context.context_type == "retrieval"
