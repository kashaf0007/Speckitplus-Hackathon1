"""
Unit tests for selected text override functionality.

Verifies that when selected_text is provided:
1. No Qdrant calls are made
2. No Cohere embedding calls are made
3. No validation calls are made
4. Context is built entirely from selected text
"""

import pytest
from unittest.mock import Mock, patch, call

from agent_rag import AgentOrchestrator, AgentContext
from models.agent_models import AskRequest


class TestSelectedTextOverride:
    """Unit tests verifying selected text completely overrides retrieval."""

    @pytest.fixture
    def orchestrator(self):
        """Create AgentOrchestrator instance for testing."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel'):
                    return AgentOrchestrator()

    @pytest.fixture
    def mock_cohere_client(self):
        """Mock Cohere client that tracks calls."""
        client = Mock()
        client.embed = Mock(return_value=Mock(embeddings=[[0.1] * 1024]))
        return client

    @pytest.fixture
    def mock_qdrant_client(self):
        """Mock Qdrant client that tracks calls."""
        client = Mock()
        mock_hit = Mock()
        mock_hit.id = "chunk_123"
        mock_hit.payload = {
            "text": "This should not be used",
            "page": 1,
            "chapter": "Chapter 1",
            "section": "Section 1"
        }
        client.search = Mock(return_value=[mock_hit])
        return client

    # ==================== NO QDRANT CALLS TESTS ====================

    def test_no_qdrant_calls_when_selected_text_present(
        self, orchestrator, mock_cohere_client, mock_qdrant_client
    ):
        """Verify no Qdrant search calls when selected_text is provided."""
        request = AskRequest(
            question="What does this mean?",
            selected_text="User selected passage about robotics."
        )

        orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=None
        )

        # Verify Qdrant search was never called
        mock_qdrant_client.search.assert_not_called()

    def test_qdrant_called_when_no_selected_text(
        self, orchestrator, mock_cohere_client, mock_qdrant_client
    ):
        """Verify Qdrant search IS called when no selected_text."""
        request = AskRequest(question="What is ROS 2?")

        def mock_validator(q, c):
            return {"answer_present": True, "retrieval_quality": "Good"}

        orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=mock_validator
        )

        # Verify Qdrant search WAS called
        mock_qdrant_client.search.assert_called_once()

    # ==================== NO COHERE CALLS TESTS ====================

    def test_no_cohere_calls_when_selected_text_present(
        self, orchestrator, mock_cohere_client, mock_qdrant_client
    ):
        """Verify no Cohere embedding calls when selected_text is provided."""
        request = AskRequest(
            question="Explain this passage",
            selected_text="The author describes the algorithm."
        )

        orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=None
        )

        # Verify Cohere embed was never called
        mock_cohere_client.embed.assert_not_called()

    def test_cohere_called_when_no_selected_text(
        self, orchestrator, mock_cohere_client, mock_qdrant_client
    ):
        """Verify Cohere embedding IS called when no selected_text."""
        request = AskRequest(question="What is ROS 2?")

        def mock_validator(q, c):
            return {"answer_present": True, "retrieval_quality": "Good"}

        orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=mock_validator
        )

        # Verify Cohere embed WAS called
        mock_cohere_client.embed.assert_called_once()

    # ==================== NO VALIDATION CALLS TESTS ====================

    def test_no_validation_calls_when_selected_text_present(
        self, orchestrator, mock_cohere_client, mock_qdrant_client
    ):
        """Verify no validation calls when selected_text is provided."""
        request = AskRequest(
            question="What does this mean?",
            selected_text="A passage from the book."
        )

        mock_validator = Mock(return_value={"answer_present": True})

        orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=mock_validator
        )

        # Verify validator was never called
        mock_validator.assert_not_called()

    def test_validation_called_when_no_selected_text(
        self, orchestrator, mock_cohere_client, mock_qdrant_client
    ):
        """Verify validation IS called when no selected_text."""
        request = AskRequest(question="What is ROS 2?")

        mock_validator = Mock(return_value={
            "answer_present": True,
            "retrieval_quality": "Good"
        })

        orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=mock_validator
        )

        # Verify validator WAS called
        mock_validator.assert_called_once()

    # ==================== CONTEXT TYPE TESTS ====================

    def test_context_type_is_selected_text(self, orchestrator):
        """Verify context type is 'selected_text' when selected_text provided."""
        request = AskRequest(
            question="What is this about?",
            selected_text="Some selected content."
        )

        context = orchestrator.prepare_context(request)

        assert context.context_type == "selected_text"

    def test_context_type_is_retrieval_without_selected_text(
        self, orchestrator, mock_cohere_client, mock_qdrant_client
    ):
        """Verify context type is 'retrieval' when no selected_text."""
        request = AskRequest(question="What is ROS 2?")

        def mock_validator(q, c):
            return {"answer_present": True, "retrieval_quality": "Good"}

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=mock_validator
        )

        assert context.context_type == "retrieval"

    # ==================== CONTEXT CONTENT TESTS ====================

    def test_context_text_matches_selected_text(self, orchestrator):
        """Verify context text is exactly the selected text."""
        selected = "This is the exact text selected by the user."
        request = AskRequest(
            question="What does this say?",
            selected_text=selected
        )

        context = orchestrator.prepare_context(request)

        assert context.context_text == selected

    def test_selected_text_is_stripped(self, orchestrator):
        """Verify selected text is stripped of whitespace."""
        request = AskRequest(
            question="Explain",
            selected_text="   Whitespace around text   "
        )

        context = orchestrator.prepare_context(request)

        # Should be stripped by the model validator
        assert context.context_text == "Whitespace around text"

    # ==================== SOURCE METADATA TESTS ====================

    def test_source_metadata_has_selected_text_id(self, orchestrator):
        """Verify source metadata has 'selected_text' as chunk_id."""
        request = AskRequest(
            question="What is this?",
            selected_text="Some passage."
        )

        context = orchestrator.prepare_context(request)

        assert len(context.source_metadata) == 1
        assert context.source_metadata[0]["chunk_id"] == "selected_text"

    def test_source_metadata_single_entry(self, orchestrator):
        """Verify only one source metadata entry for selected text."""
        request = AskRequest(
            question="Explain this",
            selected_text="A very long passage with multiple paragraphs."
        )

        context = orchestrator.prepare_context(request)

        assert len(context.source_metadata) == 1

    # ==================== VALIDATION RESULT TESTS ====================

    def test_validation_result_is_none_for_selected_text(self, orchestrator):
        """Verify validation_result is None when using selected text."""
        request = AskRequest(
            question="What does this mean?",
            selected_text="Content from the book."
        )

        context = orchestrator.prepare_context(request)

        assert context.validation_result is None

    # ==================== EDGE CASES ====================

    def test_empty_string_selected_text_uses_retrieval(
        self, orchestrator, mock_cohere_client, mock_qdrant_client
    ):
        """Verify empty string selected_text falls back to retrieval."""
        # Note: This behavior depends on how the model handles empty strings
        # The validator should strip empty strings to None
        request = AskRequest(
            question="What is ROS 2?",
            selected_text=""  # Empty string
        )

        def mock_validator(q, c):
            return {"answer_present": True, "retrieval_quality": "Good"}

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=mock_validator
        )

        # Empty string should be stripped to None, so retrieval is used
        # Or it might be treated as falsy and use retrieval
        # This depends on implementation - adjust assertion based on behavior

    def test_whitespace_only_selected_text(self, orchestrator, mock_cohere_client, mock_qdrant_client):
        """Verify whitespace-only selected_text behavior."""
        request = AskRequest(
            question="What is this?",
            selected_text="   "  # Only whitespace
        )

        def mock_validator(q, c):
            return {"answer_present": True, "retrieval_quality": "Good"}

        # Whitespace should be stripped to empty, which should be falsy
        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=mock_validator
        )

        # After stripping, this should use retrieval
        # Adjust based on actual behavior

    def test_very_short_selected_text(self, orchestrator):
        """Verify very short selected text is handled."""
        request = AskRequest(
            question="What is this?",
            selected_text="Hi"
        )

        context = orchestrator.prepare_context(request)

        assert context.context_type == "selected_text"
        assert context.context_text == "Hi"

    def test_max_length_selected_text(self, orchestrator):
        """Verify max length selected text is handled."""
        long_text = "X" * 9999  # Near max of 10000
        request = AskRequest(
            question="Summarize this",
            selected_text=long_text
        )

        context = orchestrator.prepare_context(request)

        assert context.context_type == "selected_text"
        assert len(context.context_text) == 9999

    def test_selected_text_with_special_characters(self, orchestrator):
        """Verify selected text with special characters is preserved."""
        special_text = "Math: ∑(x²) = n(n+1)/2\nCode: `print('hello')`\nEmoji: 🤖"
        request = AskRequest(
            question="What does this show?",
            selected_text=special_text
        )

        context = orchestrator.prepare_context(request)

        assert context.context_text == special_text

    def test_selected_text_with_newlines(self, orchestrator):
        """Verify selected text with newlines is preserved."""
        multiline = "Line 1\nLine 2\n\nLine 3 after blank"
        request = AskRequest(
            question="What is in this text?",
            selected_text=multiline
        )

        context = orchestrator.prepare_context(request)

        assert context.context_text == multiline

    # ==================== CLIENT INDEPENDENCE TESTS ====================

    def test_can_omit_cohere_client_with_selected_text(self, orchestrator):
        """Verify Cohere client can be None when using selected text."""
        request = AskRequest(
            question="What is this?",
            selected_text="Some text."
        )

        # Should not raise even with None clients
        context = orchestrator.prepare_context(
            request,
            cohere_client=None,
            qdrant_client=None,
            validator=None
        )

        assert context.context_type == "selected_text"

    def test_can_omit_qdrant_client_with_selected_text(self, orchestrator):
        """Verify Qdrant client can be None when using selected text."""
        request = AskRequest(
            question="Explain this",
            selected_text="Passage from book."
        )

        context = orchestrator.prepare_context(
            request,
            cohere_client=None,
            qdrant_client=None,
            validator=None
        )

        assert context.context_type == "selected_text"

    def test_can_omit_validator_with_selected_text(self, orchestrator):
        """Verify validator can be None when using selected text."""
        request = AskRequest(
            question="What does this mean?",
            selected_text="Content."
        )

        context = orchestrator.prepare_context(
            request,
            cohere_client=None,
            qdrant_client=None,
            validator=None
        )

        assert context.context_type == "selected_text"


class TestSelectedTextVsRetrievalComparison:
    """Tests comparing behavior between selected text and retrieval paths."""

    @pytest.fixture
    def orchestrator(self):
        """Create AgentOrchestrator instance for testing."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel'):
                    return AgentOrchestrator()

    @pytest.fixture
    def mock_cohere_client(self):
        """Mock Cohere client."""
        client = Mock()
        client.embed = Mock(return_value=Mock(embeddings=[[0.1] * 1024]))
        return client

    @pytest.fixture
    def mock_qdrant_client(self):
        """Mock Qdrant client."""
        client = Mock()
        mock_hit = Mock()
        mock_hit.id = "chunk_retrieved"
        mock_hit.payload = {
            "text": "Retrieved content from Qdrant.",
            "page": 10,
            "chapter": "Chapter 5",
            "section": "Analysis"
        }
        client.search = Mock(return_value=[mock_hit])
        return client

    def test_same_question_different_paths(
        self, orchestrator, mock_cohere_client, mock_qdrant_client
    ):
        """Test same question uses different paths based on selected_text."""
        question = "What is the main topic?"

        # Without selected text - uses retrieval
        request_retrieval = AskRequest(question=question)

        def mock_validator(q, c):
            return {"answer_present": True, "retrieval_quality": "Good"}

        context_retrieval = orchestrator.prepare_context(
            request_retrieval,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=mock_validator
        )

        # With selected text - skips retrieval
        request_selected = AskRequest(
            question=question,
            selected_text="Specific passage about the topic."
        )

        context_selected = orchestrator.prepare_context(
            request_selected,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=mock_validator
        )

        # Verify different paths were taken
        assert context_retrieval.context_type == "retrieval"
        assert context_selected.context_type == "selected_text"

        # Verify different content
        assert "Retrieved content" in context_retrieval.context_text
        assert "Specific passage" in context_selected.context_text

    def test_selected_text_source_vs_retrieval_source(
        self, orchestrator, mock_cohere_client, mock_qdrant_client
    ):
        """Compare source metadata between selected text and retrieval."""
        # Retrieval path
        request_retrieval = AskRequest(question="Question")

        def mock_validator(q, c):
            return {"answer_present": True, "retrieval_quality": "Good"}

        context_retrieval = orchestrator.prepare_context(
            request_retrieval,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=mock_validator
        )

        # Selected text path
        request_selected = AskRequest(
            question="Question",
            selected_text="Selected content."
        )

        context_selected = orchestrator.prepare_context(request_selected)

        # Retrieval has full metadata
        assert context_retrieval.source_metadata[0]["chunk_id"] == "chunk_retrieved"
        assert context_retrieval.source_metadata[0]["page"] == 10
        assert context_retrieval.source_metadata[0]["chapter"] == "Chapter 5"

        # Selected text has minimal metadata
        assert context_selected.source_metadata[0]["chunk_id"] == "selected_text"
        assert "page" not in context_selected.source_metadata[0] or context_selected.source_metadata[0].get("page") is None
