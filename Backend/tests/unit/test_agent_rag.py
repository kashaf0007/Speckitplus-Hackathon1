"""
Comprehensive unit tests for agent_rag.py (T052).

Tests AgentOrchestrator initialization, system prompt construction,
context routing logic, post-processing, and latency tracking.
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
import asyncio
import time

from agent_rag import (
    AgentOrchestrator,
    AgentContext,
    LatencyMetrics,
    NoAnswerFoundError,
    SYSTEM_PROMPT,
    QDRANT_TIMEOUT,
    GEMINI_TIMEOUT,
    COHERE_TIMEOUT
)
from models.agent_models import AskRequest, AskResponse, ChunkReference


class TestAgentOrchestratorInitialization:
    """Tests for AgentOrchestrator initialization."""

    def test_init_with_valid_api_key(self):
        """Test successful initialization with valid API key."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'valid-test-key'}):
            with patch('google.generativeai.configure') as mock_configure:
                with patch('google.generativeai.GenerativeModel') as mock_model:
                    orchestrator = AgentOrchestrator()

                    mock_configure.assert_called_once_with(api_key='valid-test-key')
                    mock_model.assert_called_once_with('gemini-pro')
                    assert orchestrator.model is not None

    def test_init_without_api_key(self):
        """Test initialization fails without API key."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': ''}, clear=True):
            with pytest.raises(ValueError, match='GEMINI_API_KEY not configured'):
                AgentOrchestrator()

    def test_init_with_placeholder_api_key(self):
        """Test initialization fails with placeholder API key."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'your-gemini-api-key-here'}):
            with pytest.raises(ValueError, match='GEMINI_API_KEY not configured'):
                AgentOrchestrator()

    def test_generation_config_values(self):
        """Test that generation config has correct values."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel'):
                    orchestrator = AgentOrchestrator()

                    assert orchestrator.generation_config['temperature'] == 0.1
                    assert orchestrator.generation_config['top_p'] == 0.95
                    assert orchestrator.generation_config['top_k'] == 40
                    assert orchestrator.generation_config['max_output_tokens'] == 300


class TestSystemPrompt:
    """Tests for system prompt construction."""

    def test_system_prompt_contains_strict_rules(self):
        """Test that system prompt contains strict grounding rules."""
        assert "STRICT RULES" in SYSTEM_PROMPT
        assert "NON-NEGOTIABLE" in SYSTEM_PROMPT

    def test_system_prompt_contains_refusal_message(self):
        """Test that system prompt contains exact refusal message."""
        assert "This information is not available in the book." in SYSTEM_PROMPT

    def test_system_prompt_contains_length_constraint(self):
        """Test that system prompt contains length constraint."""
        assert "1-5 sentences" in SYSTEM_PROMPT
        assert "LENGTH CONSTRAINT" in SYSTEM_PROMPT

    def test_system_prompt_contains_output_format(self):
        """Test that system prompt contains output format specification."""
        assert "OUTPUT FORMAT" in SYSTEM_PROMPT

    def test_system_prompt_contains_examples(self):
        """Test that system prompt contains few-shot examples."""
        assert "Example 1" in SYSTEM_PROMPT
        assert "Example 2" in SYSTEM_PROMPT
        assert "Example 3" in SYSTEM_PROMPT

    def test_system_prompt_forbids_external_knowledge(self):
        """Test that system prompt explicitly forbids external knowledge."""
        assert "DO NOT use any external knowledge" in SYSTEM_PROMPT
        assert "training data" in SYSTEM_PROMPT
        assert "world knowledge" in SYSTEM_PROMPT


class TestContextRouting:
    """Tests for context routing logic in prepare_context."""

    @pytest.fixture
    def orchestrator(self):
        """Create AgentOrchestrator with mocked dependencies."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel'):
                    return AgentOrchestrator()

    @pytest.fixture
    def mock_cohere_client(self):
        """Create mock Cohere client."""
        client = Mock()
        mock_embed_response = Mock()
        mock_embed_response.embeddings = [[0.1] * 1024]
        client.embed.return_value = mock_embed_response
        return client

    @pytest.fixture
    def mock_qdrant_client(self):
        """Create mock Qdrant client with results."""
        client = Mock()
        mock_result = Mock()
        mock_result.id = "chunk_1"
        mock_result.payload = {
            "text": "Test chunk content",
            "page": 1,
            "chapter": "Chapter 1",
            "section": "Section 1"
        }
        mock_result.score = 0.9
        client.search.return_value = [mock_result]
        return client

    def test_selected_text_skips_retrieval(self, orchestrator):
        """Test that selected text path skips Qdrant retrieval."""
        request = AskRequest(
            question="What does this mean?",
            selected_text="Some selected passage from the book."
        )

        mock_cohere = Mock()
        mock_qdrant = Mock()

        context = orchestrator.prepare_context(request, mock_cohere, mock_qdrant)

        # Should not call any external services
        mock_cohere.embed.assert_not_called()
        mock_qdrant.search.assert_not_called()

        assert context.context_type == "selected_text"
        assert context.context_text == "Some selected passage from the book."

    def test_retrieval_path_calls_services(self, orchestrator, mock_cohere_client, mock_qdrant_client):
        """Test that retrieval path calls Cohere and Qdrant."""
        request = AskRequest(question="What is ROS 2?")

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client
        )

        mock_cohere_client.embed.assert_called_once()
        mock_qdrant_client.search.assert_called_once()

        assert context.context_type == "retrieval"

    def test_retrieval_requires_clients(self, orchestrator):
        """Test that retrieval path requires Cohere and Qdrant clients."""
        request = AskRequest(question="What is ROS 2?")

        with pytest.raises(ValueError, match="Cohere and Qdrant clients required"):
            orchestrator.prepare_context(request)

    def test_empty_qdrant_results_raises_error(self, orchestrator, mock_cohere_client):
        """Test that empty Qdrant results raise NoAnswerFoundError."""
        request = AskRequest(question="What is ROS 2?")

        mock_qdrant = Mock()
        mock_qdrant.search.return_value = []

        with pytest.raises(NoAnswerFoundError, match="No relevant content found"):
            orchestrator.prepare_context(
                request,
                cohere_client=mock_cohere_client,
                qdrant_client=mock_qdrant
            )

    def test_validation_failure_raises_error(self, orchestrator, mock_cohere_client, mock_qdrant_client):
        """Test that validation failure raises NoAnswerFoundError."""
        request = AskRequest(question="What is ROS 2?")

        def failing_validator(question, chunks):
            return {"answer_present": False, "retrieval_quality": "Poor"}

        with pytest.raises(NoAnswerFoundError, match="Answer not found"):
            orchestrator.prepare_context(
                request,
                cohere_client=mock_cohere_client,
                qdrant_client=mock_qdrant_client,
                validator=failing_validator
            )

    def test_validation_error_continues(self, orchestrator, mock_cohere_client, mock_qdrant_client):
        """Test that validation errors don't block processing."""
        request = AskRequest(question="What is ROS 2?")

        def error_validator(question, chunks):
            raise RuntimeError("Validation service unavailable")

        # Should not raise - continues with None validation_result
        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=error_validator
        )

        assert context.validation_result is None


class TestLatencyMetrics:
    """Tests for latency tracking."""

    def test_latency_metrics_initialization(self):
        """Test LatencyMetrics initializes with None values."""
        metrics = LatencyMetrics()

        assert metrics.embedding_time_ms is None
        assert metrics.retrieval_time_ms is None
        assert metrics.validation_time_ms is None
        assert metrics.generation_time_ms is None
        assert metrics.total_time_ms is None

    def test_latency_metrics_in_retrieval_context(self):
        """Test latency metrics are populated in retrieval path."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel'):
                    orchestrator = AgentOrchestrator()

        request = AskRequest(question="What is ROS 2?")

        mock_cohere = Mock()
        mock_embed_response = Mock()
        mock_embed_response.embeddings = [[0.1] * 1024]
        mock_cohere.embed.return_value = mock_embed_response

        mock_qdrant = Mock()
        mock_result = Mock()
        mock_result.id = "chunk_1"
        mock_result.payload = {"text": "Content", "page": 1, "chapter": "Ch1", "section": "S1"}
        mock_qdrant.search.return_value = [mock_result]

        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere,
            qdrant_client=mock_qdrant
        )

        assert context.latency_metrics.embedding_time_ms is not None
        assert context.latency_metrics.retrieval_time_ms is not None

    def test_latency_metrics_in_selected_text_context(self):
        """Test latency metrics in selected text path (no retrieval)."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel'):
                    orchestrator = AgentOrchestrator()

        request = AskRequest(
            question="What does this mean?",
            selected_text="Selected text content."
        )

        context = orchestrator.prepare_context(request)

        # Selected text path skips embedding and retrieval
        assert context.latency_metrics.embedding_time_ms is None
        assert context.latency_metrics.retrieval_time_ms is None


class TestPostProcessAnswer:
    """Tests for answer post-processing."""

    @pytest.fixture
    def orchestrator(self):
        """Create AgentOrchestrator with mocked dependencies."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel'):
                    return AgentOrchestrator()

    def test_removes_filler_prefix(self, orchestrator):
        """Test that filler prefixes are removed."""
        answer = "Based on the context, ROS 2 is a robotics framework."
        processed = orchestrator._post_process_answer(answer)

        assert not processed.startswith("Based on the context")
        assert "ROS 2 is a robotics framework" in processed

    def test_removes_multiple_filler_prefixes(self, orchestrator):
        """Test various filler prefix patterns."""
        prefixes = [
            "Based on the context,",
            "According to the book,",
            "From the context,",
        ]

        for prefix in prefixes:
            answer = f"{prefix} This is the actual answer."
            processed = orchestrator._post_process_answer(answer)
            assert "This is the actual answer" in processed

    def test_removes_filler_suffix(self, orchestrator):
        """Test that filler suffixes are removed."""
        answer = "ROS 2 is a framework as mentioned in the context."
        processed = orchestrator._post_process_answer(answer)

        assert "as mentioned in the context" not in processed

    def test_truncates_long_answers(self, orchestrator):
        """Test that answers longer than 5 sentences are truncated."""
        sentences = ["This is sentence number " + str(i) + "." for i in range(10)]
        long_answer = " ".join(sentences)

        processed = orchestrator._post_process_answer(long_answer)

        # Count sentences in processed answer
        sentence_count = len([s for s in processed.split('.') if s.strip()])
        assert sentence_count <= 5

    def test_truncates_very_long_answers(self, orchestrator):
        """Test that very long answers are truncated by character limit."""
        # Create answer > 800 chars
        long_answer = "This is a test sentence. " * 50

        processed = orchestrator._post_process_answer(long_answer)

        assert len(processed) <= 800

    def test_preserves_valid_answers(self, orchestrator):
        """Test that valid short answers are preserved."""
        answer = "ROS 2 provides tools for robot development."
        processed = orchestrator._post_process_answer(answer)

        assert processed == answer

    def test_handles_empty_answer(self, orchestrator):
        """Test handling of empty answers."""
        assert orchestrator._post_process_answer("") == ""
        assert orchestrator._post_process_answer(None) is None


class TestGenerateAnswer:
    """Tests for answer generation."""

    @pytest.fixture
    def orchestrator(self):
        """Create AgentOrchestrator with mocked Gemini."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel') as mock_model:
                    mock_model.return_value.generate_content.return_value = Mock(
                        text="ROS 2 is a robotics middleware."
                    )
                    return AgentOrchestrator()

    def test_generate_answer_returns_correct_structure(self, orchestrator):
        """Test that generate_answer returns correct response structure."""
        context = AgentContext(
            context_text="ROS 2 is a robotics framework.",
            context_type="retrieval",
            source_metadata=[{
                "chunk_id": "chunk_1",
                "text": "ROS 2 is a robotics framework.",
                "page": 1,
                "chapter": "Ch1",
                "section": "S1"
            }],
            validation_result={"answer_present": True, "retrieval_quality": "Good"}
        )

        result = asyncio.run(orchestrator.generate_answer("What is ROS 2?", context))

        assert "answer" in result
        assert "sources" in result
        assert "matched_chunks" in result
        assert "grounded" in result
        assert "retrieval_quality" in result

    def test_generate_answer_includes_sources(self, orchestrator):
        """Test that generate_answer includes source references."""
        context = AgentContext(
            context_text="Content",
            context_type="retrieval",
            source_metadata=[
                {"chunk_id": "chunk_1", "text": "Text 1", "page": 1, "chapter": "Ch1", "section": "S1"},
                {"chunk_id": "chunk_2", "text": "Text 2", "page": 2, "chapter": "Ch2", "section": "S2"},
            ],
            validation_result={"answer_present": True, "retrieval_quality": "Good"}
        )

        result = asyncio.run(orchestrator.generate_answer("Question?", context))

        assert len(result["sources"]) == 2
        assert "chunk_1" in result["sources"]
        assert "chunk_2" in result["sources"]

    def test_generate_answer_sets_grounded_true(self, orchestrator):
        """Test that successful generation sets grounded=True."""
        context = AgentContext(
            context_text="Content",
            context_type="retrieval",
            source_metadata=[{"chunk_id": "c1", "text": "T1", "page": 1, "chapter": "Ch", "section": "S"}],
            validation_result={"answer_present": True, "retrieval_quality": "Good"}
        )

        result = asyncio.run(orchestrator.generate_answer("Q?", context))

        assert result["grounded"] is True


class TestRefusalResponse:
    """Tests for refusal response creation."""

    @pytest.fixture
    def orchestrator(self):
        """Create AgentOrchestrator with mocked dependencies."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel'):
                    return AgentOrchestrator()

    def test_refusal_response_exact_message(self, orchestrator):
        """Test that refusal uses exact message."""
        refusal = orchestrator.create_refusal_response()

        assert refusal["answer"] == "This information is not available in the book."

    def test_refusal_response_not_grounded(self, orchestrator):
        """Test that refusal is marked as not grounded."""
        refusal = orchestrator.create_refusal_response()

        assert refusal["grounded"] is False

    def test_refusal_response_empty_sources(self, orchestrator):
        """Test that refusal has empty sources."""
        refusal = orchestrator.create_refusal_response()

        assert refusal["sources"] == []
        assert refusal["matched_chunks"] == []

    def test_refusal_response_with_quality(self, orchestrator):
        """Test that refusal includes retrieval quality."""
        refusal = orchestrator.create_refusal_response(retrieval_quality="Partial")

        assert refusal["retrieval_quality"] == "Partial"


class TestTimeoutConfiguration:
    """Tests for timeout configuration constants."""

    def test_qdrant_timeout_value(self):
        """Test Qdrant timeout is set correctly."""
        assert QDRANT_TIMEOUT == 10

    def test_gemini_timeout_value(self):
        """Test Gemini timeout is set correctly."""
        assert GEMINI_TIMEOUT == 15

    def test_cohere_timeout_value(self):
        """Test Cohere timeout is set correctly."""
        assert COHERE_TIMEOUT == 10
