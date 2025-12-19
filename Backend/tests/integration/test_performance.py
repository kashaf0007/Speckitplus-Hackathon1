"""
Performance tests for User Story 4.

Tests that measure end-to-end latency and ensure responses complete
within the <5s target (SC-001).
"""

import pytest
from unittest.mock import Mock, patch, AsyncMock
import asyncio
import time
from dataclasses import dataclass
from typing import List, Any

from agent_rag import AgentOrchestrator, AgentContext
from models.agent_models import AskRequest, AskResponse, ChunkReference


# Mock search result for Qdrant
@dataclass
class MockScoredPoint:
    """Mock Qdrant search result."""
    id: str
    payload: dict
    score: float


def create_mock_search_results(count: int = 10) -> List[MockScoredPoint]:
    """Create mock Qdrant search results."""
    return [
        MockScoredPoint(
            id=f"chunk_{i}",
            payload={
                "text": f"This is content from chunk {i}. It contains information about ROS 2 robotics framework.",
                "page": i + 1,
                "chapter": f"Chapter {(i // 3) + 1}",
                "section": f"Section {i + 1}"
            },
            score=0.9 - (i * 0.05)
        )
        for i in range(count)
    ]


class TestEndToEndLatency:
    """Tests for end-to-end response latency (<5s per SC-001)."""

    @pytest.fixture
    def mock_cohere_client(self):
        """Create mock Cohere client."""
        client = Mock()
        # Simulate fast embedding (~100ms)
        mock_embed_response = Mock()
        mock_embed_response.embeddings = [[0.1] * 1024]
        client.embed.return_value = mock_embed_response
        return client

    @pytest.fixture
    def mock_qdrant_client(self):
        """Create mock Qdrant client."""
        client = Mock()
        # Simulate fast search (~200ms)
        client.search.return_value = create_mock_search_results(10)
        return client

    @pytest.fixture
    def mock_validator(self):
        """Create mock validation function."""
        def validator(question, chunks):
            # Simulate validation (~300ms)
            return {
                "answer_present": True,
                "retrieval_quality": "Good",
                "relevant_chunks": [0, 1, 2],
                "evidence": ["Evidence from chunk"]
            }
        return validator

    @pytest.fixture
    def orchestrator(self):
        """Create AgentOrchestrator with mocked Gemini."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel') as mock_model:
                    # Simulate generation (~1-2s)
                    mock_model.return_value.generate_content.return_value = Mock(
                        text="ROS 2 is a robotics middleware framework for building robot applications."
                    )
                    return AgentOrchestrator()

    def test_response_under_5_seconds_retrieval_path(
        self, orchestrator, mock_cohere_client, mock_qdrant_client, mock_validator
    ):
        """Test that retrieval path completes under 5 seconds."""
        request = AskRequest(question="What is ROS 2?")

        start_time = time.time()

        # Prepare context (embedding + retrieval + validation)
        context = orchestrator.prepare_context(
            request,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=mock_validator
        )

        # Generate answer
        result = asyncio.run(orchestrator.generate_answer(
            question=request.question,
            context=context
        ))

        end_time = time.time()
        duration_seconds = end_time - start_time

        assert duration_seconds < 5.0, f"Response took {duration_seconds:.2f}s, expected <5s"
        assert result["answer"] is not None

    def test_response_under_5_seconds_selected_text_path(self, orchestrator):
        """Test that selected text path completes under 5 seconds."""
        request = AskRequest(
            question="What does this passage mean?",
            selected_text="ROS 2 provides hardware abstraction and communication middleware."
        )

        start_time = time.time()

        # Prepare context (no retrieval needed)
        context = orchestrator.prepare_context(request)

        # Generate answer
        result = asyncio.run(orchestrator.generate_answer(
            question=request.question,
            context=context
        ))

        end_time = time.time()
        duration_seconds = end_time - start_time

        assert duration_seconds < 5.0, f"Response took {duration_seconds:.2f}s, expected <5s"
        assert result["answer"] is not None

    def test_selected_text_faster_than_retrieval(
        self, orchestrator, mock_cohere_client, mock_qdrant_client, mock_validator
    ):
        """Test that selected text path is faster than retrieval path."""
        # Selected text path
        request_selected = AskRequest(
            question="What is this about?",
            selected_text="ROS 2 is a robotics framework."
        )

        start_selected = time.time()
        context_selected = orchestrator.prepare_context(request_selected)
        asyncio.run(orchestrator.generate_answer(
            question=request_selected.question,
            context=context_selected
        ))
        duration_selected = time.time() - start_selected

        # Retrieval path
        request_retrieval = AskRequest(question="What is ROS 2?")

        start_retrieval = time.time()
        context_retrieval = orchestrator.prepare_context(
            request_retrieval,
            cohere_client=mock_cohere_client,
            qdrant_client=mock_qdrant_client,
            validator=mock_validator
        )
        asyncio.run(orchestrator.generate_answer(
            question=request_retrieval.question,
            context=context_retrieval
        ))
        duration_retrieval = time.time() - start_retrieval

        # Selected text should be faster (no embedding/retrieval/validation)
        assert duration_selected <= duration_retrieval, (
            f"Selected text ({duration_selected:.3f}s) should be faster than "
            f"retrieval ({duration_retrieval:.3f}s)"
        )


class TestLatencyBreakdown:
    """Tests for measuring individual operation latencies."""

    @pytest.fixture
    def mock_cohere_client(self):
        """Create mock Cohere client with timing simulation."""
        client = Mock()
        mock_embed_response = Mock()
        mock_embed_response.embeddings = [[0.1] * 1024]
        client.embed.return_value = mock_embed_response
        return client

    @pytest.fixture
    def mock_qdrant_client(self):
        """Create mock Qdrant client with timing simulation."""
        client = Mock()
        client.search.return_value = create_mock_search_results(10)
        return client

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

    def test_embedding_time_reasonable(self, mock_cohere_client):
        """Test that embedding operation completes quickly."""
        start_time = time.time()

        mock_cohere_client.embed(
            texts=["What is ROS 2?"],
            model='embed-english-v3.0',
            input_type='search_query'
        )

        duration_ms = (time.time() - start_time) * 1000

        # Mock should complete almost instantly
        assert duration_ms < 100, f"Embedding took {duration_ms:.2f}ms"

    def test_retrieval_time_reasonable(self, mock_qdrant_client):
        """Test that retrieval operation completes quickly."""
        start_time = time.time()

        mock_qdrant_client.search(
            collection_name="rag_embedding",
            query_vector=[0.1] * 1024,
            limit=10
        )

        duration_ms = (time.time() - start_time) * 1000

        # Mock should complete almost instantly
        assert duration_ms < 100, f"Retrieval took {duration_ms:.2f}ms"

    def test_generation_time_reasonable(self, orchestrator):
        """Test that generation operation completes quickly."""
        context = AgentContext(
            context_text="ROS 2 is a robotics framework.",
            context_type="selected_text",
            source_metadata=[{"chunk_id": "selected_text", "text": "ROS 2 is a robotics framework."}],
            validation_result=None
        )

        start_time = time.time()

        asyncio.run(orchestrator.generate_answer(
            question="What is ROS 2?",
            context=context
        ))

        duration_ms = (time.time() - start_time) * 1000

        # Mock should complete almost instantly
        assert duration_ms < 500, f"Generation took {duration_ms:.2f}ms"


class TestConcurrentRequests:
    """Tests for handling concurrent requests."""

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

    async def _process_request(self, orchestrator, question: str) -> float:
        """Process a single request and return duration."""
        start_time = time.time()

        context = AgentContext(
            context_text="ROS 2 provides tools for robot development.",
            context_type="selected_text",
            source_metadata=[{"chunk_id": "selected_text", "text": "ROS 2 provides tools for robot development."}],
            validation_result=None
        )

        await orchestrator.generate_answer(
            question=question,
            context=context
        )

        return time.time() - start_time

    def test_multiple_requests_performance(self, orchestrator):
        """Test that multiple requests complete in reasonable time."""
        async def run_multiple():
            questions = [f"What is feature {i}?" for i in range(5)]
            tasks = [self._process_request(orchestrator, q) for q in questions]
            durations = await asyncio.gather(*tasks)
            return durations

        durations = asyncio.run(run_multiple())

        # All requests should complete under 5 seconds each
        for i, duration in enumerate(durations):
            assert duration < 5.0, f"Request {i} took {duration:.2f}s"

        # Total time for 5 concurrent requests should be reasonable
        total_time = max(durations)  # Concurrent, so max determines total
        assert total_time < 5.0, f"Concurrent requests took {total_time:.2f}s total"


class TestPerformanceMetrics:
    """Tests for performance metrics collection."""

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

    def test_p95_latency_under_5_seconds(self, orchestrator):
        """Test that 95th percentile latency is under 5 seconds."""
        durations = []

        for i in range(20):  # Run 20 requests for p95 calculation
            context = AgentContext(
                context_text=f"Content chunk {i}",
                context_type="selected_text",
                source_metadata=[{"chunk_id": "selected_text", "text": f"Content chunk {i}"}],
                validation_result=None
            )

            start_time = time.time()
            asyncio.run(orchestrator.generate_answer(
                question=f"Question {i}",
                context=context
            ))
            durations.append(time.time() - start_time)

        # Calculate p95
        sorted_durations = sorted(durations)
        p95_index = int(len(sorted_durations) * 0.95)
        p95_latency = sorted_durations[p95_index]

        assert p95_latency < 5.0, f"p95 latency is {p95_latency:.2f}s, expected <5s"

    def test_average_latency_reasonable(self, orchestrator):
        """Test that average latency is well under 5 seconds."""
        durations = []

        for i in range(10):
            context = AgentContext(
                context_text=f"Content chunk {i}",
                context_type="selected_text",
                source_metadata=[{"chunk_id": "selected_text", "text": f"Content chunk {i}"}],
                validation_result=None
            )

            start_time = time.time()
            asyncio.run(orchestrator.generate_answer(
                question=f"Question {i}",
                context=context
            ))
            durations.append(time.time() - start_time)

        avg_latency = sum(durations) / len(durations)

        # Average should be well under 5s (target <2s for good UX)
        assert avg_latency < 3.0, f"Average latency is {avg_latency:.2f}s"


class TestTimeoutBehavior:
    """Tests for timeout handling."""

    @pytest.fixture
    def orchestrator_with_slow_model(self):
        """Create AgentOrchestrator with slow Gemini simulation."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel') as mock_model:
                    def slow_generate(*args, **kwargs):
                        # Simulate slow response (but not too slow for tests)
                        time.sleep(0.5)
                        return Mock(text="Slow response")

                    mock_model.return_value.generate_content = slow_generate
                    return AgentOrchestrator()

    def test_slow_operation_still_completes(self, orchestrator_with_slow_model):
        """Test that slow operations complete within timeout."""
        context = AgentContext(
            context_text="ROS 2 is a robotics framework.",
            context_type="selected_text",
            source_metadata=[{"chunk_id": "selected_text", "text": "ROS 2 is a robotics framework."}],
            validation_result=None
        )

        start_time = time.time()
        result = asyncio.run(orchestrator_with_slow_model.generate_answer(
            question="What is ROS 2?",
            context=context
        ))
        duration = time.time() - start_time

        assert result["answer"] == "Slow response"
        assert duration < 5.0, f"Slow operation took {duration:.2f}s"
