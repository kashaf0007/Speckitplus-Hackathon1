"""
Answer quality tests for User Story 4.

Tests that answers are:
- 1-5 sentences (concise)
- No opinions or interpretations
- Grounded in source chunks
- Direct and factual
"""

import pytest
from unittest.mock import Mock, patch
import asyncio
import re

from agent_rag import AgentOrchestrator, AgentContext
from models.agent_models import AskRequest, AskResponse, ChunkReference


# Sample chunks for testing answer quality
SAMPLE_CHUNKS = [
    {
        "chunk_id": "ros2_intro",
        "text": "ROS 2 (Robot Operating System 2) is a set of software libraries and tools for building robot applications. It provides hardware abstraction, device drivers, communication middleware, and tools for visualization and debugging.",
        "page": 1,
        "chapter": "Chapter 1",
        "section": "Introduction"
    },
    {
        "chunk_id": "ros2_features",
        "text": "Key features of ROS 2 include real-time support, cross-platform compatibility (Linux, Windows, macOS), improved security through SROS2, and support for multi-robot systems.",
        "page": 15,
        "chapter": "Chapter 2",
        "section": "Features"
    },
    {
        "chunk_id": "ros2_nodes",
        "text": "A node in ROS 2 is a fundamental building block that performs computation. Nodes communicate with each other using topics for publish-subscribe patterns, services for request-response patterns, and actions for long-running tasks with feedback.",
        "page": 30,
        "chapter": "Chapter 3",
        "section": "Nodes"
    }
]


class TestAnswerLength:
    """Tests for answer length constraint (1-5 sentences)."""

    @pytest.fixture
    def orchestrator(self):
        """Create AgentOrchestrator with mocked Gemini."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel') as mock_model:
                    # Mock returns a reasonable length answer
                    mock_model.return_value.generate_content.return_value = Mock(
                        text="ROS 2 is a robotics middleware framework. It provides tools for building robot applications."
                    )
                    return AgentOrchestrator()

    def _count_sentences(self, text: str) -> int:
        """Count sentences in text using basic heuristics."""
        # Split on sentence-ending punctuation
        sentences = re.split(r'[.!?]+', text)
        # Filter out empty strings
        sentences = [s.strip() for s in sentences if s.strip()]
        return len(sentences)

    def test_answer_within_sentence_limit(self, orchestrator):
        """Test that generated answers are 1-5 sentences."""
        context = AgentContext(
            context_text=SAMPLE_CHUNKS[0]["text"],
            context_type="retrieval",
            source_metadata=[SAMPLE_CHUNKS[0]],
            validation_result={"answer_present": True, "retrieval_quality": "Good"}
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="What is ROS 2?",
            context=context
        ))

        sentence_count = self._count_sentences(result["answer"])
        assert 1 <= sentence_count <= 5, f"Answer has {sentence_count} sentences, expected 1-5"

    def test_answer_not_empty(self, orchestrator):
        """Test that answers are not empty."""
        context = AgentContext(
            context_text=SAMPLE_CHUNKS[0]["text"],
            context_type="retrieval",
            source_metadata=[SAMPLE_CHUNKS[0]],
            validation_result={"answer_present": True, "retrieval_quality": "Good"}
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="What is ROS 2?",
            context=context
        ))

        assert len(result["answer"].strip()) > 0, "Answer should not be empty"

    def test_answer_character_limit(self, orchestrator):
        """Test that answers don't exceed reasonable character limit."""
        context = AgentContext(
            context_text=SAMPLE_CHUNKS[0]["text"],
            context_type="retrieval",
            source_metadata=[SAMPLE_CHUNKS[0]],
            validation_result={"answer_present": True, "retrieval_quality": "Good"}
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="What is ROS 2?",
            context=context
        ))

        # 5 sentences should be under ~500 characters typically
        assert len(result["answer"]) <= 1000, "Answer exceeds character limit"


class TestAnswerGrounding:
    """Tests for answer grounding in source chunks."""

    @pytest.fixture
    def orchestrator(self):
        """Create AgentOrchestrator with mocked Gemini."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel') as mock_model:
                    mock_model.return_value.generate_content.return_value = Mock(
                        text="ROS 2 provides hardware abstraction and communication middleware for robot applications."
                    )
                    return AgentOrchestrator()

    def test_answer_has_sources(self, orchestrator):
        """Test that answers include source references."""
        context = AgentContext(
            context_text=SAMPLE_CHUNKS[0]["text"],
            context_type="retrieval",
            source_metadata=[SAMPLE_CHUNKS[0]],
            validation_result={"answer_present": True, "retrieval_quality": "Good"}
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="What does ROS 2 provide?",
            context=context
        ))

        assert result["grounded"] is True
        assert len(result["sources"]) > 0

    def test_answer_has_matched_chunks(self, orchestrator):
        """Test that answers include matched chunk details."""
        context = AgentContext(
            context_text=SAMPLE_CHUNKS[0]["text"],
            context_type="retrieval",
            source_metadata=[SAMPLE_CHUNKS[0]],
            validation_result={"answer_present": True, "retrieval_quality": "Good"}
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="What does ROS 2 provide?",
            context=context
        ))

        assert len(result["matched_chunks"]) > 0
        chunk = result["matched_chunks"][0]
        assert isinstance(chunk, ChunkReference)
        assert chunk.chunk_id is not None

    def test_multiple_chunks_in_response(self, orchestrator):
        """Test handling of multiple source chunks."""
        context = AgentContext(
            context_text="\n\n".join([c["text"] for c in SAMPLE_CHUNKS[:2]]),
            context_type="retrieval",
            source_metadata=SAMPLE_CHUNKS[:2],
            validation_result={"answer_present": True, "retrieval_quality": "Good"}
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="What are the features of ROS 2?",
            context=context
        ))

        assert len(result["matched_chunks"]) == 2


class TestAnswerFactuality:
    """Tests for factual, opinion-free answers."""

    @pytest.fixture
    def orchestrator(self):
        """Create AgentOrchestrator with mocked Gemini."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel') as mock_model:
                    mock_model.return_value.generate_content.return_value = Mock(
                        text="ROS 2 supports Linux, Windows, and macOS platforms."
                    )
                    return AgentOrchestrator()

    # Opinion indicators that should not appear in factual answers
    OPINION_INDICATORS = [
        "I think",
        "I believe",
        "In my opinion",
        "probably",
        "maybe",
        "might be",
        "could be",
        "I feel",
        "I would say",
        "personally",
        "it seems",
        "arguably",
    ]

    def test_answer_no_opinion_indicators(self, orchestrator):
        """Test that answers don't contain opinion language."""
        context = AgentContext(
            context_text=SAMPLE_CHUNKS[1]["text"],
            context_type="retrieval",
            source_metadata=[SAMPLE_CHUNKS[1]],
            validation_result={"answer_present": True, "retrieval_quality": "Good"}
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="Which platforms does ROS 2 support?",
            context=context
        ))

        answer_lower = result["answer"].lower()
        for indicator in self.OPINION_INDICATORS:
            assert indicator.lower() not in answer_lower, f"Answer contains opinion indicator: {indicator}"

    def test_refusal_message_is_factual(self, orchestrator):
        """Test that refusal message is factual and consistent."""
        refusal = orchestrator.create_refusal_response()

        expected = "This information is not available in the book."
        assert refusal["answer"] == expected

        # No opinion indicators in refusal
        for indicator in self.OPINION_INDICATORS:
            assert indicator.lower() not in refusal["answer"].lower()


class TestAnswerDirectness:
    """Tests for direct, question-addressing answers."""

    @pytest.fixture
    def orchestrator(self):
        """Create AgentOrchestrator with mocked Gemini."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel') as mock_model:
                    mock_model.return_value.generate_content.return_value = Mock(
                        text="Nodes in ROS 2 communicate using topics, services, and actions."
                    )
                    return AgentOrchestrator()

    # Filler phrases that indicate indirect answers
    FILLER_PHRASES = [
        "Great question",
        "That's a good question",
        "Let me explain",
        "Well,",
        "So basically",
        "To put it simply",
        "In other words",
        "As you know",
    ]

    def test_answer_no_filler_phrases(self, orchestrator):
        """Test that answers don't start with filler phrases."""
        context = AgentContext(
            context_text=SAMPLE_CHUNKS[2]["text"],
            context_type="retrieval",
            source_metadata=[SAMPLE_CHUNKS[2]],
            validation_result={"answer_present": True, "retrieval_quality": "Good"}
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="How do ROS 2 nodes communicate?",
            context=context
        ))

        answer_lower = result["answer"].lower()
        for filler in self.FILLER_PHRASES:
            assert not answer_lower.startswith(filler.lower()), f"Answer starts with filler: {filler}"

    def test_answer_addresses_question(self, orchestrator):
        """Test that answer relates to the question topic."""
        context = AgentContext(
            context_text=SAMPLE_CHUNKS[2]["text"],
            context_type="retrieval",
            source_metadata=[SAMPLE_CHUNKS[2]],
            validation_result={"answer_present": True, "retrieval_quality": "Good"}
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="How do ROS 2 nodes communicate?",
            context=context
        ))

        # Answer should contain relevant terms
        answer_lower = result["answer"].lower()
        relevant_terms = ["node", "communicat", "topic", "service", "action"]
        has_relevant_term = any(term in answer_lower for term in relevant_terms)
        assert has_relevant_term, "Answer doesn't address the question topic"


class TestAnswerQualityMetrics:
    """Tests for overall answer quality metrics."""

    @pytest.fixture
    def orchestrator(self):
        """Create AgentOrchestrator with mocked Gemini."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel') as mock_model:
                    mock_model.return_value.generate_content.return_value = Mock(
                        text="ROS 2 is a robotics middleware framework that provides tools for building robot applications."
                    )
                    return AgentOrchestrator()

    def test_retrieval_quality_included(self, orchestrator):
        """Test that retrieval quality is included in response."""
        context = AgentContext(
            context_text=SAMPLE_CHUNKS[0]["text"],
            context_type="retrieval",
            source_metadata=[SAMPLE_CHUNKS[0]],
            validation_result={"answer_present": True, "retrieval_quality": "Good"}
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="What is ROS 2?",
            context=context
        ))

        assert result["retrieval_quality"] is not None
        assert result["retrieval_quality"] in ["Good", "Partial", "Poor"]

    def test_selected_text_no_retrieval_quality(self, orchestrator):
        """Test that selected text responses have null retrieval quality."""
        context = AgentContext(
            context_text="User selected this passage about robotics.",
            context_type="selected_text",
            source_metadata=[{"chunk_id": "selected_text", "text": "User selected this passage about robotics."}],
            validation_result=None
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="What is this about?",
            context=context
        ))

        assert result["retrieval_quality"] is None

    def test_grounded_flag_accuracy(self, orchestrator):
        """Test that grounded flag is set correctly."""
        context = AgentContext(
            context_text=SAMPLE_CHUNKS[0]["text"],
            context_type="retrieval",
            source_metadata=[SAMPLE_CHUNKS[0]],
            validation_result={"answer_present": True, "retrieval_quality": "Good"}
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="What is ROS 2?",
            context=context
        ))

        assert result["grounded"] is True

    def test_refusal_not_grounded(self, orchestrator):
        """Test that refusal responses are marked as not grounded."""
        refusal = orchestrator.create_refusal_response()

        assert refusal["grounded"] is False


class TestAnswerQualityEdgeCases:
    """Edge case tests for answer quality."""

    @pytest.fixture
    def orchestrator(self):
        """Create AgentOrchestrator with mocked Gemini."""
        with patch.dict('os.environ', {'GEMINI_API_KEY': 'test-key'}):
            with patch('google.generativeai.configure'):
                with patch('google.generativeai.GenerativeModel') as mock_model:
                    mock_model.return_value.generate_content.return_value = Mock(
                        text="The information provided discusses ROS 2 features."
                    )
                    return AgentOrchestrator()

    def test_very_short_context(self, orchestrator):
        """Test answer quality with minimal context."""
        context = AgentContext(
            context_text="ROS 2 is good.",
            context_type="retrieval",
            source_metadata=[{
                "chunk_id": "short_chunk",
                "text": "ROS 2 is good.",
                "page": 1,
                "chapter": "Chapter 1",
                "section": "Intro"
            }],
            validation_result={"answer_present": True, "retrieval_quality": "Partial"}
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="Is ROS 2 good?",
            context=context
        ))

        assert result["answer"] is not None
        assert len(result["answer"]) > 0

    def test_long_context_multiple_chunks(self, orchestrator):
        """Test answer quality with multiple long chunks."""
        long_chunks = [
            {
                "chunk_id": f"chunk_{i}",
                "text": f"Content chunk {i}. " * 20,
                "page": i,
                "chapter": f"Chapter {i}",
                "section": f"Section {i}"
            }
            for i in range(5)
        ]

        context = AgentContext(
            context_text="\n\n".join([c["text"] for c in long_chunks]),
            context_type="retrieval",
            source_metadata=long_chunks,
            validation_result={"answer_present": True, "retrieval_quality": "Good"}
        )

        result = asyncio.run(orchestrator.generate_answer(
            question="What is discussed?",
            context=context
        ))

        # Should still produce concise answer despite long context
        assert len(result["matched_chunks"]) == 5
