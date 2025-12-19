"""
Unit tests for relevance assessment module

Tests the Cohere rerank integration and relevance scoring logic.
"""

import pytest
from unittest.mock import Mock, patch, MagicMock

from retrieval_validation.relevance import (
    assess_relevance,
    filter_relevant_chunks,
    calculate_average_relevance_score,
    fallback_relevance_assessment,
    get_cohere_client
)
from retrieval_validation.models import TextChunk, RelevanceAssessment
from retrieval_validation.exceptions import CohereAPIError
from tests.fixtures.sample_chunks import (
    RELEVANT_CHUNKS_ROS2,
    IRRELEVANT_CHUNKS,
    MIXED_CHUNKS,
    EMPTY_CHUNKS
)


class TestGetCohereClient:
    """Tests for Cohere client initialization"""

    def test_get_cohere_client_with_api_key(self):
        """Test client creation with explicit API key"""
        with patch('cohere.Client') as mock_client:
            mock_client.return_value = Mock()
            client = get_cohere_client(api_key="test-key")
            mock_client.assert_called_once_with("test-key")
            assert client is not None

    def test_get_cohere_client_from_env(self):
        """Test client creation from environment variable"""
        with patch('cohere.Client') as mock_client:
            mock_client.return_value = Mock()
            client = get_cohere_client()
            mock_client.assert_called_once_with()
            assert client is not None

    def test_get_cohere_client_error(self):
        """Test error handling when client creation fails"""
        with patch('cohere.Client', side_effect=Exception("API key missing")):
            with pytest.raises(CohereAPIError, match="Failed to initialize"):
                get_cohere_client()


class TestAssessRelevance:
    """Tests for relevance assessment using Cohere rerank"""

    def test_assess_relevance_empty_chunks(self):
        """Test assessment with no chunks"""
        assessments = assess_relevance(
            query="What is ROS 2?",
            chunks=[],
            relevance_threshold=0.3
        )
        assert assessments == []

    @patch('retrieval_validation.relevance.get_cohere_client')
    def test_assess_relevance_all_relevant(self, mock_get_client):
        """Test assessment when all chunks are relevant"""
        # Mock Cohere client and response
        mock_client = Mock()
        mock_response = Mock()
        mock_response.results = [
            Mock(index=0, relevance_score=0.92),
            Mock(index=1, relevance_score=0.88),
            Mock(index=2, relevance_score=0.85)
        ]
        mock_client.rerank.return_value = mock_response
        mock_get_client.return_value = mock_client

        # Create chunks
        chunks = [TextChunk(**chunk) for chunk in RELEVANT_CHUNKS_ROS2]

        # Assess relevance
        assessments = assess_relevance(
            query="What is ROS 2?",
            chunks=chunks,
            relevance_threshold=0.3,
            cohere_client=mock_client
        )

        # Verify all marked as relevant
        assert len(assessments) == 3
        assert all(a.is_relevant for a in assessments)
        assert assessments[0].relevance_score == 0.92  # Sorted by score

    @patch('retrieval_validation.relevance.get_cohere_client')
    def test_assess_relevance_mixed_chunks(self, mock_get_client):
        """Test assessment with mixed relevant/irrelevant chunks"""
        # Mock Cohere client
        mock_client = Mock()
        mock_response = Mock()
        mock_response.results = [
            Mock(index=0, relevance_score=0.88),  # Relevant
            Mock(index=1, relevance_score=0.85),  # Relevant
            Mock(index=2, relevance_score=0.12),  # Irrelevant
            Mock(index=3, relevance_score=0.08)   # Irrelevant
        ]
        mock_client.rerank.return_value = mock_response
        mock_get_client.return_value = mock_client

        # Create chunks
        chunks = [TextChunk(**chunk) for chunk in MIXED_CHUNKS]

        # Assess relevance
        assessments = assess_relevance(
            query="What is ROS 2?",
            chunks=chunks,
            relevance_threshold=0.3,
            cohere_client=mock_client
        )

        # Verify correct filtering
        assert len(assessments) == 4
        relevant = [a for a in assessments if a.is_relevant]
        irrelevant = [a for a in assessments if not a.is_relevant]
        assert len(relevant) == 2
        assert len(irrelevant) == 2

    @patch('retrieval_validation.relevance.get_cohere_client')
    def test_assess_relevance_custom_threshold(self, mock_get_client):
        """Test assessment with custom relevance threshold"""
        # Mock Cohere client
        mock_client = Mock()
        mock_response = Mock()
        mock_response.results = [
            Mock(index=0, relevance_score=0.6),   # Relevant with 0.5 threshold
            Mock(index=1, relevance_score=0.4)    # Irrelevant with 0.5 threshold
        ]
        mock_client.rerank.return_value = mock_response
        mock_get_client.return_value = mock_client

        chunks = [TextChunk(**chunk) for chunk in RELEVANT_CHUNKS_ROS2[:2]]

        # Test with threshold 0.5
        assessments = assess_relevance(
            query="Test query",
            chunks=chunks,
            relevance_threshold=0.5,
            cohere_client=mock_client
        )

        assert len(assessments) == 2
        assert assessments[0].is_relevant  # 0.6 >= 0.5
        assert not assessments[1].is_relevant  # 0.4 < 0.5

    @patch('retrieval_validation.relevance.get_cohere_client')
    def test_assess_relevance_cohere_api_error(self, mock_get_client):
        """Test error handling when Cohere API fails"""
        # Mock Cohere client to raise error
        mock_client = Mock()
        mock_client.rerank.side_effect = Exception("API rate limit exceeded")
        mock_get_client.return_value = mock_client

        chunks = [TextChunk(**chunk) for chunk in RELEVANT_CHUNKS_ROS2[:1]]

        with pytest.raises(CohereAPIError, match="Unexpected error"):
            assess_relevance(
                query="Test query",
                chunks=chunks,
                cohere_client=mock_client
            )

    @patch('retrieval_validation.relevance.get_cohere_client')
    def test_assess_relevance_creates_summaries(self, mock_get_client):
        """Test that assessments include chunk summaries"""
        # Mock Cohere client
        mock_client = Mock()
        mock_response = Mock()
        mock_response.results = [Mock(index=0, relevance_score=0.9)]
        mock_client.rerank.return_value = mock_response
        mock_get_client.return_value = mock_client

        chunks = [TextChunk(**RELEVANT_CHUNKS_ROS2[0])]

        assessments = assess_relevance(
            query="Test query",
            chunks=chunks,
            cohere_client=mock_client
        )

        assert len(assessments) == 1
        assert len(assessments[0].summary) <= 150
        assert assessments[0].summary in chunks[0].text or chunks[0].text in assessments[0].summary


class TestFilterRelevantChunks:
    """Tests for filtering relevant chunks"""

    def test_filter_relevant_chunks_all_relevant(self):
        """Test filtering when all chunks are relevant"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.9, is_relevant=True, summary="Summary 1"
            ),
            RelevanceAssessment(
                chunk_id="ch2", relevance_score=0.8, is_relevant=True, summary="Summary 2"
            )
        ]

        relevant = filter_relevant_chunks(assessments)
        assert len(relevant) == 2

    def test_filter_relevant_chunks_mixed(self):
        """Test filtering with mixed relevant/irrelevant"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.9, is_relevant=True, summary="Summary 1"
            ),
            RelevanceAssessment(
                chunk_id="ch2", relevance_score=0.1, is_relevant=False, summary="Summary 2"
            )
        ]

        relevant = filter_relevant_chunks(assessments)
        assert len(relevant) == 1
        assert relevant[0].chunk_id == "ch1"

    def test_filter_relevant_chunks_none_relevant(self):
        """Test filtering when no chunks are relevant"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.1, is_relevant=False, summary="Summary 1"
            )
        ]

        relevant = filter_relevant_chunks(assessments)
        assert len(relevant) == 0


class TestCalculateAverageRelevanceScore:
    """Tests for average relevance score calculation"""

    def test_calculate_average_single_assessment(self):
        """Test average with single assessment"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.8, is_relevant=True, summary="Summary"
            )
        ]
        avg = calculate_average_relevance_score(assessments)
        assert avg == 0.8

    def test_calculate_average_multiple_assessments(self):
        """Test average with multiple assessments"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.9, is_relevant=True, summary="S1"
            ),
            RelevanceAssessment(
                chunk_id="ch2", relevance_score=0.7, is_relevant=True, summary="S2"
            ),
            RelevanceAssessment(
                chunk_id="ch3", relevance_score=0.5, is_relevant=False, summary="S3"
            )
        ]
        avg = calculate_average_relevance_score(assessments)
        assert avg == pytest.approx(0.7, rel=0.01)  # (0.9 + 0.7 + 0.5) / 3

    def test_calculate_average_empty_list(self):
        """Test average with no assessments"""
        avg = calculate_average_relevance_score([])
        assert avg == 0.0


class TestFallbackRelevanceAssessment:
    """Tests for fallback relevance assessment using Qdrant scores"""

    def test_fallback_with_scores(self):
        """Test fallback using original Qdrant scores"""
        chunks = [TextChunk(**chunk) for chunk in RELEVANT_CHUNKS_ROS2]

        assessments = fallback_relevance_assessment(
            chunks=chunks,
            relevance_threshold=0.7
        )

        assert len(assessments) == 3
        # All chunks have Qdrant scores >= 0.85, so all should be relevant
        assert all(a.is_relevant for a in assessments)

    def test_fallback_without_scores(self):
        """Test fallback when chunks lack Qdrant scores"""
        chunks = [
            TextChunk(chunk_id="ch1", text="Some text", metadata={})
        ]

        assessments = fallback_relevance_assessment(
            chunks=chunks,
            relevance_threshold=0.7
        )

        assert len(assessments) == 1
        # Default score is 0.5, which is < 0.7, so not relevant
        assert not assessments[0].is_relevant

    def test_fallback_creates_summaries(self):
        """Test that fallback creates summaries"""
        chunks = [TextChunk(**RELEVANT_CHUNKS_ROS2[0])]

        assessments = fallback_relevance_assessment(chunks)

        assert len(assessments) == 1
        assert len(assessments[0].summary) <= 150
