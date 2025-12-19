"""
Integration tests for retrieval validation

Tests the full validation workflow end-to-end.
"""

import pytest
from unittest.mock import Mock, patch

from retrieval_validation import (
    validate_retrieval,
    validate_retrieval_simple,
    get_relevant_chunk_texts,
    validate_and_format_context,
    ValidationError,
    CohereAPIError
)
from tests.fixtures.sample_chunks import (
    RELEVANT_CHUNKS_ROS2,
    IRRELEVANT_CHUNKS,
    MIXED_CHUNKS,
    CHUNKS_WITH_ANSWER,
    CHUNKS_WITHOUT_ANSWER,
    EMPTY_CHUNKS,
    TEST_QUERIES
)


class TestValidateRetrievalEndToEnd:
    """End-to-end integration tests for validate_retrieval"""

    @patch('retrieval_validation.validator.assess_relevance')
    def test_full_workflow_with_relevant_chunks(self, mock_assess_relevance):
        """Test complete validation workflow with relevant chunks"""
        # Mock Cohere rerank response
        from retrieval_validation.models import RelevanceAssessment
        mock_assess_relevance.return_value = [
            RelevanceAssessment(
                chunk_id="ros2_ch001",
                relevance_score=0.92,
                is_relevant=True,
                summary="ROS 2 is a set of software libraries..."
            ),
            RelevanceAssessment(
                chunk_id="ros2_ch002",
                relevance_score=0.88,
                is_relevant=True,
                summary="Key features of ROS 2 include..."
            )
        ]

        query = "What are the key features of ROS 2?"
        chunks = RELEVANT_CHUNKS_ROS2[:2]

        result = validate_retrieval(query, chunks)

        # Verify result structure
        assert "relevant_chunks" in result
        assert "answer_present" in result
        assert "evidence" in result
        assert "retrieval_quality" in result
        assert "quality_reasoning" in result
        assert "processing_time_ms" in result

        # Verify result values
        assert len(result["relevant_chunks"]) == 2
        assert result["answer_present"] is True
        assert result["retrieval_quality"] in ["Good", "Partial", "Poor"]
        assert result["processing_time_ms"] > 0

    @patch('retrieval_validation.validator.assess_relevance')
    def test_full_workflow_no_relevant_chunks(self, mock_assess_relevance):
        """Test workflow when no chunks are relevant"""
        from retrieval_validation.models import RelevanceAssessment
        mock_assess_relevance.return_value = [
            RelevanceAssessment(
                chunk_id="unrelated_001",
                relevance_score=0.12,
                is_relevant=False,
                summary="Cooking recipes for pasta..."
            )
        ]

        query = "What is quantum computing?"
        chunks = IRRELEVANT_CHUNKS[:1]

        result = validate_retrieval(query, chunks)

        # Should indicate poor quality and no answer
        assert len(result["relevant_chunks"]) == 0
        assert result["answer_present"] is False
        assert len(result["evidence"]) == 0
        assert result["retrieval_quality"] == "Poor"

    @patch('retrieval_validation.validator.assess_relevance')
    def test_full_workflow_with_answer_present(self, mock_assess_relevance):
        """Test workflow detects answer presence correctly"""
        from retrieval_validation.models import RelevanceAssessment
        mock_assess_relevance.return_value = [
            RelevanceAssessment(
                chunk_id="answer_001",
                relevance_score=0.95,
                is_relevant=True,
                summary="The company was founded in 1995..."
            )
        ]

        query = "What year was the company founded?"
        chunks = CHUNKS_WITH_ANSWER[:1]

        result = validate_retrieval(query, chunks)

        # Should detect the year entity and mark answer as present
        assert result["answer_present"] is True
        assert len(result["evidence"]) > 0

    @patch('retrieval_validation.validator.assess_relevance')
    def test_full_workflow_with_mixed_chunks(self, mock_assess_relevance):
        """Test workflow with mixed relevant and irrelevant chunks"""
        from retrieval_validation.models import RelevanceAssessment
        mock_assess_relevance.return_value = [
            RelevanceAssessment(
                chunk_id="ros2_ch001", relevance_score=0.88, is_relevant=True,
                summary="ROS 2 is a set of software..."
            ),
            RelevanceAssessment(
                chunk_id="ros2_ch002", relevance_score=0.85, is_relevant=True,
                summary="Key features include..."
            ),
            RelevanceAssessment(
                chunk_id="unrelated_001", relevance_score=0.12, is_relevant=False,
                summary="Cooking recipes..."
            )
        ]

        query = "What is ROS 2?"
        chunks = MIXED_CHUNKS

        result = validate_retrieval(query, chunks)

        # Should filter out irrelevant chunks
        assert len(result["relevant_chunks"]) == 2
        assert all(r["is_relevant"] for r in result["relevant_chunks"])
        assert result["retrieval_quality"] in ["Good", "Partial"]

    def test_empty_chunks_list(self):
        """Test handling of empty chunks list"""
        query = "What is ROS 2?"
        chunks = EMPTY_CHUNKS

        result = validate_retrieval(query, chunks)

        assert len(result["relevant_chunks"]) == 0
        assert result["answer_present"] is False
        assert len(result["evidence"]) == 0
        assert result["retrieval_quality"] == "Poor"

    def test_invalid_query(self):
        """Test error handling for invalid query"""
        with pytest.raises(ValidationError, match="Query must"):
            validate_retrieval("", [])  # Empty query

    def test_invalid_chunks(self):
        """Test error handling for malformed chunks"""
        query = "Test query"
        bad_chunks = [
            {"chunk_id": "", "text": "Valid text"}  # Empty chunk_id
        ]

        with pytest.raises(ValidationError):
            validate_retrieval(query, bad_chunks)

    @patch('retrieval_validation.validator.assess_relevance')
    def test_timeout_handling(self, mock_assess_relevance):
        """Test timeout handling"""
        import time
        from retrieval_validation.models import RelevanceAssessment

        # Mock slow processing
        def slow_assess(*args, **kwargs):
            time.sleep(0.1)
            return [
                RelevanceAssessment(
                    chunk_id="ch1", relevance_score=0.9, is_relevant=True, summary="S"
                )
            ]

        mock_assess_relevance.side_effect = slow_assess

        query = "Test query"
        chunks = RELEVANT_CHUNKS_ROS2[:1]

        # With very short timeout
        with pytest.raises(Exception):  # TimeoutError or processing continues
            validate_retrieval(query, chunks, timeout_ms=50)


class TestValidateRetrievalSimple:
    """Tests for simplified validation API"""

    @patch('retrieval_validation.validator.validate_retrieval')
    def test_simple_returns_bool(self, mock_validate):
        """Test simple validation returns boolean"""
        mock_validate.return_value = {
            "answer_present": True,
            "relevant_chunks": [],
            "evidence": [],
            "retrieval_quality": "Good",
            "quality_reasoning": "Test",
            "processing_time_ms": 100.0
        }

        result = validate_retrieval_simple("query", [])
        assert result is True
        assert isinstance(result, bool)

    @patch('retrieval_validation.validator.validate_retrieval')
    def test_simple_handles_no_answer(self, mock_validate):
        """Test simple validation when no answer"""
        mock_validate.return_value = {
            "answer_present": False,
            "relevant_chunks": [],
            "evidence": [],
            "retrieval_quality": "Poor",
            "quality_reasoning": "No relevant chunks",
            "processing_time_ms": 50.0
        }

        result = validate_retrieval_simple("query", [])
        assert result is False

    @patch('retrieval_validation.validator.validate_retrieval')
    def test_simple_fails_open_on_error(self, mock_validate):
        """Test simple validation fails open (returns True) on error"""
        mock_validate.side_effect = Exception("API error")

        result = validate_retrieval_simple("query", [])
        # Should fail open and return True
        assert result is True


class TestGetRelevantChunkTexts:
    """Tests for getting relevant chunk texts"""

    @patch('retrieval_validation.validator.validate_retrieval')
    def test_get_relevant_texts(self, mock_validate):
        """Test extraction of relevant chunk texts"""
        mock_validate.return_value = {
            "relevant_chunks": [
                {"chunk_id": "ros2_ch001", "relevance_score": 0.9, "is_relevant": True, "summary": "S1"},
                {"chunk_id": "ros2_ch002", "relevance_score": 0.85, "is_relevant": True, "summary": "S2"}
            ],
            "answer_present": True,
            "evidence": [],
            "retrieval_quality": "Good",
            "quality_reasoning": "Test",
            "processing_time_ms": 100.0
        }

        chunks = RELEVANT_CHUNKS_ROS2[:3]
        relevant_texts = get_relevant_chunk_texts("query", chunks)

        # Should return only texts from first two chunks
        assert len(relevant_texts) == 2
        assert all(isinstance(text, str) for text in relevant_texts)

    @patch('retrieval_validation.validator.validate_retrieval')
    def test_get_relevant_texts_none_relevant(self, mock_validate):
        """Test when no chunks are relevant"""
        mock_validate.return_value = {
            "relevant_chunks": [],
            "answer_present": False,
            "evidence": [],
            "retrieval_quality": "Poor",
            "quality_reasoning": "No relevant chunks",
            "processing_time_ms": 50.0
        }

        relevant_texts = get_relevant_chunk_texts("query", IRRELEVANT_CHUNKS)

        # Should return empty list
        assert len(relevant_texts) == 0

    @patch('retrieval_validation.validator.validate_retrieval')
    def test_get_relevant_texts_error_fallback(self, mock_validate):
        """Test fallback to all chunks on error"""
        mock_validate.side_effect = Exception("Error")

        chunks = RELEVANT_CHUNKS_ROS2[:2]
        relevant_texts = get_relevant_chunk_texts("query", chunks)

        # Should return all chunk texts on error
        assert len(relevant_texts) == 2


class TestValidateAndFormatContext:
    """Tests for validation with context formatting"""

    @patch('retrieval_validation.validator.validate_retrieval')
    def test_format_context_with_evidence(self, mock_validate):
        """Test context formatting with evidence"""
        mock_validate.return_value = {
            "relevant_chunks": [
                {"chunk_id": "ch1", "relevance_score": 0.9, "is_relevant": True, "summary": "S"}
            ],
            "answer_present": True,
            "evidence": [
                {"chunk_id": "ch1", "quote": "ROS 2 is great.", "sentence_index": 0}
            ],
            "retrieval_quality": "Good",
            "quality_reasoning": "Test",
            "processing_time_ms": 100.0
        }

        chunks = RELEVANT_CHUNKS_ROS2[:1]
        context, answer_present = validate_and_format_context("query", chunks, include_evidence=True)

        assert answer_present is True
        assert "ROS 2" in context
        assert "Key Evidence" in context  # Evidence section included

    @patch('retrieval_validation.validator.validate_retrieval')
    def test_format_context_without_evidence(self, mock_validate):
        """Test context formatting without evidence"""
        mock_validate.return_value = {
            "relevant_chunks": [
                {"chunk_id": "ch1", "relevance_score": 0.9, "is_relevant": True, "summary": "S"}
            ],
            "answer_present": True,
            "evidence": [
                {"chunk_id": "ch1", "quote": "Quote", "sentence_index": 0}
            ],
            "retrieval_quality": "Good",
            "quality_reasoning": "Test",
            "processing_time_ms": 100.0
        }

        chunks = RELEVANT_CHUNKS_ROS2[:1]
        context, answer_present = validate_and_format_context("query", chunks, include_evidence=False)

        assert answer_present is True
        assert "ROS 2" in context
        assert "Key Evidence" not in context  # Evidence not included

    @patch('retrieval_validation.validator.validate_retrieval')
    def test_format_context_no_answer(self, mock_validate):
        """Test context formatting when no answer present"""
        mock_validate.return_value = {
            "relevant_chunks": [],
            "answer_present": False,
            "evidence": [],
            "retrieval_quality": "Poor",
            "quality_reasoning": "No relevant chunks",
            "processing_time_ms": 50.0
        }

        context, answer_present = validate_and_format_context("query", IRRELEVANT_CHUNKS)

        assert answer_present is False
        assert context == ""  # Empty context when no answer

    @patch('retrieval_validation.validator.validate_retrieval')
    def test_format_context_error_fallback(self, mock_validate):
        """Test fallback behavior on error"""
        mock_validate.side_effect = Exception("Error")

        chunks = RELEVANT_CHUNKS_ROS2[:2]
        context, answer_present = validate_and_format_context("query", chunks)

        # Should return all chunks on error
        assert answer_present is True  # Fails open
        assert len(context) > 0
