"""
Unit tests for quality assessment module

Tests quality rating logic and metrics calculation.
"""

import pytest

from retrieval_validation.quality import (
    calculate_average_relevance_score,
    count_relevant_chunks,
    assess_retrieval_quality,
    assess_quality_with_thresholds,
    get_quality_recommendations,
    calculate_quality_metrics
)
from retrieval_validation.models import RelevanceAssessment, RetrievalQuality


class TestCalculateAverageRelevanceScore:
    """Tests for average relevance score calculation"""

    def test_average_single_assessment(self):
        """Test average with single assessment"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.8, is_relevant=True, summary="S1"
            )
        ]
        avg = calculate_average_relevance_score(assessments)
        assert avg == 0.8

    def test_average_multiple_assessments(self):
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

    def test_average_empty_list(self):
        """Test average with no assessments"""
        avg = calculate_average_relevance_score([])
        assert avg == 0.0


class TestCountRelevantChunks:
    """Tests for counting relevant chunks"""

    def test_count_all_relevant(self):
        """Test count when all chunks are relevant"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.9, is_relevant=True, summary="S1"
            ),
            RelevanceAssessment(
                chunk_id="ch2", relevance_score=0.8, is_relevant=True, summary="S2"
            )
        ]
        count = count_relevant_chunks(assessments)
        assert count == 2

    def test_count_mixed(self):
        """Test count with mixed relevant/irrelevant"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.9, is_relevant=True, summary="S1"
            ),
            RelevanceAssessment(
                chunk_id="ch2", relevance_score=0.2, is_relevant=False, summary="S2"
            ),
            RelevanceAssessment(
                chunk_id="ch3", relevance_score=0.8, is_relevant=True, summary="S3"
            )
        ]
        count = count_relevant_chunks(assessments)
        assert count == 2

    def test_count_none_relevant(self):
        """Test count when no chunks are relevant"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.1, is_relevant=False, summary="S1"
            )
        ]
        count = count_relevant_chunks(assessments)
        assert count == 0

    def test_count_empty_list(self):
        """Test count with empty list"""
        count = count_relevant_chunks([])
        assert count == 0


class TestAssessRetrievalQuality:
    """Tests for quality assessment"""

    def test_good_quality(self):
        """Test Good quality rating"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.9, is_relevant=True, summary="S1"
            ),
            RelevanceAssessment(
                chunk_id="ch2", relevance_score=0.85, is_relevant=True, summary="S2"
            ),
            RelevanceAssessment(
                chunk_id="ch3", relevance_score=0.2, is_relevant=False, summary="S3"
            )
        ]
        answer_present = True

        quality, reasoning = assess_retrieval_quality(assessments, answer_present)

        assert quality == RetrievalQuality.GOOD
        assert "relevant chunks found" in reasoning.lower()
        assert "answer fully present" in reasoning.lower()

    def test_poor_quality_no_relevant(self):
        """Test Poor quality when no relevant chunks"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.1, is_relevant=False, summary="S1"
            ),
            RelevanceAssessment(
                chunk_id="ch2", relevance_score=0.15, is_relevant=False, summary="S2"
            )
        ]
        answer_present = False

        quality, reasoning = assess_retrieval_quality(assessments, answer_present)

        assert quality == RetrievalQuality.POOR
        assert "no relevant chunks" in reasoning.lower()

    def test_poor_quality_low_scores(self):
        """Test Poor quality with low average scores"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.25, is_relevant=False, summary="S1"
            ),
            RelevanceAssessment(
                chunk_id="ch2", relevance_score=0.28, is_relevant=False, summary="S2"
            )
        ]
        answer_present = False

        quality, reasoning = assess_retrieval_quality(assessments, answer_present)

        assert quality == RetrievalQuality.POOR
        assert "score too low" in reasoning.lower() or "no relevant" in reasoning.lower()

    def test_partial_quality_one_relevant(self):
        """Test Partial quality with only one relevant chunk"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.8, is_relevant=True, summary="S1"
            ),
            RelevanceAssessment(
                chunk_id="ch2", relevance_score=0.2, is_relevant=False, summary="S2"
            )
        ]
        answer_present = True

        quality, reasoning = assess_retrieval_quality(assessments, answer_present)

        assert quality == RetrievalQuality.PARTIAL
        assert "only 1 relevant chunk" in reasoning.lower()

    def test_partial_quality_no_answer(self):
        """Test Partial quality when answer not present"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.7, is_relevant=True, summary="S1"
            ),
            RelevanceAssessment(
                chunk_id="ch2", relevance_score=0.65, is_relevant=True, summary="S2"
            )
        ]
        answer_present = False  # Answer not present despite relevant chunks

        quality, reasoning = assess_retrieval_quality(assessments, answer_present)

        assert quality == RetrievalQuality.PARTIAL
        assert "answer not fully present" in reasoning.lower()

    def test_partial_quality_moderate_scores(self):
        """Test Partial quality with moderate relevance scores"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.5, is_relevant=True, summary="S1"
            ),
            RelevanceAssessment(
                chunk_id="ch2", relevance_score=0.55, is_relevant=True, summary="S2"
            )
        ]
        answer_present = True

        quality, reasoning = assess_retrieval_quality(assessments, answer_present)

        assert quality == RetrievalQuality.PARTIAL
        assert "moderate relevance scores" in reasoning.lower()


class TestAssessQualityWithThresholds:
    """Tests for quality assessment with custom thresholds"""

    def test_custom_good_threshold(self):
        """Test with custom threshold for Good rating"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.75, is_relevant=True, summary="S1"
            ),
            RelevanceAssessment(
                chunk_id="ch2", relevance_score=0.72, is_relevant=True, summary="S2"
            )
        ]
        answer_present = True

        # With default threshold (0.6), should be Good
        quality_default, _ = assess_quality_with_thresholds(
            assessments, answer_present, good_threshold=0.6
        )
        assert quality_default == RetrievalQuality.GOOD

        # With higher threshold (0.8), should be Partial
        quality_strict, _ = assess_quality_with_thresholds(
            assessments, answer_present, good_threshold=0.8
        )
        assert quality_strict == RetrievalQuality.PARTIAL

    def test_custom_min_relevant(self):
        """Test with custom minimum relevant chunks requirement"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.9, is_relevant=True, summary="S1"
            ),
            RelevanceAssessment(
                chunk_id="ch2", relevance_score=0.85, is_relevant=True, summary="S2"
            )
        ]
        answer_present = True

        # With min_relevant=2, should be Good
        quality_2, _ = assess_quality_with_thresholds(
            assessments, answer_present, min_relevant_for_good=2
        )
        assert quality_2 == RetrievalQuality.GOOD

        # With min_relevant=3, should be Partial
        quality_3, _ = assess_quality_with_thresholds(
            assessments, answer_present, min_relevant_for_good=3
        )
        assert quality_3 == RetrievalQuality.PARTIAL

    def test_custom_poor_threshold(self):
        """Test with custom threshold for Poor rating"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.35, is_relevant=True, summary="S1"
            )
        ]
        answer_present = False

        # With poor_threshold=0.3, should be Partial (0.35 >= 0.3)
        quality_default, _ = assess_quality_with_thresholds(
            assessments, answer_present, poor_threshold=0.3
        )
        assert quality_default == RetrievalQuality.PARTIAL

        # With poor_threshold=0.4, should be Poor (0.35 < 0.4)
        quality_strict, _ = assess_quality_with_thresholds(
            assessments, answer_present, poor_threshold=0.4
        )
        assert quality_strict == RetrievalQuality.POOR


class TestGetQualityRecommendations:
    """Tests for quality recommendations"""

    def test_recommendations_for_poor(self):
        """Test recommendations for Poor quality"""
        recommendations = get_quality_recommendations(
            RetrievalQuality.POOR, relevant_count=0, avg_score=0.15
        )

        assert len(recommendations) > 0
        assert any("reformulation" in r.lower() for r in recommendations)

    def test_recommendations_for_partial(self):
        """Test recommendations for Partial quality"""
        recommendations = get_quality_recommendations(
            RetrievalQuality.PARTIAL, relevant_count=1, avg_score=0.45
        )

        assert len(recommendations) > 0
        assert any("incomplete" in r.lower() or "refine" in r.lower() for r in recommendations)

    def test_recommendations_for_good(self):
        """Test recommendations for Good quality"""
        recommendations = get_quality_recommendations(
            RetrievalQuality.GOOD, relevant_count=3, avg_score=0.85
        )

        assert len(recommendations) > 0
        assert any("good" in r.lower() and "proceed" in r.lower() for r in recommendations)

    def test_recommendations_contextual(self):
        """Test that recommendations are contextual to the issue"""
        # Low relevant count
        recs_low_count = get_quality_recommendations(
            RetrievalQuality.PARTIAL, relevant_count=1, avg_score=0.5
        )
        assert any("increase" in r.lower() or "top-k" in r.lower() for r in recs_low_count)

        # Low scores
        recs_low_score = get_quality_recommendations(
            RetrievalQuality.PARTIAL, relevant_count=2, avg_score=0.35
        )
        assert any("score" in r.lower() or "expansion" in r.lower() for r in recs_low_score)


class TestCalculateQualityMetrics:
    """Tests for quality metrics calculation"""

    def test_metrics_with_mixed_assessments(self):
        """Test metrics calculation with mixed relevant/irrelevant"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.9, is_relevant=True, summary="S1"
            ),
            RelevanceAssessment(
                chunk_id="ch2", relevance_score=0.8, is_relevant=True, summary="S2"
            ),
            RelevanceAssessment(
                chunk_id="ch3", relevance_score=0.2, is_relevant=False, summary="S3"
            ),
            RelevanceAssessment(
                chunk_id="ch4", relevance_score=0.1, is_relevant=False, summary="S4"
            )
        ]
        answer_present = True

        metrics = calculate_quality_metrics(assessments, answer_present)

        assert metrics["total_chunks"] == 4
        assert metrics["relevant_count"] == 2
        assert metrics["irrelevant_count"] == 2
        assert metrics["precision"] == 0.5  # 2/4
        assert metrics["answer_present"] is True
        assert 0 <= metrics["avg_relevance_score"] <= 1
        assert 0 <= metrics["avg_relevant_score"] <= 1

    def test_metrics_score_distribution(self):
        """Test score distribution in metrics"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.9, is_relevant=True, summary="S1"
            ),  # High
            RelevanceAssessment(
                chunk_id="ch2", relevance_score=0.5, is_relevant=True, summary="S2"
            ),  # Medium
            RelevanceAssessment(
                chunk_id="ch3", relevance_score=0.1, is_relevant=False, summary="S3"
            )   # Low
        ]

        metrics = calculate_quality_metrics(assessments, True)

        score_dist = metrics["score_distribution"]
        assert score_dist["high"] == 1
        assert score_dist["medium"] == 1
        assert score_dist["low"] == 1

    def test_metrics_all_relevant(self):
        """Test metrics when all chunks are relevant"""
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1", relevance_score=0.9, is_relevant=True, summary="S1"
            ),
            RelevanceAssessment(
                chunk_id="ch2", relevance_score=0.85, is_relevant=True, summary="S2"
            )
        ]

        metrics = calculate_quality_metrics(assessments, True)

        assert metrics["precision"] == 1.0  # 100% precision
        assert metrics["irrelevant_count"] == 0
        assert metrics["has_high_quality_chunks"] is True

    def test_metrics_empty_assessments(self):
        """Test metrics with no assessments"""
        metrics = calculate_quality_metrics([], False)

        assert metrics["total_chunks"] == 0
        assert metrics["relevant_count"] == 0
        assert metrics["precision"] == 0.0
        assert metrics["avg_relevance_score"] == 0.0
