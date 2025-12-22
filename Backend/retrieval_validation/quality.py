"""
Quality assessment module

Provides overall quality rating (Good/Partial/Poor) of retrieval results
for system observability and continuous improvement.
"""

from typing import Tuple

from .models import RelevanceAssessment, RetrievalQuality
from .logger import logger, log_quality_rating


def calculate_average_relevance_score(
    assessments: list[RelevanceAssessment]
) -> float:
    """
    Calculate average relevance score across assessments.

    Args:
        assessments: List of relevance assessments

    Returns:
        Average relevance score (0.0-1.0), or 0.0 if empty

    Example:
        >>> assessments = [assessment1, assessment2, assessment3]
        >>> avg = calculate_average_relevance_score(assessments)
        >>> print(f"Average: {avg:.2f}")
    """
    if not assessments:
        return 0.0

    total_score = sum(a.relevance_score for a in assessments)
    return total_score / len(assessments)


def count_relevant_chunks(assessments: list[RelevanceAssessment]) -> int:
    """
    Count number of relevant chunks.

    Args:
        assessments: List of relevance assessments

    Returns:
        Count of chunks where is_relevant=True

    Example:
        >>> count = count_relevant_chunks(assessments)
        >>> print(f"{count} relevant chunks")
    """
    return sum(1 for a in assessments if a.is_relevant)


def assess_retrieval_quality(
    all_assessments: list[RelevanceAssessment],
    answer_present: bool
) -> Tuple[RetrievalQuality, str]:
    """
    Assess overall retrieval quality based on relevance and answer presence.

    Quality rating logic:
    - **Good**: >=2 relevant chunks AND answer_present=True AND avg_score >= 0.6
    - **Partial**: 1 relevant chunk OR answer_present=False (but >0 relevant) OR avg_score 0.3-0.6
    - **Poor**: 0 relevant chunks OR avg_score < 0.3

    Args:
        all_assessments: List of all relevance assessments (relevant + irrelevant)
        answer_present: Whether answer was detected in chunks

    Returns:
        Tuple of (quality_rating, reasoning_text)

    Example:
        >>> quality, reasoning = assess_retrieval_quality(assessments, answer_present=True)
        >>> print(f"Quality: {quality.value}")
        >>> print(f"Reasoning: {reasoning}")
    """
    # Count relevant chunks
    relevant_count = count_relevant_chunks(all_assessments)

    # Calculate average relevance score across ALL assessments
    avg_score = calculate_average_relevance_score(all_assessments)

    # Calculate average score across only RELEVANT chunks
    relevant_assessments = [a for a in all_assessments if a.is_relevant]
    avg_relevant_score = calculate_average_relevance_score(relevant_assessments)

    # Decision logic
    quality: RetrievalQuality
    reasoning: str

    # Poor quality conditions
    if relevant_count == 0:
        quality = RetrievalQuality.POOR
        reasoning = "No relevant chunks found"

    elif avg_score < 0.3:
        quality = RetrievalQuality.POOR
        reasoning = (
            f"Average relevance score too low ({avg_score:.2f}), "
            f"only {relevant_count} relevant chunk(s)"
        )

    # Good quality conditions
    elif relevant_count >= 2 and answer_present and avg_relevant_score >= 0.6:
        quality = RetrievalQuality.GOOD
        reasoning = (
            f"{relevant_count} relevant chunks found (avg score: {avg_relevant_score:.2f}), "
            f"answer fully present"
        )

    # Partial quality (everything else)
    else:
        quality = RetrievalQuality.PARTIAL

        # Build reasoning based on what's missing/weak
        reasons = []

        if relevant_count < 2:
            reasons.append(f"only {relevant_count} relevant chunk(s)")

        if not answer_present:
            reasons.append("answer not fully present")

        if avg_relevant_score < 0.6:
            reasons.append(f"moderate relevance scores (avg: {avg_relevant_score:.2f})")

        if not reasons:
            # Edge case: meets some but not all Good criteria
            reasons.append("partially meets quality criteria")

        reasoning = "Partial quality: " + ", ".join(reasons)

    # Log the rating
    log_quality_rating(quality.value, avg_score, relevant_count)

    return quality, reasoning

def assess_quality_with_thresholds(
    all_assessments: list[RelevanceAssessment],
    answer_present: bool,
    good_threshold: float = 0.6,
    poor_threshold: float = 0.3,
    min_relevant_for_good: int = 2
) -> Tuple[RetrievalQuality, str]:
    """
    Assess retrieval quality with custom thresholds.

    Allows tuning of quality rating criteria for specific use cases.

    Args:
        all_assessments: List of all relevance assessments
        answer_present: Whether answer was detected
        good_threshold: Minimum avg relevance score for "Good" rating
        poor_threshold: Maximum avg relevance score for "Poor" rating
        min_relevant_for_good: Minimum relevant chunks for "Good" rating

    Returns:
        Tuple of (quality_rating, reasoning_text)

    Example:
        >>> # Stricter quality standards
        >>> quality, reasoning = assess_quality_with_thresholds(
        ...     assessments,
        ...     answer_present=True,
        ...     good_threshold=0.7,
        ...     min_relevant_for_good=3
        ... )
    """
    relevant_count = count_relevant_chunks(all_assessments)
    avg_score = calculate_average_relevance_score(all_assessments)
    relevant_assessments = [a for a in all_assessments if a.is_relevant]
    avg_relevant_score = calculate_average_relevance_score(relevant_assessments)

    # Decision logic with custom thresholds
    quality: RetrievalQuality
    reasoning: str

    # Poor quality
    if relevant_count == 0:
        quality = RetrievalQuality.POOR
        reasoning = "No relevant chunks found"

    elif avg_score < poor_threshold:
        quality = RetrievalQuality.POOR
        reasoning = (
            f"Average relevance score too low ({avg_score:.2f} < {poor_threshold}), "
            f"only {relevant_count} relevant chunk(s)"
        )

    # Good quality
    elif (relevant_count >= min_relevant_for_good and
          answer_present and
          avg_relevant_score >= good_threshold):
        quality = RetrievalQuality.GOOD
        reasoning = (
            f"{relevant_count} relevant chunks found (avg score: {avg_relevant_score:.2f}), "
            f"answer fully present"
        )

    # Partial quality
    else:
        quality = RetrievalQuality.PARTIAL
        reasons = []

        if relevant_count < min_relevant_for_good:
            reasons.append(f"only {relevant_count} relevant chunk(s) (need {min_relevant_for_good})")

        if not answer_present:
            reasons.append("answer not fully present")

        if avg_relevant_score < good_threshold:
            reasons.append(
                f"relevance scores below threshold "
                f"(avg: {avg_relevant_score:.2f} < {good_threshold})"
            )

        if not reasons:
            reasons.append("partially meets quality criteria")

        reasoning = "Partial quality: " + ", ".join(reasons)

    log_quality_rating(quality.value, avg_score, relevant_count)

    return quality, reasoning


def get_quality_recommendations(
    quality: RetrievalQuality,
    relevant_count: int,
    avg_score: float
) -> list[str]:
    """
    Get actionable recommendations based on quality rating.

    Provides suggestions for improving retrieval quality.

    Args:
        quality: Current quality rating
        relevant_count: Number of relevant chunks
        avg_score: Average relevance score

    Returns:
        List of recommendation strings

    Example:
        >>> recommendations = get_quality_recommendations(
        ...     RetrievalQuality.PARTIAL,
        ...     relevant_count=1,
        ...     avg_score=0.45
        ... )
        >>> for rec in recommendations:
        ...     print(f"- {rec}")
    """
    recommendations = []

    if quality == RetrievalQuality.POOR:
        recommendations.append("Query reformulation recommended - try more specific keywords")
        recommendations.append("Consider expanding the search corpus")

        if relevant_count == 0:
            recommendations.append("No relevant results found - check if data exists for this topic")

        if avg_score < 0.2:
            recommendations.append("Very low similarity scores - query may be out of domain")

    elif quality == RetrievalQuality.PARTIAL:
        if relevant_count < 2:
            recommendations.append("Increase number of retrieved chunks (current top-k may be too small)")

        if avg_score < 0.5:
            recommendations.append("Moderate relevance scores - consider query expansion")

        recommendations.append("Answer may be incomplete - user may need to refine question")

    else:  # GOOD
        recommendations.append("Quality is good - proceed with answer generation")

    return recommendations


def calculate_quality_metrics(
    all_assessments: list[RelevanceAssessment],
    answer_present: bool
) -> dict:
    """
    Calculate detailed quality metrics for observability.

    Provides comprehensive metrics for monitoring and analytics.

    Args:
        all_assessments: List of all relevance assessments
        answer_present: Whether answer was detected

    Returns:
        Dictionary of quality metrics

    Example:
        >>> metrics = calculate_quality_metrics(assessments, True)
        >>> print(f"Precision: {metrics['precision']:.2%}")
        >>> print(f"Coverage: {metrics['coverage']:.2%}")
    """
    total_chunks = len(all_assessments)
    relevant_count = count_relevant_chunks(all_assessments)
    avg_score = calculate_average_relevance_score(all_assessments)

    relevant_assessments = [a for a in all_assessments if a.is_relevant]
    avg_relevant_score = calculate_average_relevance_score(relevant_assessments)

    # Calculate metrics
    precision = relevant_count / total_chunks if total_chunks > 0 else 0.0

    # Score distribution
    score_ranges = {
        "high": sum(1 for a in all_assessments if a.relevance_score >= 0.7),
        "medium": sum(1 for a in all_assessments if 0.3 <= a.relevance_score < 0.7),
        "low": sum(1 for a in all_assessments if a.relevance_score < 0.3)
    }

    return {
        "total_chunks": total_chunks,
        "relevant_count": relevant_count,
        "irrelevant_count": total_chunks - relevant_count,
        "precision": precision,
        "avg_relevance_score": avg_score,
        "avg_relevant_score": avg_relevant_score,
        "answer_present": answer_present,
        "score_distribution": score_ranges,
        "has_high_quality_chunks": score_ranges["high"] > 0,
        "has_low_quality_chunks": score_ranges["low"] > 0
    }
