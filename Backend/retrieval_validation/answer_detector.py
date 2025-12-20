"""
Answer presence detection logic

This module determines whether retrieved chunks actually contain the answer
to the user's query. It uses rule-based entailment checking and entity matching.
"""

import re
from typing import Optional, Tuple

from .models import RelevanceAssessment
from .logger import logger, log_answer_detection


# Query type patterns
QUERY_PATTERNS = {
    'who': r'\b(who|whom|whose)\b',
    'what': r'\b(what|which)\b',
    'when': r'\b(when|what year|what date|what time)\b',
    'where': r'\b(where|in what location)\b',
    'why': r'\b(why|what reason|how come)\b',
    'how': r'\b(how|in what way)\b'
}

# Expected answer type mappings
ANSWER_TYPES = {
    'who': ['person', 'organization', 'people', 'team', 'founder', 'creator', 'author'],
    'what': ['thing', 'concept', 'feature', 'definition', 'item', 'product', 'information'],
    'when': ['year', 'date', 'time', 'century', 'decade', 'month', 'day'],
    'where': ['location', 'place', 'city', 'country', 'region', 'area'],
    'why': ['reason', 'cause', 'purpose', 'because', 'due to', 'since'],
    'how': ['method', 'process', 'way', 'manner', 'steps', 'procedure', 'information']
}

# Entity indicators for answer type matching
ENTITY_INDICATORS = {
    'person': r'\b(Mr\.|Mrs\.|Ms\.|Dr\.|[A-Z][a-z]+ [A-Z][a-z]+)\b',
    'year': r'\b(19|20)\d{2}\b',
    'date': r'\b(Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Oct|Nov|Dec)[a-z]*\s+\d{1,2}|\d{1,2}/\d{1,2}/\d{2,4}\b',
    'location': r'\b(in|at|near|from)\s+[A-Z][a-z]+\b',
    'organization': r'\b(Inc\.|Corp\.|LLC|Ltd\.|Company|Corporation)\b',
    'reason': r'\b(because|due to|since|as a result|caused by)\b',
    'method': r'\b(by|through|using|via|with|step 1|step 2|first|second|then)\b',
    # Added for better "what is X" detection
    'thing': r'\b(is a|is an|refers to|means|defined as|consists of|includes|provides|enables|allows)\b',
    'concept': r'\b(is a|is an|refers to|means|defined as|describes|represents|involves)\b',
    'definition': r'\b(is a|is an|is the|refers to|means|defined as)\b',
    'feature': r'\b(provides|enables|allows|supports|offers|includes|has|contains)\b',
    'item': r'\b(is a|is an|consists of|contains|includes|has)\b',
    'product': r'\b(is a|is an|provides|offers|includes|enables|supports)\b',
    'information': r'\b(is|are|was|were|has|have|can|will|provides|includes)\b'
}


def detect_query_type(query: str) -> Optional[str]:
    """
    Detect the type of question being asked.

    Args:
        query: User's question

    Returns:
        Query type ('who', 'what', 'when', 'where', 'why', 'how') or None

    Example:
        >>> detect_query_type("What is ROS 2?")
        'what'
        >>> detect_query_type("When was it founded?")
        'when'
    """
    query_lower = query.lower()

    for query_type, pattern in QUERY_PATTERNS.items():
        if re.search(pattern, query_lower, re.IGNORECASE):
            logger.debug(f"Detected query type: {query_type}")
            return query_type

    logger.debug("Could not detect query type")
    return None


def extract_expected_answer_types(query_type: Optional[str]) -> list[str]:
    """
    Extract expected answer types based on query type.

    Args:
        query_type: Type of question ('who', 'what', etc.)

    Returns:
        List of expected entity types in the answer

    Example:
        >>> extract_expected_answer_types('when')
        ['year', 'date', 'time', 'century', 'decade', 'month', 'day']
    """
    if query_type is None:
        # Generic answer types for unknown query type
        return ['thing', 'concept', 'information']

    return ANSWER_TYPES.get(query_type, ['information'])


def match_entity_type(text: str, expected_types: list[str]) -> bool:
    """
    Check if text contains entities matching expected types.

    Args:
        text: Text to search for entities
        expected_types: List of expected entity types

    Returns:
        True if text contains matching entities, False otherwise

    Example:
        >>> match_entity_type("The company was founded in 1995", ['year'])
        True
        >>> match_entity_type("The company was founded recently", ['year'])
        False
    """
    for entity_type in expected_types:
        # Check if entity type has a pattern
        pattern = ENTITY_INDICATORS.get(entity_type)
        if pattern and re.search(pattern, text, re.IGNORECASE):
            logger.debug(f"Found entity type '{entity_type}' in text")
            return True

        # Simple keyword matching for types without regex patterns
        if entity_type.lower() in text.lower():
            logger.debug(f"Found keyword '{entity_type}' in text")
            return True

    return False


def check_entailment(query: str, chunk_text: str) -> bool:
    """
    Rule-based entailment check: does chunk text answer the query?

    This is a simplified entailment check using keyword overlap and
    entity matching. More sophisticated than simple keyword matching,
    but avoids LLM-based entailment (which could hallucinate).

    Args:
        query: User's question
        chunk_text: Text from a relevant chunk

    Returns:
        True if chunk likely contains answer, False otherwise

    Example:
        >>> check_entailment(
        ...     "What year was it founded?",
        ...     "The company was founded in 1995"
        ... )
        True
    """
    # Extract query keywords (exclude question words and common words)
    stop_words = {'what', 'when', 'where', 'who', 'why', 'how', 'is', 'are',
                  'was', 'were', 'the', 'a', 'an', 'do', 'does', 'did', 'can',
                  'could', 'would', 'should', 'may', 'might', 'will', 'shall',
                  'about', 'this', 'that', 'it', 'for', 'of', 'in', 'to', 'and',
                  'or', 'be', 'been', 'being', 'have', 'has', 'had', 'do', 'you',
                  'me', 'my', 'your', 'tell', 'explain', 'describe'}

    query_words = set(re.findall(r'\b\w+\b', query.lower()))
    query_keywords = query_words - stop_words

    # Check keyword overlap
    chunk_text_lower = chunk_text.lower()
    keyword_matches = sum(1 for kw in query_keywords if kw in chunk_text_lower)
    overlap_ratio = keyword_matches / len(query_keywords) if query_keywords else 0

    # Entailment heuristic: lowered threshold to 25% for better recall
    # Also pass if at least 2 keywords match (for short queries)
    return overlap_ratio > 0.25 or keyword_matches >= 2


def detect_answer_presence(
    query: str,
    relevant_assessments: list[RelevanceAssessment]
) -> Tuple[bool, str]:
    """
    Detect whether the answer is present in relevant chunks.

    Uses a two-stage approach:
    1. Detect query type and expected answer entities
    2. Check if relevant chunks contain those entities

    Args:
        query: User's question
        relevant_assessments: List of relevant chunk assessments

    Returns:
        Tuple of (answer_present: bool, reasoning: str)

    Example:
        >>> answer_present, reasoning = detect_answer_presence(
        ...     query="What year was the company founded?",
        ...     relevant_assessments=[assessment1, assessment2]
        ... )
        >>> print(answer_present)
        True
    """
    # Handle empty relevant chunks
    if not relevant_assessments:
        reasoning = "No relevant chunks found"
        log_answer_detection(False, 0)
        return False, reasoning

    # Stage 1: Detect query type and expected answer types
    query_type = detect_query_type(query)
    expected_types = extract_expected_answer_types(query_type)

    logger.debug(
        f"Answer detection - Query type: {query_type}, "
        f"Expected types: {expected_types}"
    )

    # Stage 2: Check chunks for answer entities
    chunks_with_answer = 0
    chunks_with_entailment = 0

    for assessment in relevant_assessments:
        # Get full chunk text from summary (in practice, we'd have the full chunk)
        # For now, we check the summary
        chunk_text = assessment.summary

        # Check for expected entity types
        has_entities = match_entity_type(chunk_text, expected_types)
        if has_entities:
            chunks_with_answer += 1

        # Check entailment
        has_entailment = check_entailment(query, chunk_text)
        if has_entailment:
            chunks_with_entailment += 1

    # Decision logic: answer present if either:
    # - At least one chunk has expected entities
    # - At least one chunk shows strong entailment
    answer_present = (chunks_with_answer > 0) or (chunks_with_entailment > 0)

    # Generate reasoning
    if answer_present:
        reasoning = (
            f"Answer likely present: {chunks_with_answer} chunk(s) contain "
            f"expected entities, {chunks_with_entailment} show strong entailment"
        )
    else:
        reasoning = (
            "Answer not present: relevant chunks lack expected entities "
            "and show insufficient entailment"
        )

    log_answer_detection(answer_present, len(relevant_assessments))

    return answer_present, reasoning


def detect_answer_presence_with_full_chunks(
    query: str,
    relevant_assessments: list[RelevanceAssessment],
    chunk_texts: dict[str, str]
) -> Tuple[bool, str]:
    """
    Detect answer presence using full chunk texts (not just summaries).

    This is the preferred method when full chunk texts are available,
    as it provides more accurate entity and entailment matching.

    Args:
        query: User's question
        relevant_assessments: List of relevant chunk assessments
        chunk_texts: Mapping of chunk_id -> full chunk text

    Returns:
        Tuple of (answer_present: bool, reasoning: str)

    Example:
        >>> chunk_texts = {"ch1": "Full text of chunk 1...", "ch2": "..."}
        >>> answer_present, reasoning = detect_answer_presence_with_full_chunks(
        ...     query="What is ROS 2?",
        ...     relevant_assessments=[assessment1],
        ...     chunk_texts=chunk_texts
        ... )
    """
    if not relevant_assessments:
        reasoning = "No relevant chunks found"
        log_answer_detection(False, 0)
        return False, reasoning

    # Detect query type
    query_type = detect_query_type(query)
    expected_types = extract_expected_answer_types(query_type)

    logger.debug(
        f"Answer detection (full chunks) - Query type: {query_type}, "
        f"Expected types: {expected_types}"
    )

    # Check chunks using full text
    chunks_with_answer = 0
    chunks_with_entailment = 0

    for assessment in relevant_assessments:
        # Get full chunk text
        chunk_text = chunk_texts.get(assessment.chunk_id, assessment.summary)

        # Check for expected entity types
        has_entities = match_entity_type(chunk_text, expected_types)
        if has_entities:
            chunks_with_answer += 1

        # Check entailment
        has_entailment = check_entailment(query, chunk_text)
        if has_entailment:
            chunks_with_entailment += 1

    # Decision logic
    answer_present = (chunks_with_answer > 0) or (chunks_with_entailment > 0)

    # Generate reasoning
    if answer_present:
        reasoning = (
            f"Answer likely present: {chunks_with_answer} chunk(s) contain "
            f"expected entities, {chunks_with_entailment} show strong entailment"
        )
    else:
        reasoning = (
            "Answer not present: relevant chunks lack expected entities "
            "and show insufficient entailment"
        )

    log_answer_detection(answer_present, len(relevant_assessments))

    return answer_present, reasoning
