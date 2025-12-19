"""
Evidence extraction module

Extracts and quotes exact lines from chunks that support the answer,
with attribution and context for traceability.
"""

from typing import Optional
import nltk
from nltk.tokenize import sent_tokenize

from .models import EvidenceQuote, RelevanceAssessment
from .logger import logger, log_evidence_extraction


def ensure_nltk_data():
    """
    Ensure NLTK punkt tokenizer data is available.

    Downloads if not already present. Called automatically by extract_evidence.
    """
    try:
        nltk.data.find('tokenizers/punkt')
    except LookupError:
        logger.info("Downloading NLTK punkt tokenizer data")
        nltk.download('punkt', quiet=True)


def tokenize_sentences(text: str) -> list[str]:
    """
    Tokenize text into sentences using NLTK.

    Args:
        text: Text to tokenize

    Returns:
        List of sentences

    Example:
        >>> sentences = tokenize_sentences("Hello. How are you? I'm fine.")
        >>> len(sentences)
        3
    """
    ensure_nltk_data()

    try:
        sentences = sent_tokenize(text)
        return sentences
    except Exception as e:
        logger.warning(f"Sentence tokenization failed: {e}. Falling back to simple split.")
        # Fallback: split on common sentence terminators
        import re
        sentences = re.split(r'[.!?]+', text)
        return [s.strip() for s in sentences if s.strip()]


def find_relevant_sentences(
    query: str,
    sentences: list[str],
    min_overlap: float = 0.3
) -> list[int]:
    """
    Find sentence indices that are relevant to the query.

    Uses keyword overlap to identify sentences likely containing the answer.

    Args:
        query: User's question
        sentences: List of sentences from chunk
        min_overlap: Minimum keyword overlap ratio (0.0-1.0)

    Returns:
        List of sentence indices (0-indexed)

    Example:
        >>> sentences = ["ROS 2 is great.", "It has features.", "Like security."]
        >>> indices = find_relevant_sentences("What features?", sentences)
        >>> indices
        [1, 2]
    """
    import re

    # Extract query keywords (exclude common stop words)
    stop_words = {
        'what', 'when', 'where', 'who', 'why', 'how', 'is', 'are',
        'was', 'were', 'the', 'a', 'an', 'do', 'does', 'did', 'can',
        'could', 'would', 'should', 'may', 'might', 'will', 'shall',
        'in', 'on', 'at', 'to', 'for', 'of', 'with', 'by', 'from'
    }

    query_words = set(re.findall(r'\b\w+\b', query.lower()))
    query_keywords = query_words - stop_words

    if not query_keywords:
        # If no keywords after filtering, return first sentence
        return [0] if sentences else []

    relevant_indices = []

    for i, sentence in enumerate(sentences):
        sentence_lower = sentence.lower()

        # Count keyword matches
        matches = sum(1 for kw in query_keywords if kw in sentence_lower)
        overlap_ratio = matches / len(query_keywords)

        if overlap_ratio >= min_overlap:
            relevant_indices.append(i)

    # If no sentences meet threshold, return sentence with most matches
    if not relevant_indices and sentences:
        best_idx = 0
        best_matches = 0

        for i, sentence in enumerate(sentences):
            sentence_lower = sentence.lower()
            matches = sum(1 for kw in query_keywords if kw in sentence_lower)
            if matches > best_matches:
                best_matches = matches
                best_idx = i

        relevant_indices = [best_idx]

    return relevant_indices


def extract_evidence_from_chunk(
    query: str,
    chunk_id: str,
    chunk_text: str,
    include_context: bool = True
) -> list[EvidenceQuote]:
    """
    Extract evidence quotes from a single chunk.

    Args:
        query: User's question
        chunk_id: ID of the chunk being processed
        chunk_text: Full text of the chunk
        include_context: Whether to include surrounding sentences as context

    Returns:
        List of evidence quotes with attribution

    Example:
        >>> quotes = extract_evidence_from_chunk(
        ...     query="What is ROS 2?",
        ...     chunk_id="ch1",
        ...     chunk_text="ROS 2 is a robot framework. It has many features."
        ... )
        >>> len(quotes)
        1
    """
    # Tokenize into sentences
    sentences = tokenize_sentences(chunk_text)

    if not sentences:
        logger.warning(f"No sentences found in chunk {chunk_id}")
        return []

    # Find relevant sentences
    relevant_indices = find_relevant_sentences(query, sentences)

    # Create evidence quotes
    quotes = []

    for sentence_idx in relevant_indices:
        quote_text = sentences[sentence_idx]

        # Extract context if requested
        context_before = None
        context_after = None

        if include_context:
            if sentence_idx > 0:
                context_before = sentences[sentence_idx - 1]
            if sentence_idx < len(sentences) - 1:
                context_after = sentences[sentence_idx + 1]

        # Validate quote is verbatim (exists in original text)
        if quote_text not in chunk_text:
            logger.warning(
                f"Quote not found verbatim in chunk {chunk_id}, "
                f"sentence index {sentence_idx}"
            )
            continue

        evidence = EvidenceQuote(
            chunk_id=chunk_id,
            quote=quote_text,
            sentence_index=sentence_idx,
            context_before=context_before,
            context_after=context_after
        )
        quotes.append(evidence)

    log_evidence_extraction(chunk_id, len(quotes))

    return quotes


def extract_evidence(
    query: str,
    relevant_assessments: list[RelevanceAssessment],
    chunk_texts: dict[str, str],
    max_quotes_per_chunk: int = 3,
    include_context: bool = True
) -> list[EvidenceQuote]:
    """
    Extract evidence from all relevant chunks.

    This is the main evidence extraction function that processes all
    relevant chunks and aggregates evidence quotes.

    Args:
        query: User's question
        relevant_assessments: List of relevant chunk assessments
        chunk_texts: Mapping of chunk_id -> full chunk text
        max_quotes_per_chunk: Maximum number of quotes to extract per chunk
        include_context: Whether to include surrounding sentences as context

    Returns:
        List of all evidence quotes across relevant chunks

    Example:
        >>> evidence = extract_evidence(
        ...     query="What is ROS 2?",
        ...     relevant_assessments=[assessment1, assessment2],
        ...     chunk_texts={"ch1": "Full text...", "ch2": "More text..."}
        ... )
        >>> len(evidence)
        5
    """
    if not relevant_assessments:
        logger.debug("No relevant chunks for evidence extraction")
        return []

    all_evidence = []

    for assessment in relevant_assessments:
        chunk_id = assessment.chunk_id
        chunk_text = chunk_texts.get(chunk_id)

        if not chunk_text:
            logger.warning(
                f"Chunk text not found for {chunk_id}, "
                f"using summary as fallback"
            )
            chunk_text = assessment.summary

        # Extract evidence from this chunk
        chunk_evidence = extract_evidence_from_chunk(
            query=query,
            chunk_id=chunk_id,
            chunk_text=chunk_text,
            include_context=include_context
        )

        # Limit quotes per chunk
        chunk_evidence = chunk_evidence[:max_quotes_per_chunk]

        all_evidence.extend(chunk_evidence)

    logger.debug(
        f"Evidence extraction complete: {len(all_evidence)} quotes "
        f"from {len(relevant_assessments)} chunks"
    )

    return all_evidence


def validate_evidence_verbatim(
    evidence: EvidenceQuote,
    chunk_text: str
) -> bool:
    """
    Validate that evidence quote exists verbatim in chunk text.

    This ensures no paraphrasing or hallucination has occurred.

    Args:
        evidence: Evidence quote to validate
        chunk_text: Original chunk text

    Returns:
        True if quote exists verbatim, False otherwise

    Example:
        >>> evidence = EvidenceQuote(
        ...     chunk_id="ch1",
        ...     quote="ROS 2 is great.",
        ...     sentence_index=0
        ... )
        >>> validate_evidence_verbatim(evidence, "ROS 2 is great. Very nice.")
        True
    """
    return evidence.quote in chunk_text


def filter_redundant_evidence(
    evidence_list: list[EvidenceQuote],
    similarity_threshold: float = 0.8
) -> list[EvidenceQuote]:
    """
    Filter out redundant evidence quotes.

    Removes quotes that are too similar to each other to avoid repetition.

    Args:
        evidence_list: List of evidence quotes
        similarity_threshold: Threshold for considering quotes redundant (0.0-1.0)

    Returns:
        Filtered list with redundant quotes removed

    Note:
        Uses simple word-level Jaccard similarity. Could be enhanced with
        embedding-based similarity for better redundancy detection.
    """
    if len(evidence_list) <= 1:
        return evidence_list

    def jaccard_similarity(text1: str, text2: str) -> float:
        """Calculate Jaccard similarity between two texts"""
        words1 = set(text1.lower().split())
        words2 = set(text2.lower().split())

        if not words1 or not words2:
            return 0.0

        intersection = words1.intersection(words2)
        union = words1.union(words2)

        return len(intersection) / len(union)

    filtered = []

    for evidence in evidence_list:
        # Check if similar to any already-filtered evidence
        is_redundant = False

        for existing in filtered:
            similarity = jaccard_similarity(evidence.quote, existing.quote)
            if similarity >= similarity_threshold:
                is_redundant = True
                break

        if not is_redundant:
            filtered.append(evidence)

    if len(filtered) < len(evidence_list):
        logger.debug(
            f"Filtered {len(evidence_list) - len(filtered)} redundant quotes"
        )

    return filtered


def rank_evidence_by_relevance(
    evidence_list: list[EvidenceQuote],
    query: str
) -> list[EvidenceQuote]:
    """
    Rank evidence quotes by relevance to query.

    Uses keyword overlap to score each quote's relevance.

    Args:
        evidence_list: List of evidence quotes
        query: User's question

    Returns:
        Evidence list sorted by relevance (most relevant first)
    """
    import re

    # Extract query keywords
    query_words = set(re.findall(r'\b\w+\b', query.lower()))

    def relevance_score(evidence: EvidenceQuote) -> float:
        """Calculate relevance score for a quote"""
        quote_words = set(re.findall(r'\b\w+\b', evidence.quote.lower()))

        if not query_words or not quote_words:
            return 0.0

        # Jaccard similarity
        intersection = query_words.intersection(quote_words)
        union = query_words.union(quote_words)

        return len(intersection) / len(union)

    # Sort by relevance score (highest first)
    ranked = sorted(evidence_list, key=relevance_score, reverse=True)

    return ranked
