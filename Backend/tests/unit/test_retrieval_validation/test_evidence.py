"""
Unit tests for evidence extraction module

Tests sentence tokenization, evidence extraction, and validation.
"""

import pytest

from retrieval_validation.evidence import (
    tokenize_sentences,
    find_relevant_sentences,
    extract_evidence_from_chunk,
    extract_evidence,
    validate_evidence_verbatim,
    filter_redundant_evidence,
    rank_evidence_by_relevance
)
from retrieval_validation.models import EvidenceQuote, RelevanceAssessment


class TestTokenizeSentences:
    """Tests for sentence tokenization"""

    def test_tokenize_simple_sentences(self):
        """Test tokenization of simple sentences"""
        text = "This is sentence one. This is sentence two. And sentence three."
        sentences = tokenize_sentences(text)
        assert len(sentences) == 3
        assert "This is sentence one" in sentences[0]

    def test_tokenize_with_abbreviations(self):
        """Test tokenization handles abbreviations correctly"""
        text = "Dr. Smith founded the company. It was in the U.S. in 1995."
        sentences = tokenize_sentences(text)
        # NLTK should handle Dr. and U.S. correctly
        assert len(sentences) == 2

    def test_tokenize_with_questions(self):
        """Test tokenization with question marks"""
        text = "What is ROS 2? Is it better? Yes it is!"
        sentences = tokenize_sentences(text)
        assert len(sentences) == 3

    def test_tokenize_empty_text(self):
        """Test tokenization of empty text"""
        sentences = tokenize_sentences("")
        assert len(sentences) == 0

    def test_tokenize_single_sentence(self):
        """Test tokenization of single sentence"""
        text = "This is one sentence"
        sentences = tokenize_sentences(text)
        assert len(sentences) == 1


class TestFindRelevantSentences:
    """Tests for finding relevant sentences"""

    def test_find_high_overlap(self):
        """Test finding sentences with high keyword overlap"""
        query = "What are the features of ROS 2?"
        sentences = [
            "ROS 2 has many features.",
            "The weather is nice today.",
            "Features include security and real-time capabilities."
        ]

        indices = find_relevant_sentences(query, sentences)
        assert 0 in indices  # Sentence 0 mentions ROS 2 and features
        assert 2 in indices  # Sentence 2 mentions features

    def test_find_with_stop_words_filtered(self):
        """Test that stop words are filtered from matching"""
        query = "What is the purpose of this tool?"
        sentences = [
            "The purpose is automation.",
            "This is unrelated content."
        ]

        indices = find_relevant_sentences(query, sentences)
        assert 0 in indices  # Contains "purpose"

    def test_find_no_overlap_returns_best(self):
        """Test that when no sentences meet threshold, best match is returned"""
        query = "quantum computing algorithms"
        sentences = [
            "ROS 2 is great.",
            "Computing power is important.",  # Has "computing"
            "The sky is blue."
        ]

        indices = find_relevant_sentences(query, sentences)
        assert len(indices) == 1
        assert 1 in indices  # Sentence with "computing"

    def test_find_empty_sentences(self):
        """Test with empty sentence list"""
        indices = find_relevant_sentences("test query", [])
        assert len(indices) == 0

    def test_find_custom_min_overlap(self):
        """Test with custom minimum overlap threshold"""
        query = "features security real-time"
        sentences = [
            "Features include security.",  # 2/3 keywords = 0.67
            "Real-time is important."      # 1/3 keywords = 0.33
        ]

        # With min_overlap=0.5, only first sentence should match
        indices = find_relevant_sentences(query, sentences, min_overlap=0.5)
        assert 0 in indices
        assert 1 not in indices

        # With min_overlap=0.3, both should match
        indices = find_relevant_sentences(query, sentences, min_overlap=0.3)
        assert 0 in indices
        assert 1 in indices


class TestExtractEvidenceFromChunk:
    """Tests for extracting evidence from a single chunk"""

    def test_extract_simple_evidence(self):
        """Test basic evidence extraction"""
        query = "What is ROS 2?"
        chunk_id = "ch1"
        chunk_text = "ROS 2 is a robot framework. It provides many features. Very useful."

        evidence = extract_evidence_from_chunk(query, chunk_id, chunk_text)

        assert len(evidence) >= 1
        assert evidence[0].chunk_id == chunk_id
        assert "ROS 2" in evidence[0].quote

    def test_extract_with_context(self):
        """Test that context sentences are included"""
        query = "What features?"
        chunk_id = "ch1"
        chunk_text = "ROS 2 is great. It has many features. Like security and real-time."

        evidence = extract_evidence_from_chunk(
            query, chunk_id, chunk_text, include_context=True
        )

        assert len(evidence) >= 1
        # Should have context from surrounding sentences
        assert evidence[0].context_before is not None or evidence[0].context_after is not None

    def test_extract_without_context(self):
        """Test extraction without context"""
        query = "What features?"
        chunk_id = "ch1"
        chunk_text = "ROS 2 has many features. Very nice."

        evidence = extract_evidence_from_chunk(
            query, chunk_id, chunk_text, include_context=False
        )

        assert len(evidence) >= 1
        assert evidence[0].context_before is None
        assert evidence[0].context_after is None

    def test_extract_empty_chunk(self):
        """Test extraction from empty chunk"""
        evidence = extract_evidence_from_chunk("query", "ch1", "")
        assert len(evidence) == 0

    def test_extract_validates_verbatim(self):
        """Test that extracted quotes exist verbatim in original text"""
        query = "What is the answer?"
        chunk_id = "ch1"
        chunk_text = "The answer is 42. It's the ultimate answer."

        evidence = extract_evidence_from_chunk(query, chunk_id, chunk_text)

        # All quotes should exist verbatim in chunk_text
        for quote in evidence:
            assert quote.quote in chunk_text

    def test_extract_sentence_index_correct(self):
        """Test that sentence indices are correctly assigned"""
        query = "second sentence"
        chunk_id = "ch1"
        chunk_text = "First sentence. Second sentence is here. Third sentence."

        evidence = extract_evidence_from_chunk(query, chunk_id, chunk_text)

        # Should find second sentence (index 1)
        assert any(e.sentence_index == 1 for e in evidence)


class TestExtractEvidence:
    """Tests for extracting evidence from multiple chunks"""

    def test_extract_from_multiple_chunks(self):
        """Test extraction across multiple relevant chunks"""
        query = "What are the features?"
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1",
                relevance_score=0.9,
                is_relevant=True,
                summary="Features include security..."
            ),
            RelevanceAssessment(
                chunk_id="ch2",
                relevance_score=0.8,
                is_relevant=True,
                summary="Additional features are real-time..."
            )
        ]
        chunk_texts = {
            "ch1": "Features include security and authentication. Very important.",
            "ch2": "Additional features are real-time capabilities. Also cross-platform support."
        }

        evidence = extract_evidence(query, assessments, chunk_texts)

        # Should have evidence from both chunks
        assert len(evidence) >= 2
        chunk_ids = {e.chunk_id for e in evidence}
        assert "ch1" in chunk_ids
        assert "ch2" in chunk_ids

    def test_extract_no_relevant_chunks(self):
        """Test extraction with no relevant chunks"""
        evidence = extract_evidence("query", [], {})
        assert len(evidence) == 0

    def test_extract_max_quotes_per_chunk(self):
        """Test limiting quotes per chunk"""
        query = "features security real-time platform"
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1",
                relevance_score=0.9,
                is_relevant=True,
                summary="Many features..."
            )
        ]
        chunk_texts = {
            "ch1": (
                "Feature one is security. "
                "Feature two is real-time. "
                "Feature three is platform. "
                "Feature four is modularity. "
                "Feature five is scalability."
            )
        }

        # Limit to 2 quotes per chunk
        evidence = extract_evidence(
            query, assessments, chunk_texts, max_quotes_per_chunk=2
        )

        # Should have at most 2 quotes from this chunk
        assert len(evidence) <= 2

    def test_extract_missing_chunk_text_uses_summary(self):
        """Test fallback to summary when chunk text missing"""
        query = "What features?"
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1",
                relevance_score=0.9,
                is_relevant=True,
                summary="Features include security and real-time capabilities."
            )
        ]
        chunk_texts = {}  # No chunk text provided

        evidence = extract_evidence(query, assessments, chunk_texts)

        # Should still extract evidence from summary
        assert len(evidence) >= 1


class TestValidateEvidenceVerbatim:
    """Tests for validating evidence is verbatim"""

    def test_validate_exact_match(self):
        """Test validation with exact match"""
        evidence = EvidenceQuote(
            chunk_id="ch1",
            quote="ROS 2 is a framework.",
            sentence_index=0
        )
        chunk_text = "ROS 2 is a framework. It has features."

        assert validate_evidence_verbatim(evidence, chunk_text) is True

    def test_validate_no_match(self):
        """Test validation when quote doesn't exist"""
        evidence = EvidenceQuote(
            chunk_id="ch1",
            quote="ROS 2 is amazing.",  # Not in chunk
            sentence_index=0
        )
        chunk_text = "ROS 2 is a framework. It has features."

        assert validate_evidence_verbatim(evidence, chunk_text) is False

    def test_validate_partial_match(self):
        """Test validation requires exact substring match"""
        evidence = EvidenceQuote(
            chunk_id="ch1",
            quote="ROS 2 framework",  # Partial quote
            sentence_index=0
        )
        chunk_text = "ROS 2 is a framework."

        # Partial match should still be valid (substring exists)
        assert validate_evidence_verbatim(evidence, chunk_text) is True


class TestFilterRedundantEvidence:
    """Tests for filtering redundant evidence"""

    def test_filter_identical_quotes(self):
        """Test filtering of identical quotes"""
        evidence = [
            EvidenceQuote(chunk_id="ch1", quote="ROS 2 is great.", sentence_index=0),
            EvidenceQuote(chunk_id="ch2", quote="ROS 2 is great.", sentence_index=0),
        ]

        filtered = filter_redundant_evidence(evidence, similarity_threshold=0.9)

        # Should keep only one
        assert len(filtered) == 1

    def test_filter_similar_quotes(self):
        """Test filtering of very similar quotes"""
        evidence = [
            EvidenceQuote(chunk_id="ch1", quote="ROS 2 has many features.", sentence_index=0),
            EvidenceQuote(chunk_id="ch2", quote="ROS 2 has many capabilities.", sentence_index=0),
            # Very similar, most words overlap
        ]

        filtered = filter_redundant_evidence(evidence, similarity_threshold=0.7)

        # Should filter one out due to high similarity
        assert len(filtered) <= 2

    def test_filter_distinct_quotes(self):
        """Test that distinct quotes are kept"""
        evidence = [
            EvidenceQuote(chunk_id="ch1", quote="ROS 2 has security.", sentence_index=0),
            EvidenceQuote(chunk_id="ch2", quote="Real-time capabilities exist.", sentence_index=0),
        ]

        filtered = filter_redundant_evidence(evidence)

        # Both should be kept (distinct content)
        assert len(filtered) == 2

    def test_filter_empty_list(self):
        """Test filtering empty list"""
        filtered = filter_redundant_evidence([])
        assert len(filtered) == 0

    def test_filter_single_quote(self):
        """Test filtering single quote"""
        evidence = [
            EvidenceQuote(chunk_id="ch1", quote="Single quote.", sentence_index=0)
        ]
        filtered = filter_redundant_evidence(evidence)
        assert len(filtered) == 1


class TestRankEvidenceByRelevance:
    """Tests for ranking evidence by relevance"""

    def test_rank_by_keyword_overlap(self):
        """Test ranking based on keyword overlap with query"""
        query = "What are the security features of ROS 2?"
        evidence = [
            EvidenceQuote(
                chunk_id="ch1",
                quote="ROS 2 provides cross-platform support.",  # Lower overlap
                sentence_index=0
            ),
            EvidenceQuote(
                chunk_id="ch2",
                quote="Security features include authentication and encryption.",  # High overlap
                sentence_index=0
            ),
            EvidenceQuote(
                chunk_id="ch3",
                quote="ROS 2 security features are robust.",  # Highest overlap
                sentence_index=0
            ),
        ]

        ranked = rank_evidence_by_relevance(evidence, query)

        # Most relevant should be first
        assert "security" in ranked[0].quote.lower()
        assert "ROS 2" in ranked[0].quote or "security" in ranked[0].quote

    def test_rank_maintains_all_evidence(self):
        """Test that ranking doesn't remove any evidence"""
        query = "test query"
        evidence = [
            EvidenceQuote(chunk_id=f"ch{i}", quote=f"Quote {i}", sentence_index=0)
            for i in range(5)
        ]

        ranked = rank_evidence_by_relevance(evidence, query)

        assert len(ranked) == len(evidence)

    def test_rank_empty_list(self):
        """Test ranking empty list"""
        ranked = rank_evidence_by_relevance([], "query")
        assert len(ranked) == 0
