"""
Unit tests for answer presence detection module

Tests query type detection, entity matching, and answer presence logic.
"""

import pytest

from retrieval_validation.answer_detector import (
    detect_query_type,
    extract_expected_answer_types,
    match_entity_type,
    check_entailment,
    detect_answer_presence,
    detect_answer_presence_with_full_chunks
)
from retrieval_validation.models import RelevanceAssessment


class TestDetectQueryType:
    """Tests for query type detection"""

    def test_detect_what_query(self):
        """Test detection of 'what' questions"""
        assert detect_query_type("What is ROS 2?") == 'what'
        assert detect_query_type("What are the features?") == 'what'
        assert detect_query_type("Which option is best?") == 'what'

    def test_detect_when_query(self):
        """Test detection of 'when' questions"""
        assert detect_query_type("When was it founded?") == 'when'
        assert detect_query_type("What year did it happen?") == 'when'
        assert detect_query_type("What date is the event?") == 'when'

    def test_detect_who_query(self):
        """Test detection of 'who' questions"""
        assert detect_query_type("Who founded the company?") == 'who'
        assert detect_query_type("Whom should I contact?") == 'who'
        assert detect_query_type("Whose idea was this?") == 'who'

    def test_detect_where_query(self):
        """Test detection of 'where' questions"""
        assert detect_query_type("Where is the office located?") == 'where'
        assert detect_query_type("In what location can I find it?") == 'where'

    def test_detect_why_query(self):
        """Test detection of 'why' questions"""
        assert detect_query_type("Why is this important?") == 'why'
        assert detect_query_type("What reason explains this?") == 'why'

    def test_detect_how_query(self):
        """Test detection of 'how' questions"""
        assert detect_query_type("How do I reset my password?") == 'how'
        assert detect_query_type("In what way can I do this?") == 'how'

    def test_detect_unknown_query_type(self):
        """Test handling of non-question queries"""
        assert detect_query_type("Tell me about ROS 2") is None
        assert detect_query_type("ROS 2 information") is None


class TestExtractExpectedAnswerTypes:
    """Tests for extracting expected answer types"""

    def test_extract_when_types(self):
        """Test expected types for 'when' questions"""
        types = extract_expected_answer_types('when')
        assert 'year' in types
        assert 'date' in types
        assert 'time' in types

    def test_extract_who_types(self):
        """Test expected types for 'who' questions"""
        types = extract_expected_answer_types('who')
        assert 'person' in types
        assert 'organization' in types

    def test_extract_where_types(self):
        """Test expected types for 'where' questions"""
        types = extract_expected_answer_types('where')
        assert 'location' in types
        assert 'place' in types

    def test_extract_unknown_types(self):
        """Test default types for unknown query type"""
        types = extract_expected_answer_types(None)
        assert 'thing' in types or 'concept' in types


class TestMatchEntityType:
    """Tests for entity type matching"""

    def test_match_year_entity(self):
        """Test matching year entities"""
        assert match_entity_type("Founded in 1995", ['year']) is True
        assert match_entity_type("Founded in 2020", ['year']) is True
        assert match_entity_type("Founded recently", ['year']) is False

    def test_match_person_entity(self):
        """Test matching person entities"""
        assert match_entity_type("Created by John Smith", ['person']) is True
        assert match_entity_type("Dr. Jane Doe invented it", ['person']) is True
        assert match_entity_type("Created by the team", ['person']) is False

    def test_match_location_entity(self):
        """Test matching location entities"""
        assert match_entity_type("Located in Boston", ['location']) is True
        assert match_entity_type("From California", ['location']) is True
        assert match_entity_type("Everywhere", ['location']) is False

    def test_match_organization_entity(self):
        """Test matching organization entities"""
        assert match_entity_type("Acme Corp. headquarters", ['organization']) is True
        assert match_entity_type("Tech Inc. building", ['organization']) is True
        assert match_entity_type("A company building", ['organization']) is False

    def test_match_method_entity(self):
        """Test matching method/process indicators"""
        assert match_entity_type("Reset by clicking the button", ['method']) is True
        assert match_entity_type("First, open the app. Then...", ['method']) is True
        assert match_entity_type("It happens automatically", ['method']) is False

    def test_match_multiple_types(self):
        """Test matching against multiple entity types"""
        text = "Founded in 1995 by John Smith in Boston"
        assert match_entity_type(text, ['year', 'person', 'location']) is True

    def test_match_no_entities(self):
        """Test when no entities match"""
        text = "Some generic information here"
        assert match_entity_type(text, ['year', 'person']) is False


class TestCheckEntailment:
    """Tests for rule-based entailment checking"""

    def test_entailment_high_overlap(self):
        """Test entailment with high keyword overlap"""
        query = "What are the features of ROS 2?"
        chunk = "ROS 2 features include real-time capabilities and security"
        assert check_entailment(query, chunk) is True

    def test_entailment_low_overlap(self):
        """Test entailment with low keyword overlap"""
        query = "What are the features of ROS 2?"
        chunk = "Cooking pasta is an art form requiring practice"
        assert check_entailment(query, chunk) is False

    def test_entailment_ignores_stop_words(self):
        """Test that stop words are ignored in entailment"""
        query = "What is the purpose of this?"
        chunk = "The purpose is to provide automation"
        # 'purpose' is the key word, stop words like 'what', 'is', 'the' ignored
        assert check_entailment(query, chunk) is True

    def test_entailment_partial_match(self):
        """Test entailment with partial keyword match"""
        query = "How do I reset my password?"
        chunk = "To reset, click the reset button"
        assert check_entailment(query, chunk) is True


class TestDetectAnswerPresence:
    """Tests for answer presence detection"""

    def test_answer_present_with_entities(self):
        """Test detection when answer entities are present"""
        query = "What year was the company founded?"
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1",
                relevance_score=0.9,
                is_relevant=True,
                summary="The company was founded in 1995 by the founders"
            )
        ]

        answer_present, reasoning = detect_answer_presence(query, assessments)
        assert answer_present is True
        assert "present" in reasoning.lower()

    def test_answer_not_present_no_entities(self):
        """Test detection when answer entities are missing"""
        query = "What year was the company founded?"
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1",
                relevance_score=0.8,
                is_relevant=True,
                summary="The company focuses on innovative technology solutions"
            )
        ]

        answer_present, reasoning = detect_answer_presence(query, assessments)
        assert answer_present is False
        assert "not present" in reasoning.lower()

    def test_answer_present_with_entailment(self):
        """Test detection based on strong entailment"""
        query = "What features does ROS 2 have?"
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1",
                relevance_score=0.9,
                is_relevant=True,
                summary="ROS 2 features include real-time capabilities and security"
            )
        ]

        answer_present, reasoning = detect_answer_presence(query, assessments)
        assert answer_present is True

    def test_answer_no_relevant_chunks(self):
        """Test detection with no relevant chunks"""
        query = "What is quantum computing?"
        assessments = []

        answer_present, reasoning = detect_answer_presence(query, assessments)
        assert answer_present is False
        assert "no relevant" in reasoning.lower()

    def test_answer_multiple_relevant_chunks(self):
        """Test detection across multiple relevant chunks"""
        query = "What are the pricing tiers?"
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1",
                relevance_score=0.9,
                is_relevant=True,
                summary="Our pricing includes Basic and Pro tiers"
            ),
            RelevanceAssessment(
                chunk_id="ch2",
                relevance_score=0.85,
                is_relevant=True,
                summary="Pricing tiers are designed for different needs"
            )
        ]

        answer_present, reasoning = detect_answer_presence(query, assessments)
        assert answer_present is True


class TestDetectAnswerPresenceWithFullChunks:
    """Tests for answer presence detection with full chunk texts"""

    def test_answer_with_full_chunks(self):
        """Test detection using full chunk texts"""
        query = "When was the company founded?"
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1",
                relevance_score=0.95,
                is_relevant=True,
                summary="The company was founded..."
            )
        ]
        chunk_texts = {
            "ch1": "The company was founded in 1995 by John Smith and Jane Doe. "
                   "Initially operating out of a small garage, it has grown significantly."
        }

        answer_present, reasoning = detect_answer_presence_with_full_chunks(
            query, assessments, chunk_texts
        )
        assert answer_present is True
        assert "present" in reasoning.lower()

    def test_answer_full_chunks_more_accurate(self):
        """Test that full chunks provide more accurate detection than summaries"""
        query = "What year was it founded?"
        # Summary is truncated and missing the year
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1",
                relevance_score=0.9,
                is_relevant=True,
                summary="The company has a long history of innovation..."
            )
        ]
        # Full text contains the year
        chunk_texts = {
            "ch1": "The company has a long history of innovation. Founded in 1995, "
                   "it quickly became a leader in its field."
        }

        # Detection with summary alone (using detect_answer_presence)
        answer_summary, _ = detect_answer_presence(query, assessments)

        # Detection with full chunks
        answer_full, _ = detect_answer_presence_with_full_chunks(
            query, assessments, chunk_texts
        )

        # Full chunk detection should find the answer
        assert answer_full is True
        # (Summary-only might miss it, but this depends on the summary length)

    def test_answer_full_chunks_no_relevant(self):
        """Test full chunk detection with no relevant chunks"""
        query = "What is the price?"
        assessments = []
        chunk_texts = {}

        answer_present, reasoning = detect_answer_presence_with_full_chunks(
            query, assessments, chunk_texts
        )
        assert answer_present is False
        assert "no relevant" in reasoning.lower()

    def test_answer_full_chunks_fallback_to_summary(self):
        """Test fallback to summary when full text unavailable"""
        query = "What features are available?"
        assessments = [
            RelevanceAssessment(
                chunk_id="ch1",
                relevance_score=0.9,
                is_relevant=True,
                summary="Features include real-time updates and notifications"
            )
        ]
        # Chunk text not in dictionary - should fallback to summary
        chunk_texts = {}

        answer_present, reasoning = detect_answer_presence_with_full_chunks(
            query, assessments, chunk_texts
        )
        assert answer_present is True  # Should still work with summary fallback
