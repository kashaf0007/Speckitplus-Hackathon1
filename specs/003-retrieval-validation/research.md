# Research & Technology Decisions: Retrieval Validation Assistant

**Feature**: 003-retrieval-validation
**Date**: 2025-12-18
**Purpose**: Document technology choices, research findings, and design rationale for the retrieval validation component.

## Overview

The Retrieval Validation Assistant must evaluate retrieved chunks from Qdrant/Cohere for relevance, answer presence, and quality without hallucinating or adding external knowledge. Key research areas:

1. Semantic similarity measurement for relevance assessment
2. Answer presence detection strategies
3. Evidence extraction patterns
4. Quality rating criteria and thresholds
5. Integration patterns with existing RAG pipeline

## Key Decisions

### Decision 1: Semantic Similarity Measurement

**Decision**: Use Cohere's rerank API for relevance assessment

**Rationale**:
- Already using Cohere for embeddings in the existing pipeline (main.py:26)
- Cohere's rerank model specifically designed for relevance scoring in retrieval contexts
- Provides normalized relevance scores (0-1) per chunk without requiring custom threshold tuning
- Faster than computing cosine similarity with embeddings
- Consistent with existing stack (no new providers)

**Alternatives Considered**:
1. **Cosine similarity with Cohere embeddings**: More expensive (requires embedding both query and all chunks), less accurate than rerank
2. **BM25 (lexical matching)**: Too simplistic, misses semantic relevance, fails on paraphrased content
3. **Cross-encoder model (local)**: Adds complexity, requires model hosting/versioning, slower inference

**Implementation Notes**:
- Use `cohere_client.rerank()` with query and chunk texts
- Relevance threshold: score >= 0.3 (based on Cohere rerank best practices)
- Batch process all chunks in one rerank call for efficiency

### Decision 2: Answer Presence Detection

**Decision**: Rule-based entailment check with keyword extraction + semantic validation

**Rationale**:
- Answer presence is binary (yes/no) - simpler than full answer generation
- Must verify answer is *explicitly present*, not inferable
- Two-stage approach balances precision and recall:
  1. Extract query intent keywords (e.g., "What year" → looking for year/date)
  2. Validate relevant chunks contain those entities using semantic search

**Alternatives Considered**:
1. **LLM-based entailment** (e.g., "Does chunk answer query?"): Risk of hallucination, slower, non-deterministic
2. **Keyword-only matching**: Fails on paraphrasing, too brittle
3. **Question-answering model**: Violates constraint (would generate answers, not just validate presence)

**Implementation Notes**:
- Parse query type (who/what/when/where/why/how)
- Extract expected answer type (person/thing/date/location/reason/method)
- Search relevant chunks for matching entity types
- If found, mark "Answer Present: Yes" with quoted evidence

### Decision 3: Evidence Extraction

**Decision**: Sentence-level extraction with context window

**Rationale**:
- Users need to verify answers trace back to source material
- Sentence boundaries provide natural segmentation
- Include surrounding context (±1 sentence) for clarity

**Alternatives Considered**:
1. **Paragraph-level**: Too verbose, includes irrelevant content
2. **Exact phrase matching**: May miss context needed to understand quote
3. **Token-span extraction** (like SQuAD): Overly complex for validation use case

**Implementation Notes**:
- Use NLTK or spaCy for sentence tokenization
- Extract sentences containing answer-relevant keywords
- Include chunk ID + sentence index for traceability
- Quote verbatim (no paraphrasing or summarization)

### Decision 4: Quality Rating Criteria

**Decision**: Three-tier rating (Good/Partial/Poor) based on coverage + relevance

**Rating Logic**:
- **Good**: >=2 relevant chunks AND answer fully present AND avg relevance score >= 0.6
- **Partial**: 1 relevant chunk OR answer partially present OR avg relevance score 0.3-0.6
- **Poor**: 0 relevant chunks OR avg relevance score < 0.3

**Rationale**:
- Simple categorical system easy to interpret and act upon
- "Good" signals no retrieval issues, proceed to answer generation
- "Partial" signals potential need for query expansion or re-retrieval
- "Poor" signals retrieval failure, warn user or fallback

**Alternatives Considered**:
1. **Continuous score (0-100)**: Harder to interpret, requires arbitrary thresholds
2. **Five-tier rating**: Over-engineered for use case, harder to calibrate
3. **Binary (pass/fail)**: Too coarse, misses "Partial" signal for improvement

**Implementation Notes**:
- Calculate aggregate metrics from relevance + answer presence results
- Log quality rating for observability/debugging
- Include reasoning in output (e.g., "Partial: Only 1 relevant chunk found")

### Decision 5: Integration Pattern

**Decision**: Synchronous function call after Qdrant retrieval, before answer generation

**Integration Point**:
```python
# In main.py or new chatbot module
retrieved_chunks = qdrant_client.search(...)  # Existing retrieval
validation_result = validate_retrieval(query, retrieved_chunks)  # NEW

if not validation_result.answer_present:
    return "No relevant information found in retrieved data."

# Proceed to answer generation with validated chunks + evidence
```

**Rationale**:
- Synchronous: Validation is fast (<2s), no need for async complexity
- Fail-fast: Block answer generation if validation fails (prevent hallucinations)
- Clean separation: Validation logic isolated in `retrieval_validation/` module

**Alternatives Considered**:
1. **Async validation**: Unnecessary complexity for sub-2s operation
2. **Post-answer validation**: Too late - answer may already be hallucinated
3. **Parallel validation + generation**: Wastes compute if validation fails

## Technology Stack Summary

| Component | Technology | Justification |
|-----------|------------|---------------|
| Core Language | Python 3.11+ | Matches existing Backend, mature ecosystem |
| Relevance Scoring | Cohere rerank API | Already in use, purpose-built for retrieval relevance |
| Data Models | Pydantic v2 | Type safety, validation, JSON serialization |
| Sentence Tokenization | NLTK (nltk.sent_tokenize) | Lightweight, battle-tested, no ML overhead |
| Testing | pytest + fixtures | Standard Python testing, easy mock chunks |
| Integration | Synchronous function | Simple, fast, fail-fast semantics |

## Best Practices Applied

1. **RAG Validation Patterns**:
   - Always validate retrieval before generation (industry standard)
   - Use reranking for relevance scoring (recommended by Cohere, Anthropic, OpenAI)
   - Extract evidence for traceability (GDPR/compliance best practice)

2. **Error Handling**:
   - Graceful degradation: If rerank fails, fall back to embedding-based similarity
   - Return structured errors with diagnostic info
   - Never fail silently (log all validation decisions)

3. **Performance Optimization**:
   - Batch rerank calls (single API request for all chunks)
   - Cache query embeddings if needed for similarity fallback
   - Limit sentence tokenization to relevant chunks only

4. **Observability**:
   - Log all validation decisions with scores/evidence
   - Include timing metrics for performance monitoring
   - Structured output format for downstream analytics

## Open Questions & Future Considerations

1. **Multi-language Support**: Current design assumes English. For other languages, need language-specific sentence tokenizers and potentially different rerank models.

2. **Confidence Scores**: Should we add confidence intervals to quality ratings? (e.g., "Good with 85% confidence")

3. **Adaptive Thresholds**: Should relevance/quality thresholds adapt based on query complexity or domain?

4. **Negative Caching**: Should we cache "answer not present" results to avoid re-validating similar failed queries?

**Resolution**: These are deferred to Phase 2 (post-MVP). Initial implementation focuses on core functionality with fixed thresholds and English-only support.

## Dependencies & Integration

**New Dependencies** (to add to pyproject.toml):
```toml
nltk>=3.9  # Sentence tokenization
pydantic>=2.0  # Already included, verify version
```

**No Changes Needed**:
- cohere>=5.13.0 (already present)
- qdrant-client>=1.12.0 (already present)
- httpx>=0.28.0 (already present)

**NLTK Data Download** (one-time setup):
```python
import nltk
nltk.download('punkt')  # Sentence tokenizer models
```

## Validation Against Requirements

| Requirement | Research Decision | Status |
|-------------|-------------------|--------|
| FR-003: Determine relevance | Cohere rerank API | ✅ Resolved |
| FR-004: Identify answer chunks | Entailment check + entity extraction | ✅ Resolved |
| FR-007: Extract evidence | Sentence-level extraction | ✅ Resolved |
| FR-008: Assess quality | Three-tier rating (Good/Partial/Poor) | ✅ Resolved |
| SC-002: <2s processing | Batch rerank + optimized sentence tokenization | ✅ Feasible |
| SC-006: 0% hallucination | No LLM generation, rule-based validation only | ✅ Guaranteed |

All critical unknowns resolved. Ready for Phase 1 (Data Model & Contracts).
