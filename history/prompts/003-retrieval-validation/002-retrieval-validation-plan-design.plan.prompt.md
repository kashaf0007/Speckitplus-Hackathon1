---
id: 002
title: Retrieval Validation Plan Design
stage: plan
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-retrieval-validation
branch: 003-retrieval-validation
user: Kashaf Aman
command: /sp.plan
labels: ["retrieval-validation", "architecture", "design", "rag", "cohere", "planning"]
links:
  spec: specs/003-retrieval-validation/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/003-retrieval-validation/plan.md
 - specs/003-retrieval-validation/research.md
 - specs/003-retrieval-validation/data-model.md
 - specs/003-retrieval-validation/contracts/validation_api.md
 - specs/003-retrieval-validation/quickstart.md
 - CLAUDE.md
tests:
 - Constitution Check validation (pre-design and post-design - PASSED)
---

## Prompt

Goal

Check that Qdrant + Cohere retrieval returns the right book chunks and nothing irrelevant.

Inputs

User query

Top-k chunks retrieved from Qdrant

Steps

Check relevance of retrieved chunks

Verify answer presence (must be explicitly in chunks)

Extract exact evidence (no paraphrasing)

Rate retrieval quality: Good / Partial / Poor

Output Format
Relevant Chunks:
- Chunk IDs / short summaries

Answer Present:
Yes / No

Evidence:
Exact quoted lines (if Yes)

Retrieval Quality:
Good / Partial / Poor


If no answer exists:

No relevant information found in retrieved data.

Success Criteria

Correct chunks retrieved

No hallucination

Negative queries correctly rejected

## Response snapshot

Created comprehensive implementation plan for Retrieval Validation Assistant with complete architecture and design documentation:

**Phase 0: Research & Technology Decisions (research.md)**:
- **Relevance Assessment**: Cohere rerank API (purpose-built, already in stack, normalized 0-1 scores)
- **Answer Presence Detection**: Rule-based entailment check with keyword extraction + semantic validation
- **Evidence Extraction**: Sentence-level extraction with ±1 sentence context using NLTK
- **Quality Rating**: Three-tier system (Good/Partial/Poor) based on coverage + relevance
- **Integration Pattern**: Synchronous function call after Qdrant retrieval, before answer generation

**Phase 1: Design & Contracts (data-model.md, contracts/, quickstart.md)**:
- **Data Models**: Defined 6 core entities with Pydantic schemas:
  - ValidationRequest (input): query, chunks, threshold, request_id
  - TextChunk (input): chunk_id, text, metadata, score
  - ValidationResult (output): relevant_chunks, answer_present, evidence, quality, reasoning, timing
  - RelevanceAssessment, EvidenceQuote, QualityRating (internal/output)
- **API Contract**: Single function `validate_retrieval()` with comprehensive error handling
- **Integration Guide**: Complete quickstart with examples, error handling, performance tips

**Technical Stack**:
- Language: Python 3.11+ (matches existing Backend)
- Core: Cohere rerank API, Pydantic v2, NLTK
- Testing: pytest with fixtures
- Integration: Synchronous library call
- Dependencies: Reuses existing (Cohere, Qdrant, httpx)

**Project Structure**:
```
Backend/
├── retrieval_validation/       # New module
│   ├── validator.py            # Core logic
│   ├── models.py               # Pydantic models
│   ├── relevance.py            # Relevance assessment
│   ├── evidence.py             # Evidence extraction
│   └── quality.py              # Quality rating
├── tests/
│   ├── unit/test_retrieval_validation/
│   ├── integration/
│   └── fixtures/sample_chunks.py
```

**Constitution Check**: ✅ PASSED (pre-design and post-design)
- Library-first: Standalone module in Backend/
- Test-first: TDD workflow defined
- Simplicity: Minimal dependencies, reuses existing stack
- Single responsibility: Validation only, no generation
- Clear contracts: Comprehensive API documentation
- Performance: Design meets <2s requirement
- Observability: Timing metrics, structured logging

## Outcome

- ✅ Impact: Complete architectural plan ready for task breakdown. All technical decisions documented, data models defined, API contract specified.
- 🧪 Tests: Constitution Check passed (pre and post-design), test structure defined
- 📁 Files: 6 files created (plan.md, research.md, data-model.md, validation_api.md, quickstart.md, PHR) + 1 updated (CLAUDE.md)
- 🔁 Next prompts: /sp.tasks to break down implementation into testable tasks following TDD workflow
- 🧠 Reflection: Design leverages existing infrastructure (Cohere, Qdrant, Pydantic) for simplicity. Key insight: Cohere rerank API is purpose-built for this exact use case (relevance scoring), avoiding custom similarity threshold tuning. Sentence-level evidence extraction balances granularity with context. Three-tier quality rating provides actionable signal for retrieval debugging.

## Evaluation notes (flywheel)

- Failure modes observed: None. Planning process completed successfully with all design artifacts generated.
- Graders run and results (PASS/FAIL): Constitution Check - PASS (8/8 criteria met pre and post-design)
- Prompt variant (if applicable): Standard /sp.plan workflow with user clarification input integrated
- Next experiment (smallest change to try): During implementation, validate Cohere rerank threshold (0.3) with real book data - may need tuning based on corpus characteristics
