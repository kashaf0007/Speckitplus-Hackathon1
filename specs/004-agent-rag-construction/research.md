# Research: Agent Construction & RAG Integration

**Feature**: 004-agent-rag-construction
**Date**: 2025-12-18
**Purpose**: Resolve technical unknowns and document architectural decisions for controlled RAG agent implementation

## Research Areas

### 1. OpenAI Agents SDK Integration

**Decision**: Use OpenAI Agents SDK (version >=0.6.3) for agent orchestration

**Rationale**:
- Already present in pyproject.toml dependencies (openai-agents>=0.6.3)
- Provides high-level abstractions for:
  - Agent initialization with system prompts
  - Context management for single-turn interactions
  - Structured output generation (JSON responses)
  - Tool integration (though not needed for this feature)
- Supports strict instruction following for book-grounding constraints
- Native integration with OpenAI API for response generation

**Alternatives Considered**:
- **LangChain**: More complex framework with unnecessary abstractions (chains, memory, agents). Over-engineered for single-turn, stateless requirement. Rejected due to complexity.
- **Direct OpenAI API calls**: Would require manual prompt engineering, context formatting, and response parsing. OpenAI Agents SDK provides these out-of-box with better maintainability.
- **Custom agent framework**: Would add development time and maintenance burden. Rejected in favor of supported SDK.

**Implementation Notes**:
- Agent initialization with system prompt defining book-only constraints
- Single-turn interaction mode (no conversation history)
- Structured output schema for consistent JSON responses
- Error handling for API rate limits and timeouts

### 2. System Prompt Design for Hallucination Prevention

**Decision**: Implement multi-layered prompt strategy with explicit constraints and examples

**Rationale**:
- Achieving 0% hallucination rate requires more than just instructing "don't hallucinate"
- Research on prompt engineering shows multi-layered constraints are most effective:
  1. Role definition: "You are a book content assistant"
  2. Strict constraints: "Use ONLY the provided context"
  3. Explicit refusal instruction: "If answer not found, respond: 'This information is not available in the book'"
  4. Example demonstrations (few-shot)
  5. Output format specification

**Alternatives Considered**:
- **Single instruction**: "Answer only from context" - Too vague, high hallucination risk. Rejected.
- **Fine-tuned model**: Custom fine-tuning for book-grounding - Expensive, slow iteration, overkill for prompt-solvable problem. Rejected.
- **Retrieval augmentation alone**: Relying only on retrieval without prompt constraints - LLMs naturally use world knowledge unless explicitly instructed otherwise. Rejected.

**Implementation Notes**:
```
System prompt structure:
1. Role: Book content assistant
2. Input specification: Context chunks + user question
3. Constraints:
   - Use ONLY provided context
   - NO external knowledge
   - NO inference beyond text
   - If answer absent → exact refusal message
4. Output format: JSON with answer, sources, matched_chunks
5. Examples: 2-3 few-shot demonstrations
```

### 3. Selected Text Override Architecture

**Decision**: Priority-based context routing at retrieval layer

**Rationale**:
- User-selected text takes absolute priority per spec FR-011
- Clearest implementation: Check for selected_text in request → skip Qdrant call entirely if present
- Prevents any mixing of selected text with retrieved chunks (clean separation)
- Simplifies agent logic (context is just text, regardless of source)

**Alternatives Considered**:
- **Dual context handling**: Pass both selected text and retrieved chunks to agent with priority indicator - Complex, risks context mixing, agent might use both sources. Rejected.
- **Post-retrieval filtering**: Retrieve chunks then discard if selected text present - Wasteful API calls to Qdrant, unnecessary latency. Rejected.
- **Agent-level decision**: Let agent choose between contexts - Violates deterministic behavior requirement, unpredictable. Rejected.

**Implementation Notes**:
```python
if request.selected_text:
    context = request.selected_text
    source_metadata = {"type": "selected_text"}
    # Skip Qdrant retrieval entirely
else:
    # Embed query with Cohere
    # Retrieve from Qdrant
    context = format_chunks(retrieved_chunks)
    source_metadata = extract_chunk_metadata(retrieved_chunks)
```

### 4. Integration with Retrieval Validation Assistant (003-retrieval-validation)

**Decision**: Call validation assistant after Qdrant retrieval, before agent response generation

**Rationale**:
- Validation assistant (003-retrieval-validation) assesses:
  - Chunk relevance to query
  - Answer presence in retrieved data
  - Evidence extraction
  - Retrieval quality rating (Good/Partial/Poor)
- Early filtering prevents agent from attempting to answer with poor-quality retrievals
- Validation output provides structured decision point for refusal message
- Aligns with spec FR-005 requirement to integrate validation

**Alternatives Considered**:
- **Skip validation, rely on agent judgment**: Agent might attempt answers from irrelevant chunks, higher hallucination risk. Rejected.
- **Post-agent validation**: Validate after agent generates answer - Too late, wasted generation cost, can't prevent hallucinations. Rejected.
- **Parallel validation**: Validate and generate simultaneously - Can't use validation results to prevent bad generations. Rejected.

**Implementation Notes**:
```python
# Flow for standard query (no selected text)
1. Embed query (Cohere)
2. Retrieve chunks (Qdrant, top 5-10)
3. Validate retrieval (003-retrieval-validation)
4. Check validation.answer_present:
   - If False → return refusal message immediately
   - If True → proceed to agent with context
5. Generate answer (OpenAI Agent)
6. Return response with sources + matched chunks
```

### 5. Response Format and Source Attribution

**Decision**: Structured JSON response with typed Pydantic models

**Rationale**:
- Spec requires: answer + sources + matched_chunks (FR-013, FR-014)
- Pydantic models provide:
  - Type safety and validation
  - Automatic JSON serialization
  - Self-documenting schemas (for OpenAPI)
  - Easy testing with fixtures
- FastAPI native integration with Pydantic for request/response

**Alternatives Considered**:
- **Plain dict responses**: No type safety, error-prone, poor IDE support. Rejected.
- **Custom serialization**: Reinventing JSON encoding, maintenance burden. Rejected.
- **Protobuf/gRPC**: Over-engineered for HTTP REST API, adds complexity. Rejected.

**Implementation Notes**:
```python
# Pydantic models (Backend/models/agent_models.py)

class AskRequest(BaseModel):
    question: str
    selected_text: Optional[str] = None

class ChunkReference(BaseModel):
    chunk_id: str
    text: str
    page: Optional[int]
    chapter: Optional[str]
    section: Optional[str]

class AskResponse(BaseModel):
    answer: str
    sources: List[str]  # ["chunk_123", "chunk_456"] or ["selected_text"]
    matched_chunks: List[ChunkReference]
    grounded: bool  # Always True for successful responses
    retrieval_quality: Optional[str]  # "Good"/"Partial"/"Poor" from validation
```

### 6. Error Handling and Failure Modes

**Decision**: Explicit error handling for each integration point with graceful degradation

**Rationale**:
- Multiple external dependencies: Cohere API, Qdrant Cloud, OpenAI API, validation assistant
- Each can fail independently
- Spec edge cases require graceful handling (see spec.md Edge Cases section)
- Never fallback to hallucination - always fail safely with clear message

**Alternatives Considered**:
- **Silent failures**: Log errors but return generic responses - Violates user trust, debugging nightmare. Rejected.
- **Retry-only strategy**: Indefinite retries - User waits too long, poor UX. Rejected.
- **Fail fast with stack traces**: Expose internal errors to users - Security risk, poor UX. Rejected.

**Implementation Notes**:
```python
# Error handling matrix

| Failure Point | Detection | User Message |
|---------------|-----------|--------------|
| Qdrant unavailable | Connection error | "Unable to search the book. Please try again later." |
| Cohere API error | HTTP 4xx/5xx | "Unable to process your question. Please try again." |
| OpenAI API error | HTTP 4xx/5xx | "Unable to generate answer. Please try again." |
| Validation assistant error | Import/call failure | Log warning, skip validation, proceed with agent |
| Empty question | request.question == "" | "Please provide a question about the book." |
| No chunks retrieved | Qdrant returns [] | "This information is not available in the book." |
| Validation says answer absent | validation.answer_present == False | "This information is not available in the book." |
| Agent timeout | OpenAI timeout | "Request timed out. Please try again." |

All errors logged with context (question hash, timestamp, error type) for debugging.
```

### 7. Performance Optimization Strategy

**Decision**: Optimize at critical path with measured trade-offs

**Rationale**:
- Target: <5 seconds end-to-end (SC-001)
- Breakdown:
  - Cohere embedding: ~200-500ms
  - Qdrant retrieval: ~100-300ms
  - Validation assistant: ~500-1000ms (within 2s target per SC-001)
  - OpenAI Agent generation: ~1-3s (depends on answer length)
  - Total estimated: 2-5s (within target)

**Alternatives Considered**:
- **Aggressive caching**: Cache query embeddings and responses - Violates freshness if book content updated, adds complexity. Deferred to future optimization.
- **Parallel execution**: Embed + retrieve + validate in parallel - Validation depends on retrieval results, limited parallelization. Minimal gain.
- **Smaller models**: Use faster OpenAI models - May reduce answer quality. Start with gpt-4-turbo, downgrade only if performance issues.

**Implementation Notes**:
- Use async/await for I/O operations (Cohere, Qdrant, OpenAI)
- Set reasonable timeouts: 10s for Qdrant, 15s for OpenAI
- Monitor p95 latency in production, optimize bottlenecks as needed
- No premature optimization - measure first

### 8. Testing Strategy for 0% Hallucination Rate

**Decision**: Multi-level testing with golden datasets and adversarial examples

**Rationale**:
- 0% hallucination rate is non-negotiable (SC-003)
- Cannot be verified with unit tests alone
- Requires test datasets:
  - Positive examples (questions with answers in book)
  - Negative examples (out-of-scope questions)
  - Adversarial examples (questions designed to trigger hallucinations)
  - Edge cases (ambiguous, contradictory content)

**Alternatives Considered**:
- **Manual testing only**: Not scalable, no regression protection. Rejected.
- **Synthetic data generation**: LLM-generated test cases - May miss real failure modes. Use as supplement, not replacement.
- **Production monitoring only**: Too late, affects users. Use in addition to pre-deployment testing.

**Implementation Notes**:
```python
# Test structure (Backend/tests/)

1. Contract tests (contract/test_ask_endpoint.py):
   - Request/response schema validation
   - HTTP status codes
   - Error message formats

2. Integration tests (integration/test_agent_retrieval.py):
   - Qdrant mock with predefined chunks
   - Cohere mock for embeddings
   - Verify no hallucinations with known data

3. Unit tests (unit/):
   - test_agent_rag.py: Agent initialization, prompt construction
   - test_selected_text_override.py: Priority routing logic

4. Hallucination test suite (integration/test_hallucination_prevention.py):
   - 50 test cases (25 in-scope, 25 out-of-scope) per SC-004
   - 100 questions for 0% hallucination rate measurement per SC-003
   - Golden dataset: pre-labeled questions with expected answers
   - Adversarial examples: designed to trigger hallucinations
   - Automated grading: compare actual vs expected, flag deviations

5. Performance tests (integration/test_performance.py):
   - Measure end-to-end latency for 100 requests
   - Verify <5s target (SC-001)
   - Concurrent request testing
```

## Summary of Decisions

| Area | Decision | Critical For |
|------|----------|--------------|
| Agent Framework | OpenAI Agents SDK >=0.6.3 | FR-017, already in dependencies |
| System Prompt | Multi-layered constraints + few-shot examples | SC-003 (0% hallucination) |
| Context Routing | Selected text priority at retrieval layer | FR-011, SC-005 |
| Validation Integration | Call validation assistant after retrieval, before generation | FR-005, SC-003 |
| Response Format | Pydantic models (AskRequest, AskResponse, ChunkReference) | FR-013, FR-014 |
| Error Handling | Explicit handling per integration point, never hallucinate | Edge cases in spec.md |
| Performance | Async I/O, reasonable timeouts, measure before optimizing | SC-001 (<5s) |
| Testing | Multi-level with golden datasets and adversarial examples | SC-003, SC-004 |

## Next Steps

Proceed to Phase 1:
1. Generate data-model.md (Pydantic models design)
2. Create contracts/ask_endpoint.yaml (OpenAPI spec)
3. Generate quickstart.md (developer onboarding)
