# Quickstart: Agent Construction & RAG Integration

**Feature**: 004-agent-rag-construction
**Date**: 2025-12-18
**Target Audience**: Developers implementing or maintaining the RAG agent

## Overview

This guide walks you through setting up, testing, and using the controlled RAG agent that answers questions strictly from book content.

## Prerequisites

1. **Existing features operational**:
   - 002-content-embedding-pipeline: Qdrant indexed with book content
   - 003-retrieval-validation: Validation assistant available

2. **Environment**:
   - Python 3.11+
   - Backend/ directory with existing pyproject.toml

3. **API Keys** (add to `.env`):
   ```env
   OPENAI_API_KEY=sk-...           # For OpenAI Agents SDK
   COHERE_API_KEY=...              # For query embeddings (already set)
   QDRANT_URL=...                  # Qdrant Cloud URL (already set)
   QDRANT_API_KEY=...              # Qdrant API key (already set)
   ```

## Installation

1. **Navigate to Backend directory**:
   ```bash
   cd Backend/
   ```

2. **Install dependencies** (if not already installed):
   ```bash
   # Dependencies already in pyproject.toml:
   # - openai-agents>=0.6.3
   # - fastapi>=0.104.0
   # - cohere>=5.13.0
   # - qdrant-client>=1.12.0
   # - pydantic>=2.0

   # If using uv (recommended):
   uv sync

   # Or pip:
   pip install -e .
   ```

3. **Verify dependencies**:
   ```bash
   python -c "import openai; import cohere; import qdrant_client; print('All dependencies installed')"
   ```

## Project Structure

```
Backend/
├── agent_rag.py              # NEW - Agent orchestration
├── models/
│   └── agent_models.py       # NEW - Pydantic models
├── api.py                    # MODIFIED - Add /ask endpoint
├── chatbot.py                # EXISTING - Cohere/Qdrant clients (reuse)
├── retrieval_validation/     # EXISTING - Validation assistant
│   └── validator.py
└── tests/
    ├── contract/
    │   └── test_ask_endpoint.py
    ├── integration/
    │   └── test_agent_retrieval.py
    └── unit/
        ├── test_agent_rag.py
        └── test_selected_text_override.py
```

## Implementation Checklist

Use this checklist during implementation (detailed tasks in tasks.md):

- [ ] Create `Backend/models/agent_models.py` with Pydantic models
- [ ] Create `Backend/agent_rag.py` with agent orchestration logic
- [ ] Update `Backend/api.py` to add POST /ask endpoint
- [ ] Implement selected-text priority routing
- [ ] Integrate retrieval validation assistant (003-retrieval-validation)
- [ ] Write comprehensive system prompt for 0% hallucination
- [ ] Add error handling for all integration points
- [ ] Write contract tests (test_ask_endpoint.py)
- [ ] Write integration tests (test_agent_retrieval.py)
- [ ] Write unit tests (test_agent_rag.py, test_selected_text_override.py)
- [ ] Create hallucination test suite (100 test cases per SC-003)
- [ ] Measure and optimize for <5s latency target
- [ ] Update API documentation with /ask endpoint

## Development Workflow

### 1. Start the FastAPI Server

```bash
cd Backend/
uvicorn api:app --reload --port 8000
```

Server will be available at `http://localhost:8000`

### 2. Test the /ask Endpoint

**Standard Query (with retrieval)**:
```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is the main theme of Chapter 3?",
    "selected_text": null
  }'
```

Expected response:
```json
{
  "answer": "Chapter 3 explores the theme of resilience through the protagonist's journey.",
  "sources": ["chunk_123", "chunk_124"],
  "matched_chunks": [
    {
      "chunk_id": "chunk_123",
      "text": "The protagonist faces significant challenges...",
      "page": 45,
      "chapter": "Chapter 3",
      "section": "Rising Action"
    }
  ],
  "grounded": true,
  "retrieval_quality": "Good"
}
```

**Selected Text Query (no retrieval)**:
```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Explain this passage",
    "selected_text": "The author describes the process of photosynthesis in detail..."
  }'
```

Expected response:
```json
{
  "answer": "This passage describes photosynthesis as the process by which plants convert light energy into chemical energy.",
  "sources": ["selected_text"],
  "matched_chunks": [
    {
      "chunk_id": "selected_text",
      "text": "The author describes the process of photosynthesis in detail...",
      "page": null,
      "chapter": null,
      "section": null
    }
  ],
  "grounded": true,
  "retrieval_quality": null
}
```

**Out-of-Scope Question (refusal)**:
```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is the weather today?",
    "selected_text": null
  }'
```

Expected response:
```json
{
  "answer": "This information is not available in the book.",
  "sources": [],
  "matched_chunks": [],
  "grounded": false,
  "retrieval_quality": "Poor"
}
```

### 3. Run Tests

**Contract tests** (API schema validation):
```bash
pytest tests/contract/test_ask_endpoint.py -v
```

**Integration tests** (with Qdrant mock):
```bash
pytest tests/integration/test_agent_retrieval.py -v
```

**Unit tests** (agent logic):
```bash
pytest tests/unit/ -v
```

**Hallucination prevention tests** (critical for 0% hallucination rate):
```bash
pytest tests/integration/test_hallucination_prevention.py -v
```

**Full test suite**:
```bash
pytest tests/ -v --cov=Backend --cov-report=html
```

### 4. Performance Testing

Measure end-to-end latency (target: <5 seconds per SC-001):
```bash
pytest tests/integration/test_performance.py -v
```

Or manual timing:
```bash
time curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is the main theme of Chapter 3?"}'
```

## Key Implementation Details

### System Prompt Structure

The system prompt is critical for achieving 0% hallucination rate. Structure:

```python
SYSTEM_PROMPT = """
You are a book content assistant. Your role is to answer questions using ONLY the provided context.

**STRICT RULES**:
1. Use ONLY the context provided below. DO NOT use any external knowledge.
2. DO NOT infer, guess, or combine information not explicitly stated.
3. If the answer is not in the context, respond EXACTLY: "This information is not available in the book."
4. Keep answers short (1-5 sentences) and factual.
5. Include direct quotes or paraphrases from the context.

**CONTEXT**:
{context_text}

**QUESTION**:
{question}

**OUTPUT FORMAT** (JSON):
{{
  "answer": "Your answer here or refusal message",
  "evidence": ["Quote 1 from context", "Quote 2 from context"]
}}
"""
```

### Context Routing Logic

```python
# In agent_rag.py

def prepare_context(request: AskRequest) -> AgentContext:
    if request.selected_text:
        # Priority: selected text overrides retrieval
        return AgentContext(
            context_text=request.selected_text,
            context_type="selected_text",
            validation_result=None,
            source_metadata=[{"chunk_id": "selected_text"}]
        )
    else:
        # Standard retrieval path
        # 1. Embed query with Cohere
        query_embedding = cohere_client.embed(request.question)

        # 2. Retrieve from Qdrant
        chunks = qdrant_client.search(query_embedding, limit=10)

        # 3. Validate retrieval
        validation = validate_retrieval(request.question, chunks)

        # 4. Check answer presence
        if not validation.answer_present:
            raise NoAnswerFoundError()  # Triggers refusal response

        # 5. Create context
        context_text = format_chunks(chunks)
        return AgentContext(
            context_text=context_text,
            context_type="retrieval",
            validation_result=validation,
            source_metadata=extract_metadata(chunks)
        )
```

### Error Handling Pattern

```python
# In api.py

@app.post("/ask", response_model=AskResponse)
async def ask_question(request: AskRequest):
    try:
        # Prepare context (routing + validation)
        context = prepare_context(request)

        # Generate answer with OpenAI Agent
        result = await agent.generate(context)

        # Return response
        return AskResponse(
            answer=result.answer,
            sources=result.sources,
            matched_chunks=result.matched_chunks,
            grounded=True,
            retrieval_quality=context.validation_result.retrieval_quality if context.validation_result else None
        )

    except NoAnswerFoundError:
        # Refusal response
        return AskResponse(
            answer="This information is not available in the book.",
            sources=[],
            matched_chunks=[],
            grounded=False,
            retrieval_quality="Poor"
        )

    except QdrantConnectionError:
        raise HTTPException(
            status_code=500,
            detail="Unable to search the book. Please try again later."
        )

    except OpenAIError:
        raise HTTPException(
            status_code=500,
            detail="Unable to generate answer. Please try again."
        )

    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        raise HTTPException(
            status_code=500,
            detail="An error occurred. Please try again."
        )
```

## Testing Guidelines

### Critical Tests for 0% Hallucination Rate

1. **Golden Dataset Tests** (tests/integration/test_hallucination_prevention.py):
   - 50 test cases: 25 in-scope, 25 out-of-scope (per SC-004)
   - Pre-labeled expected answers
   - Automated comparison: actual vs expected
   - FAIL if any hallucination detected

2. **Adversarial Tests**:
   - Questions designed to trigger hallucinations
   - Ambiguous questions
   - Questions with plausible but incorrect answers

3. **Selected Text Tests**:
   - Verify no retrieval when selected_text provided
   - Verify agent uses only selected text
   - Verify correct refusal when answer not in selection

### Performance Tests

Target: <5 seconds end-to-end (SC-001)

```python
# tests/integration/test_performance.py

def test_latency_target():
    start = time.time()
    response = client.post("/ask", json={
        "question": "What is the main theme of Chapter 3?"
    })
    duration = time.time() - start

    assert response.status_code == 200
    assert duration < 5.0, f"Latency {duration}s exceeds 5s target"
```

## Monitoring and Debugging

### Key Metrics to Track

1. **Latency**:
   - Cohere embedding time
   - Qdrant retrieval time
   - Validation assistant time
   - OpenAI generation time
   - End-to-end time

2. **Accuracy**:
   - Hallucination rate (target: 0%)
   - Refusal accuracy (out-of-scope questions)
   - Answer relevance (user feedback)

3. **Error Rates**:
   - Qdrant connection errors
   - OpenAI API errors
   - Validation failures

### Logging

```python
import logging

logger = logging.getLogger(__name__)

# Log each request with context
logger.info(
    "Ask request",
    extra={
        "question_hash": hash(request.question),
        "has_selected_text": bool(request.selected_text),
        "timestamp": datetime.utcnow().isoformat()
    }
)

# Log latencies
logger.info(
    "Ask response",
    extra={
        "latency_total": duration,
        "latency_retrieval": retrieval_time,
        "latency_validation": validation_time,
        "latency_generation": generation_time,
        "grounded": response.grounded,
        "retrieval_quality": response.retrieval_quality
    }
)
```

## Common Issues and Solutions

### Issue: Agent hallucinates despite system prompt

**Solution**:
- Review system prompt - ensure explicit constraints
- Add more few-shot examples demonstrating refusal
- Verify validation assistant is correctly filtering poor retrievals
- Check if context is properly formatted and passed to agent

### Issue: High latency (>5 seconds)

**Solution**:
- Profile each step: embedding, retrieval, validation, generation
- Optimize bottleneck:
  - Qdrant: reduce chunk count, optimize query
  - OpenAI: use faster model (gpt-4-turbo), reduce max tokens
  - Validation: optimize validation logic
- Consider caching query embeddings (if appropriate)

### Issue: Incorrect refusals (answer is in book but agent refuses)

**Solution**:
- Check retrieval: are relevant chunks being retrieved?
- Check validation: is validation assistant correctly identifying answer presence?
- Review system prompt: is refusal instruction too aggressive?

### Issue: Selected text override not working

**Solution**:
- Verify `if request.selected_text:` condition is checked FIRST
- Ensure no retrieval calls when selected_text present
- Check logs to confirm context_type="selected_text"

## Next Steps

After implementing the agent:

1. **Run full test suite**: Ensure all tests pass, especially hallucination tests
2. **Performance profiling**: Verify <5s latency target
3. **Manual QA**: Test with diverse questions from the book
4. **User acceptance testing**: Get feedback on answer quality
5. **Documentation**: Update main README with /ask endpoint usage
6. **Deployment**: Deploy to staging, then production with monitoring

## Resources

- **Spec**: [spec.md](./spec.md)
- **Plan**: [plan.md](./plan.md)
- **Data Model**: [data-model.md](./data-model.md)
- **API Contract**: [contracts/ask_endpoint.yaml](./contracts/ask_endpoint.yaml)
- **Research**: [research.md](./research.md)
- **OpenAI Agents SDK**: https://github.com/openai/openai-agents-python
- **Pydantic**: https://docs.pydantic.dev/
- **FastAPI**: https://fastapi.tiangolo.com/
