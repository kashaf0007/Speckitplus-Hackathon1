# Implementation Plan: Agent Construction & RAG Integration

**Branch**: `004-agent-rag-construction` | **Date**: 2025-12-18 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-agent-rag-construction/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build a controlled RAG agent using OpenAI Agents SDK that answers user questions strictly from book content. The system integrates with existing Qdrant vector database (Cohere embeddings from 002-content-embedding-pipeline) and retrieval validation assistant (003-retrieval-validation) to ensure 0% hallucination rate. Core features include: FastAPI /ask endpoint, selected-text override (skips retrieval), strict book-grounding constraints, and single-turn stateless interactions.

**User Input Integration**: The user provided a technical flow outline:
- Create FastAPI /ask endpoint
- Initialize OpenAI Agent with strict book-only system prompt
- If selected text exists → skip retrieval, else → embed query (Cohere) → retrieve chunks (Qdrant)
- Pass context + query to agent
- Agent answers only from context
- Return JSON: answer, sources, matched_chunks
- If answer missing → return refusal message

This flow is integrated into the architectural decisions below.

## Technical Context

**Language/Version**: Python 3.11 (matching existing Backend)
**Primary Dependencies**: FastAPI, OpenAI Agents SDK (>=0.6.3), Cohere (>=5.13.0), Qdrant Client (>=1.12.0), Pydantic (>=2.0)
**Storage**: Qdrant Cloud vector database (external dependency from 002-content-embedding-pipeline)
**Testing**: pytest with contract tests, integration tests, and unit tests
**Target Platform**: Linux server (containerized backend)
**Project Type**: Web application backend (extends existing Backend/ directory)
**Performance Goals**: <5 seconds end-to-end response time, <2 seconds retrieval validation
**Constraints**: 0% hallucination rate (non-negotiable), single-turn stateless interactions, answer length 1-5 sentences
**Scale/Scope**: Support concurrent users (target from SC-002: 95% accuracy at scale), integrate with 2 existing features (embedding pipeline + validation assistant)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Constitution Status**: Constitution file contains only template placeholders. Proceeding with standard software engineering best practices:

1. **Simplicity**: Feature extends existing Backend/ structure without introducing new architectural layers. Uses existing dependencies (FastAPI, Cohere, Qdrant) already in pyproject.toml.

2. **Testability**: Design supports independent testing of:
   - FastAPI endpoint (contract tests)
   - Retrieval logic (integration tests with Qdrant mock)
   - Agent behavior (unit tests with context fixtures)
   - Selected-text override (unit tests)

3. **Clear Contracts**:
   - Input: POST /ask with JSON {question: str, selected_text?: str}
   - Output: JSON {answer: str, sources: list, matched_chunks: list, grounded: bool}
   - Error cases defined in edge cases (spec.md)

4. **No Over-Engineering**:
   - Single module (agent_rag.py) for agent orchestration
   - Reuses existing chatbot.py and retrieval_validation/ components
   - No new abstractions beyond what spec requires

**Gate Status**: ✅ PASS - No violations detected. Feature aligns with simplicity, testability, and clear contracts principles.

**Phase 1 Re-Evaluation** (after design completion):

After completing research.md, data-model.md, contracts/, and quickstart.md, the design continues to align with best practices:

1. **Simplicity Maintained**:
   - Added 3 Pydantic models (AskRequest, AskResponse, ChunkReference) - minimal, well-justified
   - Single agent orchestration module (agent_rag.py)
   - No additional frameworks or dependencies
   - Reuses all existing infrastructure

2. **Testability Enhanced**:
   - Clear contract defined in OpenAPI spec (contracts/ask_endpoint.yaml)
   - Test strategy documented in research.md (contract, integration, unit, hallucination tests)
   - Data models enable easy fixture creation for testing
   - Independent testability preserved

3. **Clear Contracts Formalized**:
   - OpenAPI 3.0.3 spec with full request/response schemas
   - Error cases explicitly defined with HTTP status codes
   - Input validation rules documented
   - Examples provided for all scenarios

4. **No Over-Engineering Confirmed**:
   - Design adds only what's required by spec
   - System prompt strategy is necessary for 0% hallucination requirement
   - Pydantic models are standard practice for FastAPI
   - No premature optimization (async I/O only where needed)

**Final Gate Status**: ✅ PASS - Design is complete, simple, testable, and well-documented. Ready for implementation (Phase 2: tasks generation).

## Project Structure

### Documentation (this feature)

```text
specs/004-agent-rag-construction/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── ask_endpoint.yaml  # OpenAPI spec for /ask endpoint
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
Backend/
├── agent_rag.py         # NEW - OpenAI Agent orchestration, context handling
├── api.py               # EXISTING - FastAPI app, add /ask endpoint
├── chatbot.py           # EXISTING - Cohere/Qdrant client utilities (reuse)
├── main.py              # EXISTING - Content embedding pipeline
├── retrieval_validation/ # EXISTING - Validation assistant integration
│   └── validator.py     # Interface to 003-retrieval-validation
├── models/              # NEW - Pydantic models for request/response
│   └── agent_models.py  # AskRequest, AskResponse, ChunkReference
├── tests/
│   ├── contract/        # NEW - API contract tests for /ask
│   │   └── test_ask_endpoint.py
│   ├── integration/     # NEW - Integration tests with Qdrant mock
│   │   └── test_agent_retrieval.py
│   └── unit/           # NEW - Unit tests for agent logic
│       ├── test_agent_rag.py
│       └── test_selected_text_override.py
├── pyproject.toml       # EXISTING - dependencies already include required packages
└── .env                 # EXISTING - OpenAI API key to be added
```

**Structure Decision**: Extends existing Backend/ web application structure. No new top-level directories. All new code lives in Backend/ alongside existing embedding pipeline (main.py) and chatbot utilities (chatbot.py). The agent_rag.py module serves as the main orchestration layer, integrating with existing retrieval_validation/ and chatbot.py components.

**Rationale**:
- Reuses existing FastAPI app in api.py (add new route)
- Leverages existing Cohere/Qdrant clients from chatbot.py
- Integrates with existing retrieval_validation/ from 003-retrieval-validation
- Maintains consistency with current Backend/ organization
- No need for separate frontend (out of scope per spec.md)

## Complexity Tracking

> **No violations detected - this section is not applicable**

The implementation adds minimal complexity:
- Single new orchestration module (agent_rag.py)
- One new API endpoint (/ask)
- Pydantic models for type safety (standard practice)
- Reuses all existing infrastructure (Qdrant, Cohere, FastAPI)

No architectural complexity justification needed.
