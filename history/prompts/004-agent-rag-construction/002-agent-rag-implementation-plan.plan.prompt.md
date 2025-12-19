---
id: 002
title: Agent RAG Implementation Plan
stage: plan
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5
feature: 004-agent-rag-construction
branch: 004-agent-rag-construction
user: system
command: /sp.plan
labels: ["planning", "architecture", "rag", "openai-agents", "fastapi", "qdrant"]
links:
  spec: specs/004-agent-rag-construction/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-agent-rag-construction/plan.md
  - specs/004-agent-rag-construction/research.md
  - specs/004-agent-rag-construction/data-model.md
  - specs/004-agent-rag-construction/quickstart.md
  - specs/004-agent-rag-construction/contracts/ask_endpoint.yaml
  - CLAUDE.md
tests:
  - Validated Constitution Check (passed - no violations)
  - Verified design aligns with simplicity, testability, clear contracts
---

## Prompt

Create FastAPI /ask endpoint

Initialize OpenAI Agent with strict book-only system prompt

If selected text exists → skip retrieval

Else → embed query (Cohere) → retrieve chunks (Qdrant)

Pass context + query to agent

Agent answers only from context

Return JSON: answer, sources, matched_chunks

If answer missing → return refusal message

## Response snapshot

Created comprehensive implementation plan for Agent Construction & RAG Integration (004-agent-rag-construction).

**Planning Deliverables**:

1. **plan.md** (150 lines) - Complete architectural plan:
   - Technical Context: Python 3.11, FastAPI, OpenAI Agents SDK, Cohere, Qdrant
   - Constitution Check: ✅ PASS (simplicity, testability, clear contracts)
   - Project Structure: Backend/ extension with agent_rag.py, models/, tests/
   - Phase 1 Re-Evaluation: ✅ PASS - design complete and ready for implementation

2. **research.md** (270 lines) - Architectural research resolving 8 key decisions:
   - OpenAI Agents SDK integration strategy
   - System prompt design for 0% hallucination rate (multi-layered constraints + few-shot)
   - Selected text override architecture (priority-based routing)
   - Retrieval validation integration (call after retrieval, before generation)
   - Response format (Pydantic models)
   - Error handling matrix (6 failure modes)
   - Performance optimization strategy (<5s target breakdown)
   - Testing strategy (golden datasets + adversarial examples)

3. **data-model.md** (350 lines) - Complete data model specification:
   - Request models: AskRequest (question, selected_text)
   - Response models: AskResponse (answer, sources, matched_chunks, grounded, retrieval_quality)
   - ChunkReference model (chunk_id, text, page, chapter, section)
   - Internal models: AgentContext, ValidationResult
   - State transitions and data flow diagrams
   - Validation rules for all entities

4. **contracts/ask_endpoint.yaml** (300 lines) - OpenAPI 3.0.3 specification:
   - POST /ask endpoint with full schema
   - Request/response examples (standard query, selected text, refusal, errors)
   - HTTP status codes (200, 400, 500, 503)
   - Error response formats
   - Performance and constraint documentation

5. **quickstart.md** (400 lines) - Developer onboarding guide:
   - Installation and setup instructions
   - Implementation checklist (aligned with spec requirements)
   - Development workflow with curl examples
   - System prompt structure template
   - Context routing logic code samples
   - Error handling patterns
   - Testing guidelines (contract, integration, unit, hallucination tests)
   - Performance testing and monitoring guidance
   - Common issues and solutions

6. **CLAUDE.md** - Agent context updated:
   - Added Python 3.11, FastAPI, OpenAI Agents SDK, Cohere, Qdrant, Pydantic to tech stack
   - Updated via update-agent-context.ps1 script

**Key Architectural Decisions**:

1. **Agent Framework**: OpenAI Agents SDK (already in dependencies)
2. **Hallucination Prevention**: Multi-layered system prompt with explicit constraints and few-shot examples
3. **Context Routing**: Selected text takes absolute priority at retrieval layer (skip Qdrant entirely if present)
4. **Validation Integration**: Call 003-retrieval-validation after retrieval, before agent generation
5. **Response Format**: Pydantic models (AskRequest, AskResponse, ChunkReference) for type safety
6. **Error Handling**: Explicit handling per integration point (Qdrant, Cohere, OpenAI, validation)
7. **Performance**: Async I/O, reasonable timeouts, <5s end-to-end target
8. **Testing**: Multi-level with 100 test cases for 0% hallucination rate validation

**Technical Flow** (from user input):
```
User Question → AskRequest
    ↓
[if selected_text] → AgentContext (selected) → OpenAI Agent
    ↓
[else] → Cohere Embed → Qdrant Retrieve → Validate (003-retrieval-validation)
    ↓
    [if answer_present == False] → Refusal Response
    ↓
    [else] → AgentContext (retrieved) → OpenAI Agent
    ↓
Agent generates answer → AskResponse
    ├── answer (1-5 sentences)
    ├── sources (chunk IDs or "selected_text")
    ├── matched_chunks (full details)
    ├── grounded (bool)
    └── retrieval_quality ("Good"/"Partial"/"Poor")
```

**Project Structure**:
- Extends existing Backend/ directory
- Single new module: agent_rag.py (orchestration)
- New subdirectory: models/ (Pydantic models)
- Comprehensive test suite: contract/, integration/, unit/
- Reuses: chatbot.py (Cohere/Qdrant), retrieval_validation/, api.py (FastAPI)

**Constitution Check Results**:
- ✅ Simplicity: No new architectural layers, reuses existing dependencies
- ✅ Testability: Independent contract, integration, and unit tests
- ✅ Clear Contracts: OpenAPI spec with full schemas and examples
- ✅ No Over-Engineering: Minimal additions, no premature optimization

**Phase 1 Re-Evaluation**: ✅ PASS - Design is complete, simple, testable, and well-documented. Ready for Phase 2 (tasks generation).

## Outcome

- ✅ Impact: Complete architectural plan for RAG agent, ready for implementation phase
- 🧪 Tests: Constitution Check passed (initial + re-evaluation), design validated against best practices
- 📁 Files: plan.md, research.md, data-model.md, quickstart.md, contracts/ask_endpoint.yaml, CLAUDE.md (updated)
- 🔁 Next prompts: /sp.tasks to generate implementation tasks from plan.md
- 🧠 Reflection: User-provided technical flow outline successfully integrated into architecture. All 8 research areas resolved with clear decisions and rationales. Multi-layered system prompt strategy critical for achieving 0% hallucination rate requirement. Selected-text priority routing ensures deterministic behavior per spec. Design balances simplicity (reuse existing code) with robustness (comprehensive error handling and testing).

## Evaluation notes (flywheel)

- Failure modes observed: None - planning workflow executed successfully
- Graders run and results (PASS/FAIL): Constitution Check PASS (initial), Constitution Check PASS (Phase 1 re-evaluation)
- Prompt variant (if applicable): Standard /sp.plan workflow with user technical flow integration
- Next experiment (smallest change to try): Consider adding ADR for system prompt strategy (architecturally significant: affects hallucination rate, requires explicit constraints design)
