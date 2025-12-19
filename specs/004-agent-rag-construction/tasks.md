# Tasks: Agent Construction & RAG Integration

**Input**: Design documents from `/specs/004-agent-rag-construction/`
**Prerequisites**: plan.md (architecture), spec.md (user stories), research.md (decisions), data-model.md (models), contracts/ (API spec)

**Tests**: Test tasks are included based on spec requirements (SC-003: 0% hallucination rate requires comprehensive testing)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md, this feature extends the existing `Backend/` web application structure:
- Source code: `Backend/`
- Tests: `Backend/tests/`
- Models: `Backend/models/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and environment configuration

- [X] T001 Verify Python 3.11 environment in Backend/ directory
- [X] T002 Verify existing dependencies in Backend/pyproject.toml (FastAPI, OpenAI Agents SDK >=0.6.3, Cohere >=5.13.0, Qdrant Client >=1.12.0, Pydantic >=2.0)
- [X] T003 Add GEMINI_API_KEY to Backend/.env file (already has COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY)
- [X] T004 Create Backend/models/ directory for Pydantic models
- [X] T005 [P] Create Backend/tests/contract/ directory for API contract tests
- [X] T006 [P] Create Backend/tests/integration/ directory for integration tests
- [X] T007 [P] Create Backend/tests/unit/ directory for unit tests

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core agent infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T008 Create Backend/models/agent_models.py with AskRequest model (question: str, selected_text: Optional[str])
- [X] T009 Create Backend/models/agent_models.py with AskResponse model (answer: str, sources: List[str], matched_chunks: List[ChunkReference], grounded: bool, retrieval_quality: Optional[str])
- [X] T010 Create Backend/models/agent_models.py with ChunkReference model (chunk_id: str, text: str, page: Optional[int], chapter: Optional[str], section: Optional[str])
- [X] T011 Create Backend/agent_rag.py with AgentOrchestrator class (using Google Gemini API with system prompt)
- [X] T012 Implement system prompt in Backend/agent_rag.py (multi-layered constraints: role, rules, output format, few-shot examples per research.md)
- [X] T013 Implement prepare_context() function in Backend/agent_rag.py (handles context routing: selected text vs retrieval)
- [X] T014 Verify integration with existing Backend/chatbot.py (Cohere embed, Qdrant search functions)
- [X] T015 Verify integration with existing Backend/retrieval_validation/ (validation assistant from 003-retrieval-validation)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Answer Questions from Retrieved Book Chunks (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask questions and receive answers derived strictly from retrieved book content with source references

**Independent Test**: Submit a question with known answer in book → verify agent retrieves relevant chunks → returns grounded answer with chunk IDs and page numbers

### Tests for User Story 1

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [X] T016 [P] [US1] Create contract test for POST /ask endpoint in Backend/tests/contract/test_ask_endpoint.py (validate request/response schemas per contracts/ask_endpoint.yaml)
- [X] T017 [P] [US1] Create integration test for standard query flow in Backend/tests/integration/test_agent_retrieval.py (mock Qdrant with predefined chunks, verify no hallucinations)
- [X] T018 [P] [US1] Create hallucination prevention test suite in Backend/tests/integration/test_hallucination_prevention.py (50 test cases: 25 in-scope, 25 out-of-scope per SC-004)

### Implementation for User Story 1

- [X] T019 [US1] Implement query embedding in Backend/agent_rag.py (use Cohere client from chatbot.py, handle API errors)
- [X] T020 [US1] Implement Qdrant retrieval in Backend/agent_rag.py (use Qdrant client from chatbot.py, retrieve top 10 chunks, handle connection errors)
- [X] T021 [US1] Integrate retrieval validation in Backend/agent_rag.py (call validation assistant, check answer_present field, handle validation errors)
- [X] T022 [US1] Implement refusal logic in Backend/agent_rag.py (if validation.answer_present == False, return refusal message: "This information is not available in the book.")
- [X] T023 [US1] Implement agent generation in Backend/agent_rag.py (pass context to OpenAI Agent, enforce JSON output, handle OpenAI API errors, timeout after 15s)
- [X] T024 [US1] Implement source attribution in Backend/agent_rag.py (extract chunk IDs, map to ChunkReference objects with metadata)
- [X] T025 [US1] Add POST /ask endpoint in Backend/api.py (route to agent_rag.py, return AskResponse, handle all exceptions per research.md error matrix)
- [X] T026 [US1] Add error handling for Qdrant unavailable in Backend/api.py (return 500 with message: "Unable to search the book. Please try again later.")
- [X] T027 [US1] Add error handling for OpenAI unavailable in Backend/api.py (return 500 with message: "Unable to generate answer. Please try again.")
- [X] T028 [US1] Add logging for all request/response in Backend/agent_rag.py (question hash, latencies, grounded status, retrieval quality)

**Checkpoint**: At this point, User Story 1 should be fully functional - users can ask questions and receive grounded answers with sources

---

## Phase 4: User Story 2 - Answer Questions from User-Selected Text (Priority: P1)

**Goal**: Enable users to highlight specific text and ask questions about that selection without system searching elsewhere

**Independent Test**: Provide selected text + question → verify agent answers only from selection without Qdrant retrieval → returns answer with "selected_text" source

### Tests for User Story 2

- [X] T029 [P] [US2] Create unit test for selected text override in Backend/tests/unit/test_selected_text_override.py (verify no Qdrant calls when selected_text present)
- [X] T030 [P] [US2] Create integration test for selected text query in Backend/tests/integration/test_agent_retrieval.py (verify answer strictly from selection, correct refusal when answer not in selection)

### Implementation for User Story 2

- [X] T031 [US2] Implement selected text detection in Backend/agent_rag.py prepare_context() (check if request.selected_text is not None/empty)
- [X] T032 [US2] Implement selected text path in Backend/agent_rag.py (skip Cohere embed, skip Qdrant retrieval, skip validation, create AgentContext with selected_text)
- [X] T033 [US2] Update agent generation in Backend/agent_rag.py (handle selected text context type, source_metadata = [{"chunk_id": "selected_text"}])
- [X] T034 [US2] Update source attribution in Backend/agent_rag.py (map "selected_text" to ChunkReference with null metadata: page=None, chapter=None, section=None)
- [X] T035 [US2] Update POST /ask endpoint in Backend/api.py (validate selected_text max length 10000 chars, handle empty question with selected text)
- [X] T036 [US2] Add logging for selected text queries in Backend/agent_rag.py (log context_type="selected_text", skip retrieval quality)

**Checkpoint**: At this point, User Stories 1 AND 2 both work independently - users can ask standard questions OR about selected text

---

## Phase 5: User Story 3 - Refuse Out-of-Scope Questions (Priority: P1)

**Goal**: Ensure agent refuses questions not about book content to maintain 0% hallucination rate and user trust

**Independent Test**: Submit out-of-scope questions (weather, current events, unrelated topics) → verify agent responds with exact refusal message without attempting answer

### Tests for User Story 3

- [X] T037 [P] [US3] Expand hallucination test suite in Backend/tests/integration/test_hallucination_prevention.py (add 25 adversarial out-of-scope questions, verify exact refusal message)
- [X] T038 [P] [US3] Create refusal accuracy test in Backend/tests/integration/test_hallucination_prevention.py (verify 100% refusal rate for out-of-scope per SC-004)

### Implementation for User Story 3


- [X] T039 [US3] Enhance system prompt in Backend/agent_rag.py (add explicit out-of-scope handling: "If question is not about the book, respond with refusal message")
- [X] T040 [US3] Add few-shot examples to system prompt in Backend/agent_rag.py (demonstrate correct refusal for out-of-scope questions)
- [X] T041 [US3] Validate refusal message format in Backend/agent_rag.py (ensure agent returns exact message: "This information is not available in the book.")
- [X] T042 [US3] Add empty question validation in Backend/api.py (if question is empty/whitespace, return 400 with message: "Please provide a question about the book.")
- [X] T043 [US3] Test refusal message consistency in Backend/tests/integration/test_hallucination_prevention.py (all refusals must use exact same message)

**Checkpoint**: At this point, User Stories 1, 2, AND 3 all work independently - agent handles standard queries, selected text queries, AND correctly refuses out-of-scope questions

---

## Phase 6: User Story 4 - Provide Short, Clear, Factual Answers (Priority: P2) ✅ COMPLETED

**Goal**: Optimize answer quality for user experience - concise (1-5 sentences), direct, factual responses

**Independent Test**: Submit diverse questions → measure answer length, clarity rating, factual accuracy against source chunks using human evaluation

### Tests for User Story 4

- [X] T044 [P] [US4] Create answer quality test in Backend/tests/integration/test_answer_quality.py (verify answers are 1-5 sentences, no opinions, grounded in source chunks)
- [X] T045 [P] [US4] Create performance test in Backend/tests/integration/test_performance.py (measure end-to-end latency for 100 requests, verify <5s per SC-001)

### Implementation for User Story 4

- [X] T046 [US4] Refine system prompt in Backend/agent_rag.py (add length constraint: "Keep answers short (1-5 sentences)", emphasize direct addressing of question)
- [X] T047 [US4] Add output format specification to system prompt in Backend/agent_rag.py (specify sentence count limit, no elaboration)
- [X] T048 [US4] Implement answer post-processing in Backend/agent_rag.py (trim excessive length if needed, validate no opinions/interpretations leaked)
- [X] T049 [US4] Add latency monitoring in Backend/agent_rag.py (log breakdown: embedding time, retrieval time, validation time, generation time)
- [X] T050 [US4] Optimize for <5s latency in Backend/agent_rag.py (use async/await for I/O operations: Cohere, Qdrant, Gemini calls)
- [X] T051 [US4] Add timeout configurations in Backend/agent_rag.py (10s for Qdrant, 15s for Gemini, handle timeouts gracefully)

**Checkpoint**: All user stories (US1-US4) are now complete and independently functional - agent provides high-quality, grounded, concise answers

---

## Phase 7: Polish & Cross-Cutting Concerns ✅ COMPLETED

**Purpose**: Improvements that affect multiple user stories and production readiness

- [X] T052 [P] Create comprehensive unit tests in Backend/tests/unit/test_agent_rag.py (test AgentOrchestrator initialization, system prompt construction, context routing logic)
- [X] T053 [P] Add edge case tests in Backend/tests/integration/test_agent_retrieval.py (empty Qdrant results, contradictory chunks, malformed validation response)
- [X] T054 [P] Validate all 100 hallucination test cases pass in Backend/tests/integration/test_hallucination_prevention.py (verify 0% hallucination rate per SC-003)
- [X] T055 [P] Run full test suite with coverage report (pytest Backend/tests/ -v --cov=Backend --cov-report=html, target >80% coverage)
- [X] T056 [P] Update Backend/README.md with /ask endpoint usage examples (curl commands, request/response samples)
- [X] T057 [P] Add API documentation to Backend/api.py (FastAPI automatic docs at /docs endpoint, verify OpenAPI spec matches contracts/ask_endpoint.yaml)
- [X] T058 Code review for security vulnerabilities (no secrets in code, input validation complete, error messages don't leak sensitive info)
- [X] T059 Performance profiling and optimization (identify bottlenecks, optimize slowest operations, verify p95 latency <5s)
- [X] T060 [P] Run quickstart.md validation (follow all examples in quickstart.md, verify instructions work end-to-end)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - US1, US2, US3 are all P1 and CAN proceed in parallel (different aspects)
  - US4 (P2) depends on US1-US3 for answer quality optimization
- **Polish (Phase 7)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational - Implements core RAG flow (retrieval → validation → generation)
- **User Story 2 (P1)**: Can start after Foundational - Implements selected text override (parallel with US1, different code paths)
- **User Story 3 (P1)**: Can start after Foundational - Enhances US1 with refusal handling (can implement in parallel, touches system prompt + validation)
- **User Story 4 (P2)**: Depends on US1 completion - Optimizes answer quality and performance (refinement of US1 output)

**Recommended Sequence for Single Developer**:
1. Complete Setup → Foundational (T001-T015)
2. US1 core flow (T016-T028) - Get basic Q&A working
3. US2 selected text (T029-T036) - Add override feature
4. US3 refusal handling (T037-T043) - Harden against hallucinations
5. US4 quality optimization (T044-T051) - Polish user experience
6. Polish & production readiness (T052-T060)

**Parallel Team Strategy** (if 3+ developers available):
1. Team completes Setup + Foundational together (T001-T015)
2. Once Foundational done:
   - Developer A: US1 (T016-T028)
   - Developer B: US2 (T029-T036) in parallel
   - Developer C: US3 tests (T037-T038) in parallel
3. After US1 complete:
   - Developer C: US3 implementation (T039-T043, depends on US1 system prompt)
   - Developer A: US4 (T044-T051, refines US1 output)
4. All join for Polish phase (T052-T060)

### Within Each User Story

- Tests MUST be written and FAIL before implementation (TDD approach for 0% hallucination guarantee)
- Models (Phase 2) before services (agent_rag.py)
- Core implementation before error handling
- Error handling before logging
- Story complete and tested before moving to next priority

### Parallel Opportunities

**Setup Phase (Phase 1)**:
- T005, T006, T007 (create test directories) can run in parallel

**Foundational Phase (Phase 2)**:
- T008, T009, T010 (Pydantic models) can be written in parallel (same file, different classes)
- T014, T015 (verify integrations) can run in parallel

**User Story 1 (Phase 3)**:
- T016, T017, T018 (all tests) can be written in parallel (different test files)

**User Story 2 (Phase 4)**:
- T029, T030 (tests) can be written in parallel (different test files)
- US2 entire phase can run in parallel with US1 (different code paths in agent_rag.py)

**User Story 3 (Phase 5)**:
- T037, T038 (tests) can be written in parallel (same test file, different test cases)
- US3 tests (T037-T038) can run in parallel with US1/US2 implementation

**User Story 4 (Phase 6)**:
- T044, T045 (tests) can be written in parallel (different test files)

**Polish Phase (Phase 7)**:
- T052, T053, T054, T055, T056, T057, T060 (all [P] tasks) can run in parallel (different files/purposes)

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together:
Task: "Contract test for POST /ask endpoint in Backend/tests/contract/test_ask_endpoint.py"
Task: "Integration test for standard query flow in Backend/tests/integration/test_agent_retrieval.py"
Task: "Hallucination prevention test suite in Backend/tests/integration/test_hallucination_prevention.py"

# After tests written, implement core flow sequentially:
Task: "Query embedding in Backend/agent_rag.py" (T019)
Task: "Qdrant retrieval in Backend/agent_rag.py" (T020)
Task: "Retrieval validation in Backend/agent_rag.py" (T021)
# ... continue sequentially through T028
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T007)
2. Complete Phase 2: Foundational (T008-T015) - CRITICAL foundation
3. Complete Phase 3: User Story 1 (T016-T028) - Core Q&A with retrieval
4. **STOP and VALIDATE**:
   - Run hallucination test suite (T018) - must achieve 0% hallucination
   - Test end-to-end with real questions
   - Verify <5s latency
5. Deploy/demo if ready - this is a functional MVP!

**MVP Scope**: Users can ask questions about the book and receive accurate, grounded answers with source references. Hallucination rate is 0%.

### Incremental Delivery

1. **Foundation** (Setup + Foundational) → Core infrastructure ready
2. **MVP: US1** → Test independently → Deploy (basic Q&A working)
3. **+ US2** → Test independently → Deploy (now with selected text feature)
4. **+ US3** → Test independently → Deploy (hardened against out-of-scope)
5. **+ US4** → Test independently → Deploy (optimized quality and performance)
6. **Polish** → Production-ready

Each increment adds value without breaking previous functionality.

### Parallel Team Strategy

With 3 developers:

1. **Phase 1-2 (Together)**: All devs complete Setup + Foundational (T001-T015)
2. **Phase 3-5 (Parallel)**:
   - Dev A: US1 core flow (T016-T028)
   - Dev B: US2 selected text (T029-T036) - independent of US1 implementation
   - Dev C: US3 tests + hallucination prevention (T037-T043) - can test US1 as it develops
3. **Phase 6 (Sequential)**: US4 refines US1, so Dev A or C handles after US1 done
4. **Phase 7 (Parallel)**: All devs handle polish tasks (T052-T060)

---

## Notes

- **[P] tasks**: Different files, no dependencies - can run in parallel
- **[Story] labels**: Map tasks to specific user stories for traceability and independent testing
- **Test-First Approach**: Critical for 0% hallucination requirement (SC-003) - write tests, ensure they fail, then implement
- **Error Handling**: Comprehensive error matrix in research.md - each integration point has explicit handling
- **Performance Target**: <5 seconds end-to-end (SC-001) - async I/O and timeouts are essential
- **System Prompt**: Central to hallucination prevention - multi-layered constraints with few-shot examples
- **Selected Text Priority**: Absolute priority routing at retrieval layer - skip Qdrant entirely when present
- **Validation Integration**: Call 003-retrieval-validation after retrieval, before generation - prevents bad answers
- **Independent Stories**: Each user story should be completable and testable on its own
- **Commit Strategy**: Commit after each task or logical group of related tasks
- **Stop at Checkpoints**: Validate each story independently before proceeding to next
- **Avoid**: Vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Task Count Summary

- **Setup**: 7 tasks (T001-T007)
- **Foundational**: 8 tasks (T008-T015)
- **User Story 1** (P1): 13 tasks (T016-T028) - 3 tests + 10 implementation
- **User Story 2** (P1): 8 tasks (T029-T036) - 2 tests + 6 implementation
- **User Story 3** (P1): 7 tasks (T037-T043) - 2 tests + 5 implementation
- **User Story 4** (P2): 8 tasks (T044-T051) - 2 tests + 6 implementation
- **Polish**: 9 tasks (T052-T060)

**Total**: 60 tasks

**Parallel Opportunities**: 18 tasks marked [P] across all phases

**Independent Test Criteria**:
- US1: Submit question with known answer → verify grounded response with sources
- US2: Provide selected text + question → verify answer only from selection
- US3: Submit out-of-scope question → verify exact refusal message
- US4: Measure answer quality (length, clarity, accuracy) across 100 questions

**Suggested MVP Scope**: Complete through User Story 1 (T001-T028) = 28 tasks for a functional MVP with 0% hallucination rate.
