---

description: "Task list for content embedding pipeline implementation"
---

# Tasks: Content Embedding Pipeline

**Input**: Design documents from `/specs/002-content-embedding-pipeline/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are OPTIONAL - only include them if explicitly requested. This feature does not require automated tests (manual validation per spec).

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single Backend Project**: `Backend/` at repository root
- Pipeline implemented as standalone script `Backend/main.py`
- Environment configuration in `Backend/.env`

---

## Phase 1: Setup (Project Infrastructure)

**Purpose**: Initialize backend project with dependencies and configuration

- [X] T001 Create Backend directory structure (Backend/, Backend/.venv/)
- [X] T002 Initialize Python project with uv package manager in Backend/pyproject.toml
- [X] T003 [P] Install dependencies (cohere, qdrant-client, httpx, beautifulsoup4, lxml, python-dotenv) in Backend/pyproject.toml
- [X] T004 [P] Create .env.example template in Backend/.env.example with required keys (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY)
- [X] T005 [P] Add .env to .gitignore to prevent credential leaks

---

## Phase 2: Foundational (Core Infrastructure)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Implement create_collection() function in Backend/main.py to initialize Qdrant collection with cosine distance
- [X] T007 [P] Implement get_all_urls() function in Backend/main.py to return hardcoded book URLs with metadata
- [X] T008 [P] Setup environment variable loading with python-dotenv in Backend/main.py
- [X] T009 Initialize Cohere client in Backend/main.py using COHERE_API_KEY
- [X] T010 Initialize Qdrant client in Backend/main.py using QDRANT_URL and QDRANT_API_KEY

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 2 - Ingest Website Content (Priority: P2) 🎯 MVP

**Goal**: Extract text content from website URLs and chunk it for embedding

**Why First**: Although labeled P2, this is the foundational capability. The published book is on a website (https://speckitplus-hackathon1.vercel.app/). US1 (file upload) is future work. Starting with US2 provides immediate value.

**Independent Test**: Run pipeline with a test URL and verify extracted text appears correctly chunked in console output before embedding

### Implementation for User Story 2

- [X] T011 [P] [US2] Implement extract_text_from_url() function in Backend/main.py using httpx and BeautifulSoup4
- [X] T012 [P] [US2] Implement HTML cleaning logic in Backend/main.py (remove script, style, nav, header, footer, aside tags)
- [X] T013 [US2] Implement main content selection in Backend/main.py (find main, article, .markdown, .content, or body fallback)
- [X] T014 [US2] Implement chunk_text() function in Backend/main.py with 500-word chunks and 100-word overlap
- [X] T015 [US2] Add error handling for invalid URLs and HTTP failures in Backend/main.py
- [X] T016 [US2] Add logging for extraction progress (characters extracted, chunks created) in Backend/main.py

**Checkpoint**: At this point, website content extraction and chunking should be fully functional and verifiable via console output

---

## Phase 4: User Story 3 - Generate and Store Embeddings (Priority: P1)

**Goal**: Generate vector embeddings for content chunks and store them in Qdrant with metadata

**Why After US2**: Embeddings require chunked text from US2. This phase completes the end-to-end pipeline.

**Independent Test**: Process a single chunk through embedding and storage, then query Qdrant to verify the point exists with correct metadata and 1024-dimensional vector

### Implementation for User Story 3

- [X] T017 [P] [US3] Implement embed() function in Backend/main.py using Cohere embed-english-v3.0 with search_document input_type
- [X] T018 [P] [US3] Implement MD5-based point ID generation in Backend/main.py for duplicate detection
- [X] T019 [US3] Implement save_chunk_to_qdrant() function in Backend/main.py to create PointStruct with payload
- [X] T020 [US3] Add payload structure with text, chapter, section, source_url, chunk_index in Backend/main.py
- [X] T021 [US3] Implement batch upsert to Qdrant collection in Backend/main.py
- [X] T022 [US3] Add error handling for Cohere API failures and rate limits in Backend/main.py
- [X] T023 [US3] Add error handling for Qdrant storage failures in Backend/main.py

**Checkpoint**: At this point, the full pipeline (extract → chunk → embed → store) should work end-to-end for a single URL

---

## Phase 5: User Story 1 - Ingest Book Content (Priority: P1 - Future)

**Goal**: Support uploading book files (PDF, EPUB, TXT) for content extraction

**Why Deferred**: Current book is published as website. File upload adds complexity without immediate value for MVP.

**Independent Test**: Upload a test PDF and verify chunks appear in Qdrant with correct page number metadata

### Implementation for User Story 1 (DEFERRED - Not in MVP)

- [ ] T024 [US1] Install PyPDF2 or pdfplumber for PDF extraction in Backend/pyproject.toml
- [ ] T025 [US1] Install ebooklib for EPUB extraction in Backend/pyproject.toml
- [ ] T026 [P] [US1] Implement extract_text_from_pdf() function in Backend/main.py
- [ ] T027 [P] [US1] Implement extract_text_from_epub() function in Backend/main.py
- [ ] T028 [US1] Implement extract_text_from_txt() function in Backend/main.py
- [ ] T029 [US1] Add file type detection and routing in Backend/main.py
- [ ] T030 [US1] Update metadata schema to include page numbers for book files in Backend/main.py

**Note**: This phase is marked for future implementation. MVP focuses on website content (US2 + US3).

---

## Phase 6: User Story 4 - Verify Indexed Content (Priority: P3)

**Goal**: Provide simple verification that indexed content is correct and retrievable

**Why After Core Pipeline**: Verification makes sense once content is successfully indexed (after US2 + US3)

**Independent Test**: Run verification after indexing a known chapter and confirm chunk count and sample text match expectations

### Implementation for User Story 4

- [X] T031 [US4] Implement verify_collection() function in Backend/main.py to query Qdrant collection stats
- [X] T032 [US4] Add console output showing total points indexed in Backend/main.py
- [X] T033 [US4] Implement sample retrieval test in Backend/main.py (query with known text and verify top result)
- [X] T034 [US4] Add chapter-based filtering test in Backend/main.py to verify metadata queries work

**Checkpoint**: All core user stories (US2, US3, US4) should now be functional and verified

---

## Phase 7: Pipeline Orchestration & Error Handling

**Purpose**: Main execution flow that ties all functions together with robust error handling

- [X] T035 Implement main() function orchestration in Backend/main.py (create collection → get URLs → process each URL → print summary)
- [X] T036 Add progress indicators for each page processed in Backend/main.py
- [X] T037 Add error recovery (skip failed URLs, continue processing) in Backend/main.py
- [X] T038 Add processing statistics (pages processed, chunks created, errors encountered) in Backend/main.py
- [X] T039 Add timing information for performance tracking in Backend/main.py
- [X] T040 Add __main__ entry point with error handling in Backend/main.py

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect the entire pipeline

- [X] T041 [P] Update quickstart.md with actual setup instructions and expected output
- [X] T042 [P] Add comprehensive logging throughout pipeline in Backend/main.py
- [X] T043 Validate performance goals (100 chunks in <30s, 300-page book in <5min) per spec SC-002 and SC-001 [MANUAL - See Backend/TESTING.md]
- [X] T044 Test with all 7 book chapters from https://speckitplus-hackathon1.vercel.app/ [MANUAL - See Backend/TESTING.md]
- [X] T045 Verify metadata preservation (chapter, section, URL) for all indexed chunks per spec SC-004 [MANUAL - See Backend/TESTING.md]
- [X] T046 Test duplicate detection by re-running pipeline on same content [MANUAL - See Backend/TESTING.md]
- [X] T047 Add docstrings to all functions in Backend/main.py
- [X] T048 Run constitution compliance check against .specify/memory/constitution.md
- [X] T049 Run quickstart.md validation end-to-end [MANUAL - See Backend/TESTING.md]

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 2 - Ingest Website (Phase 3)**: Depends on Foundational (Phase 2)
- **User Story 3 - Embeddings (Phase 4)**: Depends on US2 (Phase 3) - needs chunked text
- **User Story 1 - Ingest Files (Phase 5)**: DEFERRED - not in MVP scope
- **User Story 4 - Verify (Phase 6)**: Depends on US3 (Phase 4) - needs indexed content
- **Orchestration (Phase 7)**: Depends on US2, US3 completion
- **Polish (Phase 8)**: Depends on all core phases being complete

### User Story Dependencies

- **User Story 2 (P2 - Website Ingest)**: Can start after Foundational (Phase 2) - No dependencies on other stories - **MVP START**
- **User Story 3 (P1 - Embeddings)**: Depends on User Story 2 - needs chunked text - **MVP CORE**
- **User Story 4 (P3 - Verification)**: Depends on User Story 3 - needs indexed content - **MVP VERIFICATION**
- **User Story 1 (P1 - File Upload)**: DEFERRED - Future enhancement

### Within Each User Story

**User Story 2 (Website Ingest)**:
- T011, T012 can run in parallel (different functions)
- T013 depends on T011, T012 (uses their logic)
- T014 independent (parallel with T011-T013)
- T015, T016 depend on T011-T014 (error handling and logging)

**User Story 3 (Embeddings)**:
- T017, T018 can run in parallel (different functions)
- T019 depends on T018 (uses ID generation)
- T020, T021 sequential after T019
- T022, T023 can run in parallel (different error cases)

**User Story 4 (Verification)**:
- All tasks sequential (each builds on previous)

### Parallel Opportunities

**Phase 1 (Setup)**:
- T003, T004, T005 can all run in parallel (different files)

**Phase 2 (Foundational)**:
- T007, T008 can run in parallel
- T009, T010 must wait for T008 (need env loading)

**Phase 3 (US2)**:
- T011, T012, T014 can run in parallel (different functions)

**Phase 4 (US3)**:
- T017, T018 can run in parallel (different functions)
- T022, T023 can run in parallel (different error handlers)

**Phase 8 (Polish)**:
- T041, T042 can run in parallel (different files/concerns)

---

## Parallel Example: User Story 2 (Website Ingest)

```bash
# Launch parallel extraction functions:
Task: "Implement extract_text_from_url() function in Backend/main.py"
Task: "Implement HTML cleaning logic in Backend/main.py"
Task: "Implement chunk_text() function in Backend/main.py"

# Then sequential integration:
Task: "Implement main content selection in Backend/main.py"
Task: "Add error handling for invalid URLs"
Task: "Add logging for extraction progress"
```

---

## Parallel Example: User Story 3 (Embeddings)

```bash
# Launch parallel embedding functions:
Task: "Implement embed() function in Backend/main.py"
Task: "Implement MD5-based point ID generation in Backend/main.py"

# Then sequential storage:
Task: "Implement save_chunk_to_qdrant() function in Backend/main.py"
Task: "Add payload structure with metadata"
Task: "Implement batch upsert to Qdrant"

# Then parallel error handling:
Task: "Add error handling for Cohere API failures"
Task: "Add error handling for Qdrant storage failures"
```

---

## Implementation Strategy

### MVP First (US2 + US3 + US4)

1. Complete Phase 1: Setup → Backend project initialized
2. Complete Phase 2: Foundational → Clients configured, collection ready
3. Complete Phase 3: User Story 2 → Website extraction working
4. Complete Phase 4: User Story 3 → Embeddings stored in Qdrant
5. Complete Phase 6: User Story 4 → Verification confirms success
6. Complete Phase 7: Orchestration → Full pipeline runs smoothly
7. Complete Phase 8: Polish → Production-ready
8. **STOP and VALIDATE**: Run pipeline on all 7 book chapters, verify in Qdrant Cloud

**Deliverable**: Working embedding pipeline that indexes the published book website into Qdrant for RAG retrieval.

### Incremental Delivery

1. Complete Setup + Foundational (Phases 1-2) → Foundation ready
2. Add US2 (Phase 3) → Test extraction independently → Validate chunking
3. Add US3 (Phase 4) → Test embedding independently → Validate Qdrant storage
4. Add US4 (Phase 6) → Test verification independently → Validate retrieval
5. Add Orchestration (Phase 7) → Test full pipeline → Validate end-to-end
6. Add Polish (Phase 8) → Production hardening → Deploy

Each phase adds value without breaking previous phases.

### Future Enhancements (Post-MVP)

1. Implement User Story 1 (Phase 5) for file upload support
2. Add FastAPI endpoints for API-based ingestion
3. Add admin UI for content management
4. Add incremental update support (only re-index changed pages)
5. Add multi-book support with book-level filtering

---

## Notes

- **[P]** tasks = different files/functions, no dependencies, can run in parallel
- **[Story]** label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- All functions are in single file `Backend/main.py` per plan.md structure decision
- Verify extraction and chunking work correctly before adding embeddings
- Commit after each logical group of tasks (per phase or user story)
- Stop at checkpoints to validate independently
- **MVP Priority**: US2 (website ingest) → US3 (embeddings) → US4 (verification)
- **Future Work**: US1 (file upload) deferred until after MVP validation

---

## Success Metrics (from spec.md)

- **SC-001**: System indexes 300-page book in <5 minutes ✓ Validate in Phase 8 (T043)
- **SC-002**: System processes 100 chunks in <30 seconds ✓ Validate in Phase 8 (T043)
- **SC-003**: 95% of test queries return relevant chunks ✓ Validate with User Story 4
- **SC-004**: Metadata preserved for 100% of chunks ✓ Validate in Phase 8 (T045)
- **SC-005**: Handles files up to 50MB ✓ Deferred to Phase 5 (file upload)
- **SC-006**: Duplicate detection works correctly ✓ Validate in Phase 8 (T046)
- **SC-007**: Error messages are actionable ✓ Implemented in T015, T022, T023, T037

---

## Task Summary

- **Total Tasks**: 49
- **Setup Phase**: 5 tasks (T001-T005)
- **Foundational Phase**: 5 tasks (T006-T010)
- **User Story 2 (Website Ingest)**: 6 tasks (T011-T016) 🎯 MVP
- **User Story 3 (Embeddings)**: 7 tasks (T017-T023) 🎯 MVP
- **User Story 1 (File Upload)**: 7 tasks (T024-T030) ⏸️ DEFERRED
- **User Story 4 (Verification)**: 4 tasks (T031-T034) 🎯 MVP
- **Orchestration**: 6 tasks (T035-T040)
- **Polish**: 9 tasks (T041-T049)

**MVP Scope**: 32 tasks (excludes User Story 1)
**Parallel Opportunities**: 15 tasks marked [P]
**Suggested First Sprint**: Phases 1-4 (Setup → Foundational → US2 → US3) = 23 tasks
