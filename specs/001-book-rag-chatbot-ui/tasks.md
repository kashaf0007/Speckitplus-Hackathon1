---
description: "Implementation tasks for Book RAG Chatbot UI feature"
---

# Tasks: Book RAG Chatbot UI

**Input**: Design documents from `/specs/001-book-rag-chatbot-ui/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/

**Tests**: Tests are NOT explicitly requested in the specification, so no test tasks are included. Focus is on implementation only.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: `my-book/src/` (Docusaurus React application)
- **Backend**: `Backend/` (Existing FastAPI - NO CHANGES REQUIRED)
- All paths are relative to repository root

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for chatbot UI

- [X] T001 Create TypeScript types file at my-book/src/types/chatbot.ts with all interfaces (ChatMessage, SourceReference, ChunkReference, SelectedTextContext, AskRequest, AskResponse, ChatbotState)
- [X] T002 [P] Create chatbot components directory structure: my-book/src/components/ChatbotUI/ with subdirectories
- [X] T003 [P] Create hooks directory at my-book/src/hooks/
- [X] T004 [P] Create services directory at my-book/src/services/
- [X] T005 [P] Create CSS modules file at my-book/src/components/ChatbotUI/styles.module.css with Docusaurus theme variables
- [X] T006 Swizzle Docusaurus Root component: run `npm run swizzle @docusaurus/theme-classic Root -- --wrap` in my-book/ to create my-book/src/theme/Root.tsx

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T007 Implement chatbotApi service in my-book/src/services/chatbotApi.ts with askQuestion function using Fetch API, 5-second AbortController timeout, error classification (network, 400, 500)
- [X] T008 Implement useBackendApi hook in my-book/src/hooks/useBackendApi.ts wrapping chatbotApi service with error handling and loading state
- [X] T009 Implement ChatbotContext and ChatbotProvider in my-book/src/hooks/useChatbot.ts with React Context API (state: messages, loading, error, selectedText; actions: sendMessage, clearError, setSelectedText)
- [X] T010 Implement Root.tsx wrapper in my-book/src/theme/Root.tsx wrapping children with ChatbotProvider to enable global state across all pages
- [X] T011 Implement LoadingIndicator component in my-book/src/components/ChatbotUI/LoadingIndicator.tsx with inline spinner and "Searching..." text
- [X] T012 Implement ErrorMessage component in my-book/src/components/ChatbotUI/ErrorMessage.tsx with user-friendly error display and dismiss button

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Basic Question and Answer (Priority: P1) 🎯 MVP ✅ COMPLETED

**Goal**: Enable users to ask questions about book content and receive accurate answers with source references

**Independent Test**: Embed chatbot UI in book page, ask question (e.g., "What is the main theme of Chapter 3?"), verify system returns relevant answer with source citations from book content

### Implementation for User Story 1

- [X] T013 [P] [US1] Implement ChatbotToggle component in my-book/src/components/ChatbotUI/ChatbotToggle.tsx with floating 💬 icon, fixed bottom-right position, onClick to toggle panel visibility
- [X] T014 [P] [US1] Implement ChatbotPanel component in my-book/src/components/ChatbotUI/ChatbotPanel.tsx with modal container, dismissible overlay, max-width 400px, bottom-right positioning
- [X] T015 [P] [US1] Implement MessageList component in my-book/src/components/ChatbotUI/MessageList.tsx to render conversation history (user and assistant messages) with scrollable area
- [X] T016 [P] [US1] Implement QueryInput component in my-book/src/components/ChatbotUI/QueryInput.tsx with textarea input, Send button, empty query validation (disable button when input is blank)
- [X] T017 [US1] Implement AnswerDisplay component in my-book/src/components/ChatbotUI/AnswerDisplay.tsx to render assistant message with answer text
- [X] T018 [US1] Implement SourcesList component in my-book/src/components/ChatbotUI/SourcesList.tsx to render clickable source references with chapter/section links using @docusaurus/router useHistory for navigation
- [X] T019 [US1] Implement ChunkDisplay component in my-book/src/components/ChatbotUI/ChunkDisplay.tsx to render matched content chunks with text preview and source location (chapter, page, section)
- [X] T020 [US1] Integrate ChatbotToggle and ChatbotPanel into Root.tsx to render on all book pages, connect to ChatbotContext, handle panel open/close state
- [X] T021 [US1] Implement sendMessage action in useChatbot.ts to call askQuestion from useBackendApi, create user ChatMessage, handle response, create assistant ChatMessage with sources and chunks
- [X] T022 [US1] Add loading indicator display in QueryInput component when loading state is true (disable Send button, show LoadingIndicator below input)
- [X] T023 [US1] Add error message display in ChatbotPanel component when error state is not null (render ErrorMessage component with error text)
- [X] T024 [US1] Style all components using styles.module.css with Docusaurus theme variables (--ifm-color-primary, --ifm-background-color, --ifm-color-emphasis-300)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently. Users can ask questions, see loading indicators, receive answers with sources and chunks, click source links to navigate to book sections.

---

## Phase 4: User Story 2 - Context-Aware Query with Selected Text (Priority: P1) ✅ COMPLETED

**Goal**: Enable users to highlight text in the book and ask questions specifically about that selection for precise answers

**Independent Test**: Select a paragraph in the book, open chatbot, ask "Explain this passage", verify system uses selected text as primary context for generating answer

### Implementation for User Story 2

- [X] T025 [US2] Implement useTextSelection hook in my-book/src/hooks/useTextSelection.ts with window.getSelection() API, mouseup event listener, capture selection on mouseup, trim whitespace, truncate to 2000 characters
- [X] T026 [US2] Integrate useTextSelection hook into ChatbotProvider in useChatbot.ts to update selectedText state when user selects text
- [X] T027 [US2] Update QueryInput component to display selected text badge when selectedText is not null (show truncated preview with "Selected text: {preview}..." and clear button)
- [X] T028 [US2] Update sendMessage action in useChatbot.ts to include selectedText in AskRequest when selectedText state is not null (pass as selected_text parameter)
- [X] T029 [US2] Update AnswerDisplay component to show indicator when answer is based on selected text (e.g., "Answer based on selected text" badge)
- [X] T030 [US2] Handle selected text in SourcesList component to display "Selected Text" as source when chunk_id is 'selected_text'
- [X] T031 [US2] Add clear selection button in QueryInput selected text badge to reset selectedText state to null when clicked

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently. Users can ask general questions (US1) or select text and ask questions about the selection (US2).

---

## Phase 5: User Story 3 - Handle No Results Gracefully (Priority: P2) ✅ COMPLETED

**Goal**: Provide clear feedback when questions cannot be answered from book content so users understand system limitations

**Independent Test**: Ask question about content not covered in book (e.g., "What is quantum physics?" in a history book), verify system displays helpful message like "I couldn't find relevant information about this in the book."

### Implementation for User Story 3

- [X] T032 [US3] Update AnswerDisplay component to detect refusal messages (check if answer contains "not available in the book") and render with special styling (e.g., info icon, muted color)
- [X] T033 [US3] Update ErrorMessage component to handle empty results case (when answer is refusal message) with helpful suggestions ("Try using different keywords" or "This topic may not be covered in the book")
- [X] T034 [US3] Update sendMessage action in useChatbot.ts to check if backend response has grounded=false and display refusal message without showing sources/chunks (empty arrays)

**Checkpoint**: At this point, User Stories 1, 2, AND 3 should all work independently. System gracefully handles empty results with user-friendly messages.

---

## Phase 6: User Story 4 - View Answer Sources and Matched Chunks (Priority: P2) ✅ COMPLETED

**Goal**: Show which parts of the book were used to generate each answer for transparency and deeper exploration

**Independent Test**: Ask any question, receive answer, verify interface displays matched content chunks and source references (chapter/page numbers) used to generate response

### Implementation for User Story 4

- [X] T035 [US4] Update AnswerDisplay component to always render SourcesList and ChunkDisplay components below answer text when sources and chunks arrays are not empty
- [X] T036 [US4] Implement source navigation in SourcesList component using @docusaurus/router useHistory to construct paths from ChunkReference.chapter and .section (e.g., /docs/chapter-3#section-title), use Docusaurus slugify for anchor links
- [X] T037 [US4] Update ChunkDisplay component to show full chunk text (up to 2000 chars), page number, chapter, and section in expandable/collapsible format for better readability
- [X] T038 [US4] Add "View in book" button for each chunk in ChunkDisplay that navigates to the source location using constructed path and anchor
- [X] T039 [US4] Style SourcesList with numbered list format (1. Chapter 3: ROS 2 Overview, 2. Chapter 3: Security Features) for easy reference

**Checkpoint**: At this point, User Stories 1-4 should all work independently. Users can view sources and matched chunks, click source links to navigate to book sections.

---

## Phase 7: User Story 5 - Continuous Conversation Experience (Priority: P3) ✅ COMPLETED

**Goal**: Keep chatbot accessible throughout reading session so users can ask multiple questions without interruption

**Independent Test**: Embed chatbot as persistent UI element (e.g., sidebar or overlay) that remains available as user navigates through book pages

### Implementation for User Story 5

- [X] T040 [US5] Update ChatbotPanel component to persist conversation history in ChatbotContext state across page navigation (already handled by Root.tsx wrapper, verify it works)
- [X] T041 [US5] Implement minimize/maximize functionality in ChatbotPanel with minimize button that hides panel content but keeps toggle icon visible, clicking toggle reopens panel with history preserved
- [X] T042 [US5] Update MessageList component to display conversation history in chronological order (oldest first) with auto-scroll to latest message when new message added
- [X] T043 [US5] Add conversation history limit in useChatbot.ts to truncate messages array to max 100 messages to prevent memory bloat (remove oldest messages when limit exceeded)
- [X] T044 [US5] Implement panel position persistence using sessionStorage to remember panel open/closed state across page navigation (restore state on mount)

**Checkpoint**: All user stories should now be independently functional. Chatbot remains accessible throughout reading session with conversation history preserved.

---

## Phase 8: Polish & Cross-Cutting Concerns ✅ COMPLETED

**Purpose**: Improvements that affect multiple user stories

- [X] T045 [P] Add responsive design styles in styles.module.css for mobile/tablet screens (smaller panel width on mobile, full-screen on small devices)
- [X] T046 [P] Add keyboard accessibility to ChatbotToggle (Enter/Space to open panel), QueryInput (Enter to send, Shift+Enter for newline), and panel close button (Escape to close)
- [X] T047 [P] Add loading state visual feedback in MessageList showing "..." animation while waiting for assistant response
- [X] T048 [P] Add empty state message in MessageList when messages array is empty (e.g., "Ask a question about the book to get started")
- [X] T049 Code cleanup: Remove unused imports, fix TypeScript strict mode errors, ensure all components follow consistent naming conventions
- [X] T050 Run manual testing checklist from quickstart.md (all 8 tests: basic Q&A, selected text, empty query validation, no results, error handling, source navigation, panel minimize, navigation persistence)
- [X] T051 Verify all functional requirements (FR-001 to FR-013) and success criteria (SC-001 to SC-010) are met per spec.md acceptance scenarios
- [X] T052 [P] Update README.md or quickstart.md with screenshots of chatbot UI in action (toggle icon, panel open, answer with sources)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - Core chatbot functionality
  - User Story 2 (P1): Can start after Foundational - Text selection feature (can work in parallel with US1 if staffed)
  - User Story 3 (P2): Depends on US1 (extends AnswerDisplay component)
  - User Story 4 (P2): Depends on US1 (extends AnswerDisplay, SourcesList, ChunkDisplay)
  - User Story 5 (P3): Depends on US1 (extends ChatbotPanel and state management)
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories. This is the MVP.
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Independent of US1 but typically integrated after US1 for coherent UX
- **User Story 3 (P2)**: Depends on US1 completion - Extends AnswerDisplay and error handling from US1
- **User Story 4 (P2)**: Depends on US1 completion - Extends SourcesList and ChunkDisplay from US1
- **User Story 5 (P3)**: Depends on US1 completion - Extends ChatbotPanel and state management from US1

### Within Each User Story

- Components marked [P] can be built in parallel (different files, no dependencies)
- Integration tasks depend on component completion
- State management updates depend on related components being built
- Styling tasks depend on all components in that story being implemented

### Parallel Opportunities

- All Setup tasks (T002-T005) marked [P] can run in parallel after T001
- All Foundational tasks (T007-T012) can run in parallel (different files)
- Within US1: T013-T016 (components) can run in parallel, then T017-T019 can run in parallel
- Within US2: T025-T026 can run in parallel, then T027-T031 can run sequentially
- User Story 1 and User Story 2 can be worked on in parallel by different team members after Foundational phase
- All Polish tasks marked [P] (T045-T048, T052) can run in parallel

---

## Parallel Example: User Story 1 (MVP)

```bash
# After Foundational phase complete, launch these US1 components in parallel:
Task T013: "Implement ChatbotToggle component in my-book/src/components/ChatbotUI/ChatbotToggle.tsx"
Task T014: "Implement ChatbotPanel component in my-book/src/components/ChatbotUI/ChatbotPanel.tsx"
Task T015: "Implement MessageList component in my-book/src/components/ChatbotUI/MessageList.tsx"
Task T016: "Implement QueryInput component in my-book/src/components/ChatbotUI/QueryInput.tsx"

# After T013-T016 complete, launch these US1 display components in parallel:
Task T017: "Implement AnswerDisplay component in my-book/src/components/ChatbotUI/AnswerDisplay.tsx"
Task T018: "Implement SourcesList component in my-book/src/components/ChatbotUI/SourcesList.tsx"
Task T019: "Implement ChunkDisplay component in my-book/src/components/ChatbotUI/ChunkDisplay.tsx"
```

---

## Parallel Example: User Story 2 (Selected Text)

```bash
# After Foundational phase complete, launch these US2 tasks in parallel:
Task T025: "Implement useTextSelection hook in my-book/src/hooks/useTextSelection.ts"
Task T026: "Integrate useTextSelection hook into ChatbotProvider in useChatbot.ts"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T006) - ~30 minutes
2. Complete Phase 2: Foundational (T007-T012) - ~2 hours (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (T013-T024) - ~4-6 hours
4. **STOP and VALIDATE**: Test User Story 1 independently using quickstart.md test checklist
5. Deploy/demo if ready - this is the MVP!

**MVP Success Criteria**:
- Users can open chatbot (💬 icon)
- Users can ask questions and receive answers
- Loading indicator shows while processing
- Answers display with sources and matched chunks
- Source links navigate to book sections
- Error messages display for network/backend failures
- Empty query submission prevented

### Incremental Delivery

1. **Week 1**: Setup + Foundational → Foundation ready
2. **Week 1-2**: Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. **Week 2**: Add User Story 2 → Test independently → Deploy/Demo (MVP + Selected Text)
4. **Week 3**: Add User Stories 3 & 4 → Test independently → Deploy/Demo (Enhanced UX)
5. **Week 4**: Add User Story 5 + Polish → Test independently → Deploy/Demo (Full Feature)

Each story adds value without breaking previous stories.

### Parallel Team Strategy

With 2-3 developers:

1. **Team completes Setup + Foundational together** (critical path)
2. Once Foundational is done:
   - **Developer A**: User Story 1 (MVP - highest priority)
   - **Developer B**: User Story 2 (Selected text - can start in parallel)
3. After US1 & US2 complete:
   - **Developer A**: User Story 3 (Error handling)
   - **Developer B**: User Story 4 (Sources/chunks display)
   - **Developer C**: User Story 5 (Conversation history)
4. All developers: Polish phase together

---

## Notes

- **[P]** tasks = different files, no dependencies, can run in parallel
- **[Story]** label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- **Backend unchanged**: All tasks are frontend-only, Backend/api.py already has /ask endpoint
- Commit after each task or logical group (e.g., after each component)
- Stop at any checkpoint to validate story independently
- **No tests included**: Tests not explicitly requested in spec, focus is on implementation
- **Zero new dependencies**: All tasks use native browser APIs and existing Docusaurus/React features
- Verify with quickstart.md manual testing checklist after each story
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Task Count Summary

- **Phase 1 (Setup)**: 6 tasks
- **Phase 2 (Foundational)**: 6 tasks (CRITICAL - blocks all stories)
- **Phase 3 (US1 - MVP)**: 12 tasks
- **Phase 4 (US2)**: 7 tasks
- **Phase 5 (US3)**: 3 tasks
- **Phase 6 (US4)**: 5 tasks
- **Phase 7 (US5)**: 5 tasks
- **Phase 8 (Polish)**: 8 tasks

**Total: 52 tasks**

**Parallelizable tasks**: 23 tasks marked [P] (44% can run in parallel)

**MVP scope**: Phase 1 + Phase 2 + Phase 3 (24 tasks total for basic chatbot Q&A)

**Estimated effort**:
- MVP (US1): 8-10 hours for single developer
- Full feature (US1-US5 + Polish): 20-30 hours for single developer
- With 2 developers working in parallel: 12-18 hours to complete all stories
