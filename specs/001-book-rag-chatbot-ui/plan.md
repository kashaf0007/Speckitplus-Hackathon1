# Implementation Plan: Book RAG Chatbot Interface

**Branch**: `001-book-rag-chatbot-ui` | **Date**: 2025-12-18 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-book-rag-chatbot-ui/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Embed an interactive RAG chatbot UI directly into the Docusaurus-published book, allowing readers to ask questions and receive book-grounded answers with source references. The chatbot will appear as a floating message icon (💬) that opens a chat panel, supporting both general queries (with RAG retrieval) and selected-text queries (with context override). The UI will communicate with the existing FastAPI `/ask` endpoint, display answers with sources and matched chunks, handle loading states, and gracefully manage errors and empty results.

## Technical Context

**Language/Version**: TypeScript 5.6, React 19, Python 3.11 (backend)
**Primary Dependencies**:
- Frontend: Docusaurus 3.9.2, React 19, @mdx-js/react 3.0
- Backend: FastAPI (existing), Pydantic (existing)
**Storage**: N/A (stateless UI - conversation history in component state only)
**Testing**: Jest + React Testing Library (frontend unit), Playwright (E2E), pytest (backend - existing)
**Target Platform**: Modern web browsers (Chrome, Firefox, Safari) via Docusaurus static site
**Project Type**: Web application (frontend + backend separation)
**Performance Goals**:
- UI response to user input: <100ms
- Display loading indicator within 100ms of query submission
- Render answer within 200ms of receiving backend response
- Support 3+ concurrent queries without UI blocking
**Constraints**:
- Must not break existing book layout or reading experience
- Must not obscure book content
- Chatbot panel must be dismissible/minimizable
- Text selection capture must work across all book content
- Must handle backend timeouts (5s max)
- Must prevent empty query submission
**Scale/Scope**:
- Single-page Docusaurus site with ~50-100 book pages
- Estimated 10-50 concurrent readers
- Support text selections up to 2000 characters

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the Book RAG Chatbot Constitution (from PHR 002-book-rag-chatbot-constitution):

### Core Principles Compliance

1. **Book-Only Knowledge (NON-NEGOTIABLE)** ✅
   - Status: PASS
   - Implementation: UI displays only answers from backend `/ask` endpoint, which enforces book-only responses
   - Verification: No external API calls from UI; all answers sourced from FastAPI backend with RAG validation

2. **No Hallucination Policy (NON-NEGOTIABLE)** ✅
   - Status: PASS
   - Implementation: UI displays backend refusal messages when `answer` field contains "not available in the book"
   - Verification: Error handling shows user-friendly messages; no client-side answer generation

3. **Selected Text Priority (NON-NEGOTIABLE)** ✅
   - Status: PASS
   - Implementation: UI captures text selection via `window.getSelection()` and sends as `selected_text` parameter
   - Verification: Backend AskRequest model already supports `selected_text` field; UI will populate it when user has active selection

4. **Concise Response Format** ✅
   - Status: PASS
   - Implementation: UI renders backend response as-is; backend already returns 1-5 sentence answers
   - Verification: No client-side answer modification; display logic only

5. **RAG Flow Separation** ✅
   - Status: PASS
   - Implementation: UI sends either `question` only (normal) or `question + selected_text` (override); backend handles routing
   - Verification: Backend AgentOrchestrator already implements two-path logic; UI passes parameters correctly

6. **Technology Stack Compliance** ✅
   - Status: PASS
   - Implementation: React UI in Docusaurus, FastAPI backend (existing), communication via REST /ask endpoint
   - Verification: Matches constitution tech stack (Docusaurus frontend + FastAPI)

7. **Strict Agent Instructions** ✅
   - Status: PASS (Backend responsibility)
   - Implementation: Backend enforces agent instructions; UI displays responses faithfully
   - Verification: UI does not modify or filter backend answers

### Gate Result: **PASS** - All constitution principles satisfied

## Project Structure

### Documentation (this feature)

```text
specs/001-book-rag-chatbot-ui/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── chatbot-ui-api.yaml
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application structure (Docusaurus frontend + FastAPI backend)

my-book/                              # Docusaurus book frontend
├── src/
│   ├── components/
│   │   ├── ChatbotUI/                # NEW: Chatbot UI components
│   │   │   ├── ChatbotPanel.tsx      # Main chat panel component
│   │   │   ├── ChatbotToggle.tsx     # Floating message icon toggle
│   │   │   ├── MessageList.tsx       # Messages display area
│   │   │   ├── QueryInput.tsx        # Input box + send button
│   │   │   ├── AnswerDisplay.tsx     # Answer rendering with sources
│   │   │   ├── SourcesList.tsx       # Clickable source references
│   │   │   ├── ChunkDisplay.tsx      # Matched chunks display
│   │   │   ├── LoadingIndicator.tsx  # Loading spinner
│   │   │   ├── ErrorMessage.tsx      # Error display component
│   │   │   └── styles.module.css     # Component styles
│   │   └── [existing components]
│   ├── hooks/                        # NEW: React hooks
│   │   ├── useChatbot.ts             # Main chatbot logic hook
│   │   ├── useTextSelection.ts       # Text selection capture
│   │   └── useBackendApi.ts          # API communication
│   ├── services/                     # NEW: API service layer
│   │   └── chatbotApi.ts             # FastAPI /ask endpoint client
│   ├── types/                        # NEW: TypeScript types
│   │   └── chatbot.ts                # Request/response types
│   └── theme/
│       └── Root.tsx                  # NEW: Global chatbot wrapper
├── tests/                            # NEW: Frontend tests
│   ├── unit/
│   │   ├── ChatbotPanel.test.tsx
│   │   ├── useChatbot.test.ts
│   │   └── chatbotApi.test.ts
│   └── e2e/
│       └── chatbot-flow.spec.ts      # Playwright E2E tests
└── [existing Docusaurus structure]

Backend/                              # FastAPI backend (EXISTING)
├── api.py                            # /ask endpoint (existing)
├── agent_rag.py                      # RAG orchestrator (existing)
├── models/
│   └── agent_models.py               # AskRequest/AskResponse (existing)
└── tests/                            # Backend tests (existing)
```

**Structure Decision**: Web application structure selected because:
1. Clear frontend/backend separation already exists (my-book/ vs Backend/)
2. Docusaurus is React-based, requires component-based architecture
3. FastAPI backend already implemented with /ask endpoint
4. UI is purely additive - no changes to existing Backend/ code required
5. All new code is in my-book/src/components/ChatbotUI/ and supporting files

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations detected - Constitution Check passed all gates.

## Phase 0: Research & Clarifications

*Purpose: Resolve all "NEEDS CLARIFICATION" items from Technical Context and research best practices*

### Research Topics

1. **Docusaurus Theme Customization**
   - Decision: Use `src/theme/Root.tsx` for global chatbot wrapper (swizzled Root component)
   - Rationale:
     - Root.tsx is the highest-level wrapper in Docusaurus, ensures chatbot is available on all pages
     - Swizzling Root is a documented Docusaurus pattern for adding global UI elements
     - Avoids modifying every page individually
   - Alternatives considered:
     - Layout wrapper: Too low-level, would require swizzling multiple layouts
     - Plugin: Overkill for a single UI component, adds complexity
     - Manual injection: Fragile, breaks with Docusaurus updates

2. **Text Selection Capture in React**
   - Decision: Use `window.getSelection()` API with `useEffect` listener on `mouseup` event
   - Rationale:
     - Browser-native API, no dependencies
     - Works across all Docusaurus content (MDX, HTML, code blocks)
     - Handles edge cases (empty selection, cross-element selection)
   - Alternatives considered:
     - `document.getSelection()`: Identical to window.getSelection() in modern browsers
     - Third-party library (react-text-selection): Adds dependency for simple use case
     - Selection event: Not supported in all browsers

3. **Frontend State Management**
   - Decision: React Context API with `useChatbot` custom hook
   - Rationale:
     - Single chatbot instance across all pages (managed in Root.tsx)
     - Conversation history persists during page navigation
     - No need for Redux/Zustand - simple state shape (messages array, loading bool, error string)
     - Built-in React feature, no dependencies
   - Alternatives considered:
     - Component-level state: Would lose history on page navigation
     - Redux: Overkill for 3-4 state variables
     - LocalStorage: Adds complexity, not needed for session-only history

4. **Backend Communication**
   - Decision: Fetch API with async/await, 5-second timeout
   - Rationale:
     - Native browser API, no dependencies
     - Simple async error handling with try/catch
     - Timeout via AbortController (native API)
     - Matches backend timeout expectations (SC-009: 5s max)
   - Alternatives considered:
     - Axios: Adds 15KB dependency for features we don't need
     - WebSocket: Out of scope (MVP is request/response, not streaming)
     - Server-Sent Events: Overkill for single response per query

5. **Source Reference Navigation**
   - Decision: Use Docusaurus router (`useHistory` from `@docusaurus/router`) + anchor links
   - Rationale:
     - Docusaurus content has structured IDs (auto-generated from headings)
     - Backend returns chapter/section metadata in `ChunkReference.chapter` and `.section`
     - Can construct anchor links: `/docs/chapter-3#section-title`
     - Native Docusaurus navigation, no page reload
   - Alternatives considered:
     - Page reload with hash: Works but jarring UX
     - Scroll-only: Requires knowing DOM structure, fragile
     - No navigation: Fails FR-010 (clickable source references)

6. **UI Component Library**
   - Decision: Pure CSS modules + Docusaurus theme variables
   - Rationale:
     - Docusaurus has built-in CSS modules support
     - Can use theme variables (`--ifm-color-*`) for consistency with book theme
     - No external dependency
     - Lightweight, fast load time
   - Alternatives considered:
     - Material-UI: 300KB+, overkill for 6 components
     - Tailwind: Requires build config changes, not Docusaurus-native
     - Styled-components: Adds runtime CSS-in-JS overhead

7. **Error Handling Strategy**
   - Decision: Three-tier error handling (network, backend, validation)
   - Rationale:
     - Network errors (timeout, offline): "Unable to connect. Check your internet."
     - Backend 500 errors: "Something went wrong. Please try again."
     - Backend 400 errors: Display backend error message (e.g., "Question too long")
     - Empty results: Display backend refusal message (e.g., "Not available in the book")
   - Alternatives considered:
     - Generic "Error occurred" for all: Poor UX, user can't diagnose
     - Retry logic: Adds complexity, not required by spec
     - Error codes: Backend doesn't return structured error codes

8. **Loading Indicator Design**
   - Decision: Inline spinner below input box with "Searching..." text
   - Rationale:
     - Meets SC-002 (displayed within 100ms)
     - Non-blocking: user can still scroll, read, select text
     - Clear feedback: user knows system is processing
   - Alternatives considered:
     - Overlay spinner: Blocks UI interaction, fails SC-008 (concurrent queries)
     - Disabled input: Bad UX, user can't queue next question
     - Progress bar: Misleading (can't estimate progress)

### Research Artifacts

See [research.md](./research.md) for detailed findings.

## Phase 1: Design & Contracts

*Prerequisites: research.md complete*

### Data Model

See [data-model.md](./data-model.md) for entity definitions.

**Summary of entities**:
- `ChatMessage`: User query or bot response in conversation history
- `AnswerWithSources`: Structured answer data from backend (answer + sources + chunks + metadata)
- `ChunkReference`: Book content chunk with location metadata (from backend AskResponse)
- `SelectedTextContext`: Captured text selection with position info

### API Contracts

See [contracts/chatbot-ui-api.yaml](./contracts/chatbot-ui-api.yaml) for OpenAPI specification.

**Summary**:
- **POST /ask**: Existing FastAPI endpoint (no changes required)
  - Request: `{ question: string, selected_text?: string }`
  - Response: `{ answer: string, sources: string[], matched_chunks: ChunkReference[], grounded: bool, retrieval_quality?: string }`
  - Errors: 400 (validation), 500 (system error), timeout (5s)

### Quickstart Guide

See [quickstart.md](./quickstart.md) for developer setup and testing instructions.

**Summary**:
1. Install frontend deps: `cd my-book && npm install`
2. Start backend: `cd Backend && python start_server.py`
3. Start frontend: `cd my-book && npm start`
4. Test: Open book, click 💬 icon, ask question, verify answer display

## Phase 2: Implementation Tasks

*This section is intentionally empty. Run `/sp.tasks` to generate actionable tasks from this plan.*

Tasks will be generated in [tasks.md](./tasks.md) by the `/sp.tasks` command.

## Architecture Decisions

### Decision 1: Floating Icon + Panel UI Pattern

**Context**: Need chatbot UI that doesn't break existing book layout (FR-011, SC-010)

**Options Considered**:
1. Modal dialog (click icon → full-screen modal)
2. Sidebar panel (always visible, pushes content)
3. Floating panel (click icon → overlay panel, dismissible)

**Decision**: Floating panel with toggle icon

**Rationale**:
- Modal: Too intrusive, blocks reading
- Sidebar: Changes layout, can obscure content on small screens
- Floating panel: Non-intrusive, dismissible, doesn't affect layout

**Trade-offs**:
- Benefit: Best for reading flow - user controls visibility
- Cost: Slightly more complex CSS (z-index management, positioning)

### Decision 2: Request/Response Model (No Streaming)

**Context**: Backend /ask returns complete response, no streaming API available

**Options Considered**:
1. Single request/response (wait for full answer)
2. WebSocket with streaming (would require backend changes)
3. Polling (send request, poll for status)

**Decision**: Single request/response with loading indicator

**Rationale**:
- Backend doesn't support streaming (out of scope per spec assumptions)
- Polling adds unnecessary complexity and backend load
- Loading indicator provides sufficient feedback (SC-002: <100ms)

**Trade-offs**:
- Benefit: Simple, matches existing backend, no backend changes
- Cost: User waits for full answer (but SC-001: <5s is acceptable)

### Decision 3: Session-Only History (No Persistence)

**Context**: Spec notes "Authentication or user account features" are out of scope

**Options Considered**:
1. Session-only history (component state)
2. LocalStorage persistence
3. Backend-persisted history with user accounts

**Decision**: Session-only history in React Context

**Rationale**:
- Backend persistence requires auth (out of scope)
- LocalStorage adds complexity, edge cases (quota, privacy, sync)
- Session-only meets P3 story (conversation history during session)

**Trade-offs**:
- Benefit: Simple, privacy-friendly, no storage management
- Cost: History lost on page reload (acceptable for MVP)

### Decision 4: Text Selection via Browser API

**Context**: Need to capture user-selected text for context override (FR-004, FR-005)

**Options Considered**:
1. Browser Selection API (`window.getSelection()`)
2. Third-party React library (react-text-selection)
3. Custom selection tracking (mousedown/mouseup events)

**Decision**: Browser Selection API with `mouseup` listener

**Rationale**:
- Native API, no dependencies
- Works across all content types (MDX, HTML, code blocks)
- Handles edge cases (empty selection, collapsed selection)

**Trade-offs**:
- Benefit: Zero dependencies, reliable, cross-browser
- Cost: Requires handling edge cases (selection outside book content)

### Decision 5: Error Classification Strategy

**Context**: Multiple error types possible (network, backend, validation) - need user-friendly messages

**Options Considered**:
1. Generic "Error occurred" for all errors
2. Structured error types with specific messages
3. Pass backend error messages directly to user

**Decision**: Three-tier classification (network, backend 500, backend 400)

**Rationale**:
- Generic message: Poor UX, user can't diagnose or retry appropriately
- Direct backend messages: Some are too technical (e.g., "Qdrant connection failed")
- Three-tier: Balances specificity with user-friendliness

**Trade-offs**:
- Benefit: Clear user guidance, meets SC-009 (no stack traces)
- Cost: Requires error type detection logic in frontend

## Re-evaluation: Constitution Check Post-Design

*GATE: Must pass before proceeding to /sp.tasks*

### Constitution Compliance After Design

1. **Book-Only Knowledge (NON-NEGOTIABLE)** ✅
   - Design: UI has no external API calls, all answers from /ask endpoint
   - Verified: chatbotApi.ts service only calls /ask, no other endpoints

2. **No Hallucination Policy (NON-NEGOTIABLE)** ✅
   - Design: ErrorMessage.tsx displays backend refusal messages verbatim
   - Verified: No client-side answer generation, no answer modification

3. **Selected Text Priority (NON-NEGOTIABLE)** ✅
   - Design: useTextSelection.ts captures selection, useChatbot.ts sends as selected_text param
   - Verified: Backend AskRequest.selected_text field populated when selection exists

4. **Concise Response Format** ✅
   - Design: AnswerDisplay.tsx renders backend answer without modification
   - Verified: No text truncation, summarization, or expansion

5. **RAG Flow Separation** ✅
   - Design: chatbotApi.ts sends selected_text only when user has active selection
   - Verified: Backend routing handled server-side, UI just passes parameters

6. **Technology Stack Compliance** ✅
   - Design: React/TypeScript UI in Docusaurus, FastAPI backend unchanged
   - Verified: Matches constitution stack (Docusaurus + FastAPI + REST)

7. **Strict Agent Instructions** ✅ (Backend responsibility)
   - Design: UI displays backend responses faithfully
   - Verified: No answer filtering, modification, or augmentation

### Gate Result: **PASS** - All constitution principles maintained after design

## Implementation Risks

1. **Text Selection Across Page Boundaries**
   - Risk: User selects text, navigates to new page, selection lost
   - Mitigation: Capture selection immediately on mouseup, store in React Context
   - Fallback: Display "Selection lost" message if user tries to query with stale selection

2. **Backend Timeout on Slow Queries**
   - Risk: Backend takes >5s, frontend timeout triggers, user sees error
   - Mitigation: Frontend timeout set to 5s (matches SC-009), show "Query took too long" message
   - Fallback: User can retry query (backend may cache embeddings)

3. **Chatbot Panel Obscures Content**
   - Risk: Panel overlaps important book text on small screens
   - Mitigation: Panel is dismissible, max-width 400px, positioned bottom-right
   - Fallback: User can minimize panel and re-open after scrolling

4. **Concurrent Queries Cause Confusion**
   - Risk: User sends multiple queries, responses arrive out of order
   - Mitigation: Disable send button during processing, show loading indicator
   - Fallback: Cancel previous request when new query sent (AbortController)

## Dependencies

- Docusaurus 3.9.2+ (existing)
- React 19+ (existing)
- TypeScript 5.6+ (existing)
- Backend /ask endpoint (existing at Backend/api.py)
- FastAPI server running on http://localhost:8000 (existing)

## Next Steps

1. **Review this plan** - Ensure all stakeholders agree with architecture decisions
2. **Run `/sp.tasks`** - Generate actionable implementation tasks from this plan
3. **Implement P1 stories first** - Basic Q&A and selected text override
4. **Test with real book content** - Validate source navigation and chunk display
5. **Iterate on UX** - Adjust panel positioning, loading indicators based on user feedback
