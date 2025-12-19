---
id: 002
title: Book RAG Chatbot UI Implementation Plan
stage: plan
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5
feature: 001-book-rag-chatbot-ui
branch: 001-book-rag-chatbot-ui
user: Kashaf Aman
command: /sp.plan
labels: ["planning", "architecture", "rag", "chatbot", "ui", "docusaurus", "react"]
links:
  spec: specs/001-book-rag-chatbot-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-book-rag-chatbot-ui/plan.md
  - specs/001-book-rag-chatbot-ui/research.md
  - specs/001-book-rag-chatbot-ui/data-model.md
  - specs/001-book-rag-chatbot-ui/quickstart.md
  - specs/001-book-rag-chatbot-ui/contracts/chatbot-ui-api.yaml
  - CLAUDE.md (updated by agent context script)
tests:
  - Constitution Check validation (all 7 principles passed)
  - Technical Context completeness validation
---

## Prompt

```
/sp.plan Add chat input to book pages (HTML/JS or React)

Capture user query or selected text

Send request to backend /ask endpoint (REST/WebSocket)

Receive JSON response: answer + sources + matched_chunks

Display response in UI with clear formatting

Show loading indicators while processing

Handle errors and empty results gracefully

Test queries, selected-text override, and end-to-end flow
Add a floating message icon 💬 on each book page

On click → open/close a chatbot panel ("Ask the Book")

Chat UI:

Messages area

Input box + Send button
```

## Response snapshot

Created comprehensive implementation plan for Book RAG Chatbot UI feature:

**Technical Context Established**:
- Language/Version: TypeScript 5.6, React 19, Python 3.11 (backend)
- Dependencies: Docusaurus 3.9.2, FastAPI (existing), Pydantic (existing)
- Testing: Jest + React Testing Library (unit), Playwright (E2E)
- Target Platform: Modern web browsers (Chrome, Firefox, Safari) via Docusaurus
- Project Type: Web application (frontend + backend separation)
- Performance Goals: <100ms UI response, 100ms loading indicator display, <5s query response
- Constraints: Must not break book layout, dismissible UI, 5s backend timeout, prevent empty queries
- Scale: ~50-100 book pages, 10-50 concurrent readers, 2000 char text selection limit

**Constitution Check**: All 7 principles PASSED
1. Book-Only Knowledge ✅ - UI displays only backend /ask responses
2. No Hallucination Policy ✅ - Shows refusal messages verbatim
3. Selected Text Priority ✅ - Captures via window.getSelection(), sends as parameter
4. Concise Response Format ✅ - Renders backend responses without modification
5. RAG Flow Separation ✅ - Sends selected_text parameter when present
6. Technology Stack Compliance ✅ - React/TypeScript + FastAPI + REST
7. Strict Agent Instructions ✅ - Backend enforces, UI displays faithfully

**Phase 0: Research Complete** (8 research topics resolved):
1. Docusaurus Theme Customization → Use Root.tsx swizzling
2. Text Selection Capture → window.getSelection() with mouseup listener
3. Frontend State Management → React Context API with useChatbot hook
4. Backend Communication → Fetch API with 5s AbortController timeout
5. Source Navigation → Docusaurus router + anchor links
6. UI Component Library → Pure CSS modules + Docusaurus theme variables
7. Error Handling Strategy → Three-tier classification (network, backend 500, backend 400)
8. Loading Indicator Design → Inline spinner below input, non-blocking

**Phase 1: Design Complete**:
- **Data Model** (6 entities defined in data-model.md):
  - ChatMessage: User query or bot response in conversation history
  - SourceReference: Simplified source identifier for clickable links
  - ChunkReference: Book content chunk with location metadata
  - SelectedTextContext: Captured text selection state
  - AskRequest: Backend API request payload
  - AskResponse: Backend API response payload
- **API Contract** (OpenAPI 3.0 spec in contracts/chatbot-ui-api.yaml):
  - POST /ask endpoint documented with request/response schemas
  - Error responses (400, 500, 504) with user-friendly messages
  - Examples for normal query, selected text, refusal, timeout
- **Quickstart Guide** (developer setup in quickstart.md):
  - Prerequisites, installation steps, testing checklist
  - 8 manual tests covering all functional requirements and success criteria
  - Troubleshooting section for common issues

**Project Structure Defined**:
- Frontend: my-book/src/components/ChatbotUI/ (9 new components)
- Hooks: my-book/src/hooks/ (useChatbot, useTextSelection, useBackendApi)
- Services: my-book/src/services/chatbotApi.ts (API client)
- Types: my-book/src/types/chatbot.ts (TypeScript interfaces)
- Global Wrapper: my-book/src/theme/Root.tsx (swizzled)
- Tests: my-book/tests/unit/ and my-book/tests/e2e/

**Architecture Decisions** (5 key decisions documented):
1. Floating Icon + Panel UI Pattern → Non-intrusive, dismissible overlay
2. Request/Response Model → Single request/response (no streaming)
3. Session-Only History → React Context state (no persistence)
4. Text Selection via Browser API → Native window.getSelection()
5. Error Classification Strategy → Three-tier user-friendly messages

**Dependencies**: Zero new dependencies (all native browser APIs and existing Docusaurus/React features)

**Re-evaluation**: Constitution Check PASSED post-design - all 7 principles maintained

**Implementation Risks** (4 identified with mitigations):
1. Text Selection Across Page Boundaries → Capture immediately on mouseup, store in Context
2. Backend Timeout on Slow Queries → 5s timeout with "Query took too long" message
3. Chatbot Panel Obscures Content → Dismissible, max-width 400px, bottom-right position
4. Concurrent Queries Cause Confusion → Disable send button during processing, cancel previous request

## Outcome

- ✅ Impact: Complete implementation plan with research, data model, API contract, quickstart guide; ready for /sp.tasks
- 🧪 Tests: Constitution Check (7/7 passed), Technical Context validation (complete)
- 📁 Files: 5 planning artifacts created (plan.md, research.md, data-model.md, quickstart.md, chatbot-ui-api.yaml)
- 🔁 Next prompts: /sp.tasks (generate implementation tasks), /sp.adr (document architectural decisions)
- 🧠 Reflection: Plan leverages existing Backend /ask endpoint (no backend changes required). All tech decisions use zero-dependency native APIs. Constitution compliance verified pre- and post-design. Manual testing checklist covers all 13 functional requirements and 10 success criteria.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully
- Graders run and results (PASS/FAIL): Constitution Check pre-design PASS, Constitution Check post-design PASS
- Prompt variant (if applicable): Standard /sp.plan workflow with detailed user input
- Next experiment (smallest change to try): Proceed to /sp.tasks to generate actionable implementation tasks prioritized by user story (P1 → P2 → P3)
