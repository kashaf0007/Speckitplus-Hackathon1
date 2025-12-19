# Research: Book RAG Chatbot UI

**Feature**: 001-book-rag-chatbot-ui
**Date**: 2025-12-18
**Status**: Complete

## Purpose

This document consolidates research findings from Phase 0, resolving all technical unknowns and establishing best practices for implementing the chatbot UI in Docusaurus.

## Research Topics & Findings

### 1. Docusaurus Theme Customization

**Question**: How to embed a persistent UI component across all book pages?

**Decision**: Use `src/theme/Root.tsx` for global chatbot wrapper (swizzled Root component)

**Rationale**:
- Root.tsx is the highest-level wrapper in Docusaurus v3
- Ensures chatbot is available on all pages without modifying each page
- Swizzling Root is a documented Docusaurus pattern for adding global UI elements
- Component state managed in Root.tsx persists across page navigation

**Alternatives Considered**:
1. **Layout wrapper**: Requires swizzling multiple layouts (DocLayout, BlogLayout, etc.) - more fragile
2. **Docusaurus plugin**: Overkill for a single UI component, adds build complexity
3. **Manual injection in each page**: Breaks with content updates, not maintainable

**Implementation Notes**:
- Run `npm run swizzle @docusaurus/theme-classic Root -- --wrap` to create wrapper
- Wrap children with ChatbotProvider context
- ChatbotToggle and ChatbotPanel rendered at Root level

**References**:
- Docusaurus Swizzling docs: https://docusaurus.io/docs/swizzling
- Root wrapper pattern: https://docusaurus.io/docs/advanced/client#root

---

### 2. Text Selection Capture in React

**Question**: How to reliably capture user text selection across all book content types?

**Decision**: Use `window.getSelection()` API with `useEffect` listener on `mouseup` event

**Rationale**:
- Browser-native API with excellent cross-browser support (Chrome 1+, Firefox 1+, Safari 1+)
- Works across all Docusaurus content types (MDX, HTML, code blocks, tables)
- Handles edge cases: empty selection, collapsed selection, cross-element selection
- Zero dependencies

**Alternatives Considered**:
1. **`document.getSelection()`**: Functionally identical to `window.getSelection()` in modern browsers
2. **Third-party library** (react-text-selection): Adds 8KB dependency for simple use case
3. **Selection event**: Not supported in Safari, unreliable cross-browser

**Implementation Notes**:
```typescript
useEffect(() => {
  const handleMouseUp = () => {
    const selection = window.getSelection();
    if (selection && selection.toString().trim().length > 0) {
      setSelectedText(selection.toString().trim());
    } else {
      setSelectedText(null);
    }
  };

  document.addEventListener('mouseup', handleMouseUp);
  return () => document.removeEventListener('mouseup', handleMouseUp);
}, []);
```

**Edge Cases Handled**:
- Empty selection (user clicks without dragging): Set selectedText to null
- Whitespace-only selection: Trim and check length
- Selection outside book content: Allow (user may select from UI elements)
- Selection limit: Truncate to 2000 characters (backend max per spec)

**References**:
- MDN Selection API: https://developer.mozilla.org/en-US/docs/Web/API/Selection
- Browser compatibility: https://caniuse.com/mdn-api_selection

---

### 3. Frontend State Management

**Question**: How to manage chatbot state across page navigation?

**Decision**: React Context API with `useChatbot` custom hook

**Rationale**:
- Simple state shape: `{ messages: ChatMessage[], loading: boolean, error: string | null, selectedText: string | null }`
- Context provider in Root.tsx ensures state persists across Docusaurus page navigation
- No external dependencies (built-in React feature)
- Performance adequate for expected load (10-50 concurrent users, <100 messages per session)

**Alternatives Considered**:
1. **Component-level state**: Loses conversation history on page navigation (fails P3 user story)
2. **Redux**: Overkill for 4 state variables, adds 15KB+ bundle size
3. **Zustand**: Lighter than Redux (3KB) but still unnecessary for this use case
4. **LocalStorage**: Adds complexity (quota, privacy, sync), not needed for session-only history

**Implementation Notes**:
```typescript
// Context shape
interface ChatbotContextType {
  messages: ChatMessage[];
  loading: boolean;
  error: string | null;
  selectedText: string | null;
  sendMessage: (question: string) => Promise<void>;
  clearError: () => void;
  setSelectedText: (text: string | null) => void;
}

// Provider in Root.tsx
<ChatbotProvider>
  {children}
</ChatbotProvider>
```

**Performance Considerations**:
- Max 100 messages in history (truncate older messages to prevent memory bloat)
- Debounce text selection updates (100ms) to avoid excessive re-renders
- Memoize AnswerDisplay components to prevent unnecessary re-renders

**References**:
- React Context API: https://react.dev/reference/react/useContext
- Performance optimization: https://react.dev/reference/react/memo

---

### 4. Backend Communication

**Question**: How to communicate with FastAPI /ask endpoint reliably?

**Decision**: Fetch API with async/await, 5-second timeout via AbortController

**Rationale**:
- Native browser API, zero dependencies
- Simple async error handling with try/catch
- AbortController provides clean timeout mechanism (also native)
- Matches backend timeout expectations (SC-009: error within 5s)

**Alternatives Considered**:
1. **Axios**: Adds 15KB bundle size for features we don't use (interceptors, cancellation is simpler with AbortController)
2. **WebSocket**: Out of scope (spec assumes request/response, not streaming)
3. **Server-Sent Events**: Overkill for single response per query

**Implementation Notes**:
```typescript
const askQuestion = async (question: string, selectedText?: string): Promise<AskResponse> => {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), 5000); // 5s timeout

  try {
    const response = await fetch('http://localhost:8000/ask', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ question, selected_text: selectedText }),
      signal: controller.signal
    });

    clearTimeout(timeoutId);

    if (!response.ok) {
      // Handle 400/500 errors
      const error = await response.json();
      throw new Error(error.detail || 'Request failed');
    }

    return await response.json();
  } catch (error) {
    if (error.name === 'AbortError') {
      throw new Error('Request timed out. The book search took too long.');
    }
    throw error;
  }
};
```

**Error Handling**:
- Network error (offline): "Unable to connect. Check your internet connection."
- Timeout (>5s): "Request timed out. The book search took too long."
- 400 error: Display backend error message (e.g., "Question too long")
- 500 error: "Something went wrong. Please try again."

**References**:
- Fetch API: https://developer.mozilla.org/en-US/docs/Web/API/Fetch_API
- AbortController: https://developer.mozilla.org/en-US/docs/Web/API/AbortController

---

### 5. Source Reference Navigation

**Question**: How to navigate users to referenced book sections?

**Decision**: Use Docusaurus router (`useHistory` from `@docusaurus/router`) + anchor links

**Rationale**:
- Docusaurus auto-generates IDs for headings (e.g., `## Introduction` → `#introduction`)
- Backend `ChunkReference` includes `chapter` and `section` fields
- Can construct anchor links: `/docs/chapter-3#section-title`
- Native Docusaurus navigation, no page reload, preserves chatbot state

**Alternatives Considered**:
1. **Page reload with hash**: Works but jarring UX, loses chatbot state
2. **Scroll-only without navigation**: Requires knowing DOM structure, fragile if content changes
3. **No navigation**: Fails FR-010 (clickable source references)

**Implementation Notes**:
```typescript
import { useHistory } from '@docusaurus/router';

const SourceLink = ({ chunk }: { chunk: ChunkReference }) => {
  const history = useHistory();

  const handleClick = () => {
    // Construct path from chunk metadata
    const path = `/docs/${chunk.chapter || 'intro'}`;
    const hash = chunk.section ? `#${slugify(chunk.section)}` : '';

    history.push(path + hash);
  };

  return <a onClick={handleClick}>{chunk.chapter} - {chunk.section}</a>;
};
```

**Edge Cases**:
- Missing chapter: Default to `/docs/intro`
- Missing section: Navigate to chapter top
- Invalid section name: Slugify and attempt match (e.g., "Chapter 3: Setup" → "chapter-3-setup")

**References**:
- Docusaurus router: https://docusaurus.io/docs/advanced/routing
- Heading IDs: https://docusaurus.io/docs/markdown-features/headings

---

### 6. UI Component Library

**Question**: Should we use a component library or custom CSS?

**Decision**: Pure CSS modules + Docusaurus theme variables

**Rationale**:
- Docusaurus has built-in CSS modules support (zero config)
- Can use theme variables (`--ifm-color-primary`, `--ifm-font-size-base`) for consistency
- No external dependency
- Lightweight (<5KB CSS total), fast load time
- Full control over styling

**Alternatives Considered**:
1. **Material-UI**: 300KB+ bundle size, overkill for 6 simple components
2. **Tailwind CSS**: Requires build config changes, not Docusaurus-native
3. **Styled-components**: Adds runtime CSS-in-JS overhead, 12KB bundle size

**Implementation Notes**:
```css
/* styles.module.css */
.chatPanel {
  position: fixed;
  bottom: 20px;
  right: 20px;
  width: 400px;
  max-height: 600px;
  background: var(--ifm-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 8px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  z-index: 1000;
}

.toggle {
  position: fixed;
  bottom: 20px;
  right: 20px;
  width: 56px;
  height: 56px;
  background: var(--ifm-color-primary);
  color: white;
  border-radius: 50%;
  z-index: 999;
}
```

**Theme Variables Used**:
- `--ifm-color-primary`: Toggle button background
- `--ifm-background-color`: Panel background
- `--ifm-color-emphasis-300`: Panel border
- `--ifm-font-size-base`: Text sizing
- `--ifm-spacing-horizontal`: Padding/margins

**References**:
- Docusaurus styling: https://docusaurus.io/docs/styling-layout
- CSS modules: https://docusaurus.io/docs/styling-layout#css-modules

---

### 7. Error Handling Strategy

**Question**: How to present errors in a user-friendly way?

**Decision**: Three-tier error classification (network, backend, validation)

**Rationale**:
- Provides specific guidance without exposing technical details
- Meets SC-009 (no stack traces visible)
- Users can understand and potentially resolve issues

**Error Types & Messages**:

| Error Type | Detection | User Message |
|------------|-----------|--------------|
| Network timeout | `AbortError` after 5s | "Request timed out. The book search took too long. Please try again." |
| Network offline | `TypeError: Failed to fetch` | "Unable to connect. Check your internet connection." |
| Backend 500 | `response.status >= 500` | "Something went wrong on our end. Please try again later." |
| Backend 400 | `response.status === 400` | Display `error.detail` from backend |
| Empty results | `answer` contains "not available in the book" | Display backend refusal message verbatim |
| Validation (empty query) | Client-side check | "Please enter a question before submitting." |

**Alternatives Considered**:
1. **Generic "Error occurred"**: Poor UX, user can't diagnose or take action
2. **Pass all backend errors directly**: Some are too technical (e.g., "Qdrant connection failed")
3. **Retry logic**: Adds complexity, not required by spec, user can manually retry

**Implementation Notes**:
```typescript
const classifyError = (error: Error, response?: Response): string => {
  if (error.name === 'AbortError') {
    return 'Request timed out. The book search took too long. Please try again.';
  }

  if (error.message === 'Failed to fetch') {
    return 'Unable to connect. Check your internet connection.';
  }

  if (response) {
    if (response.status >= 500) {
      return 'Something went wrong on our end. Please try again later.';
    }
    if (response.status === 400) {
      return error.message; // Backend validation message
    }
  }

  return 'An unexpected error occurred. Please try again.';
};
```

**References**:
- Error UX best practices: Nielsen Norman Group - Error Message Guidelines

---

### 8. Loading Indicator Design

**Question**: How to show loading state without blocking the UI?

**Decision**: Inline spinner below input box with "Searching..." text

**Rationale**:
- Meets SC-002 (displayed within 100ms of query submission)
- Non-blocking: user can still scroll book content, select text, view previous messages
- Clear feedback: user knows system is processing
- Disable send button during loading to prevent duplicate submissions

**Alternatives Considered**:
1. **Full-screen overlay spinner**: Blocks all UI interaction, fails SC-008 (3+ concurrent queries)
2. **Disabled input field**: Bad UX, user can't prepare next question
3. **Progress bar**: Misleading - can't estimate RAG retrieval progress

**Implementation Notes**:
```tsx
{loading && (
  <div className={styles.loadingIndicator}>
    <Spinner size="small" />
    <span>Searching book content...</span>
  </div>
)}

<button
  disabled={loading || !question.trim()}
  onClick={handleSend}
>
  Send
</button>
```

**Interaction Rules**:
- Query input remains enabled (user can type next question)
- Send button disabled while loading
- Previous messages remain visible and scrollable
- User can close/minimize panel while loading (request continues in background)

**References**:
- Loading states UX: Material Design - Progress indicators

---

## Summary & Recommendations

### Key Decisions Made

1. **Global Wrapper**: Use Root.tsx swizzling for persistent chatbot across all pages
2. **State Management**: React Context API for simple, performant state
3. **Text Selection**: Native `window.getSelection()` with mouseup listener
4. **Backend Communication**: Fetch API with 5s AbortController timeout
5. **Navigation**: Docusaurus router + anchor links for source references
6. **Styling**: CSS modules + Docusaurus theme variables
7. **Error Handling**: Three-tier classification with user-friendly messages
8. **Loading UX**: Inline spinner, non-blocking, disabled send button

### No Dependencies Added

All decisions use native browser APIs and existing Docusaurus/React features:
- No external state management library
- No HTTP client library
- No UI component library
- No text selection library

**Total bundle size impact**: <5KB (CSS only)

### Next Steps

1. Proceed to Phase 1: Create data-model.md with entity definitions
2. Generate OpenAPI contract for /ask endpoint (documents existing backend)
3. Write quickstart.md with setup and testing instructions
4. Run `/sp.tasks` to generate implementation tasks

### References Used

- Docusaurus v3 Documentation: https://docusaurus.io/docs
- React 19 Documentation: https://react.dev
- MDN Web APIs: https://developer.mozilla.org/en-US/docs/Web/API
- TypeScript 5.6 Handbook: https://www.typescriptlang.org/docs/
