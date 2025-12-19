# Data Model: Book RAG Chatbot UI

**Feature**: 001-book-rag-chatbot-ui
**Date**: 2025-12-18
**Status**: Complete

## Purpose

This document defines the frontend data entities for the chatbot UI. These TypeScript interfaces model conversation state, backend responses, and user interactions.

## Entity Definitions

### 1. ChatMessage

Represents a single message in the conversation history (user query or bot response).

**Purpose**: Store conversation history in React Context for display in MessageList component.

**TypeScript Definition**:
```typescript
interface ChatMessage {
  id: string;                    // Unique message ID (UUID or timestamp)
  role: 'user' | 'assistant';    // Message sender
  content: string;               // Message text (question or answer)
  timestamp: Date;               // Message creation time
  selectedTextUsed?: string;     // If user query, the selected text context (if any)
  sources?: SourceReference[];   // If assistant response, source references
  chunks?: ChunkReference[];     // If assistant response, matched chunks
  error?: string;                // If assistant response, error message (if any)
}
```

**Field Descriptions**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | `string` | Yes | Unique identifier (e.g., `Date.now().toString()` or UUID) |
| `role` | `'user' \| 'assistant'` | Yes | Who sent the message |
| `content` | `string` | Yes | Message text (1-1000 chars for user, 1-5 sentences for assistant) |
| `timestamp` | `Date` | Yes | When message was created |
| `selectedTextUsed` | `string?` | No | If user query with selected text, store the selection here |
| `sources` | `SourceReference[]?` | No | If assistant response, list of source identifiers |
| `chunks` | `ChunkReference[]?` | No | If assistant response, matched content chunks |
| `error` | `string?` | No | If assistant response is an error, error message to display |

**Validation Rules**:
- `content` must not be empty or whitespace-only
- `role` must be exactly `'user'` or `'assistant'`
- If `role === 'user'`, `sources` and `chunks` must be undefined
- If `role === 'assistant'` and `error` is present, `sources` and `chunks` should be empty

**Relationships**:
- User message → Assistant message (1:1, each query gets one response)
- Assistant message → SourceReference (1:many, one answer may cite multiple sources)
- Assistant message → ChunkReference (1:many, one answer may use multiple chunks)

**State Transitions**:
1. User sends query → Create ChatMessage with `role: 'user'`
2. Backend processing → No message yet (show loading indicator)
3. Backend responds → Create ChatMessage with `role: 'assistant'`, populate `sources` and `chunks`
4. Backend errors → Create ChatMessage with `role: 'assistant'`, populate `error` field

**Example**:
```typescript
// User message
{
  id: '1702845123456',
  role: 'user',
  content: 'What are the key features of ROS 2?',
  timestamp: new Date('2025-12-18T10:30:00Z'),
  selectedTextUsed: undefined
}

// Assistant message (success)
{
  id: '1702845127890',
  role: 'assistant',
  content: 'ROS 2 provides real-time capabilities, improved security features, and enhanced cross-platform support.',
  timestamp: new Date('2025-12-18T10:30:03Z'),
  sources: [
    { chunkId: 'chunk-123', label: 'Chapter 3: ROS 2 Overview' }
  ],
  chunks: [
    {
      chunk_id: 'chunk-123',
      text: 'ROS 2 introduces several key improvements...',
      page: 45,
      chapter: 'Chapter 3',
      section: 'ROS 2 Overview'
    }
  ],
  error: undefined
}

// Assistant message (error)
{
  id: '1702845130000',
  role: 'assistant',
  content: '',
  timestamp: new Date('2025-12-18T10:30:05Z'),
  error: 'Unable to connect. Check your internet connection.'
}
```

---

### 2. SourceReference

Simplified source identifier for display in the sources list.

**Purpose**: Provide clickable source links in the AnswerDisplay component.

**TypeScript Definition**:
```typescript
interface SourceReference {
  chunkId: string;        // Chunk ID from backend (e.g., 'chunk-123' or 'selected_text')
  label: string;          // Human-readable label (e.g., 'Chapter 3: ROS 2 Overview')
  path?: string;          // Optional Docusaurus path for navigation (e.g., '/docs/chapter-3')
  anchor?: string;        // Optional anchor for navigation (e.g., 'ros-2-overview')
}
```

**Field Descriptions**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `chunkId` | `string` | Yes | Unique chunk identifier from backend |
| `label` | `string` | Yes | Display text for source link (constructed from chapter + section) |
| `path` | `string?` | No | Docusaurus path for navigation (e.g., `/docs/chapter-3`) |
| `anchor` | `string?` | No | Anchor hash for navigation (e.g., `ros-2-overview`) |

**Validation Rules**:
- `chunkId` must not be empty
- `label` must not be empty
- If `path` is present, must start with `/`
- If `anchor` is present, must be a valid slug (lowercase, hyphens only)

**Derived From**: Constructed from `ChunkReference.chapter`, `.section`, and `.chunk_id`

**Example**:
```typescript
{
  chunkId: 'chunk-123',
  label: 'Chapter 3: ROS 2 Overview',
  path: '/docs/chapter-3',
  anchor: 'ros-2-overview'
}
```

---

### 3. ChunkReference

Detailed book content chunk from backend (maps to backend `ChunkReference` model).

**Purpose**: Display matched chunks with metadata in ChunkDisplay component. This is the frontend representation of the backend `ChunkReference` Pydantic model.

**TypeScript Definition**:
```typescript
interface ChunkReference {
  chunk_id: string;       // Unique identifier from Qdrant or 'selected_text'
  text: string;           // Full chunk text (max 2000 chars, truncated if longer)
  page: number | null;    // Page number in book (null for selected text or web content)
  chapter: string | null; // Chapter name/number (null if not available)
  section: string | null; // Section name (null if not available)
}
```

**Field Descriptions**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `chunk_id` | `string` | Yes | Unique identifier from Qdrant (e.g., `'chunk-123'`) or `'selected_text'` |
| `text` | `string` | Yes | Full text of the chunk (truncated to 2000 chars by backend) |
| `page` | `number \| null` | Yes | Page number (1-indexed) or null |
| `chapter` | `string \| null` | Yes | Chapter identifier (e.g., `'Chapter 3'` or `'Introduction'`) |
| `section` | `string \| null` | Yes | Section identifier (e.g., `'ROS 2 Overview'`) |

**Validation Rules**:
- `chunk_id` must not be empty
- `text` must not be empty
- `page` must be >= 1 if present
- All fields must be present (though `page`, `chapter`, `section` may be null)

**Source**: Received from backend `/ask` endpoint in `AskResponse.matched_chunks` array

**Example**:
```typescript
// From retrieval
{
  chunk_id: 'chunk-123',
  text: 'ROS 2 introduces several key improvements over ROS 1, including real-time capabilities through DDS middleware, enhanced security features with authentication and encryption, and improved cross-platform support for Windows, macOS, and embedded systems.',
  page: 45,
  chapter: 'Chapter 3',
  section: 'ROS 2 Overview'
}

// From selected text
{
  chunk_id: 'selected_text',
  text: 'Real-time systems require deterministic behavior and bounded latency.',
  page: null,
  chapter: null,
  section: null
}
```

---

### 4. SelectedTextContext

Captures user text selection state.

**Purpose**: Store selected text and position for context override in queries.

**TypeScript Definition**:
```typescript
interface SelectedTextContext {
  text: string;           // Selected text content (trimmed)
  startOffset: number;    // Character offset of selection start
  endOffset: number;      // Character offset of selection end
  capturedAt: Date;       // When selection was captured
}
```

**Field Descriptions**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `text` | `string` | Yes | Selected text content (trimmed, max 2000 chars) |
| `startOffset` | `number` | Yes | Character position where selection starts |
| `endOffset` | `number` | Yes | Character position where selection ends |
| `capturedAt` | `Date` | Yes | Timestamp when selection was captured |

**Validation Rules**:
- `text` must not be empty or whitespace-only
- `text` must be <= 2000 characters (spec requirement SC-007)
- `endOffset` must be > `startOffset`
- `capturedAt` must not be in the future

**Lifecycle**:
1. User selects text → `mouseup` event fires
2. `window.getSelection()` captures selection
3. Create `SelectedTextContext` with trimmed text and offsets
4. Store in React Context state
5. If user clears selection → Set context to `null`
6. If user navigates away → Clear context (selection invalid)

**Example**:
```typescript
{
  text: 'Real-time systems require deterministic behavior and bounded latency.',
  startOffset: 1523,
  endOffset: 1593,
  capturedAt: new Date('2025-12-18T10:29:45Z')
}
```

---

### 5. AskRequest

Request payload for backend `/ask` endpoint (frontend representation of backend `AskRequest` Pydantic model).

**Purpose**: Type-safe request construction for backend API calls.

**TypeScript Definition**:
```typescript
interface AskRequest {
  question: string;           // User's question (1-1000 chars)
  selected_text?: string;     // Optional selected text override (max 10000 chars)
}
```

**Field Descriptions**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `question` | `string` | Yes | User's natural language question (1-1000 chars) |
| `selected_text` | `string?` | No | User-highlighted passage that overrides retrieval (max 10000 chars) |

**Validation Rules** (enforced by backend, documented for frontend):
- `question` must not be empty or whitespace-only
- `question` must be 1-1000 characters
- `selected_text` if provided, must be <= 10000 characters
- Backend trims whitespace from both fields

**Example**:
```typescript
// Normal query
{
  question: 'What are the key features of ROS 2?'
}

// Query with selected text
{
  question: 'Explain this passage',
  selected_text: 'Real-time systems require deterministic behavior and bounded latency.'
}
```

---

### 6. AskResponse

Response payload from backend `/ask` endpoint (frontend representation of backend `AskResponse` Pydantic model).

**Purpose**: Type-safe backend response parsing.

**TypeScript Definition**:
```typescript
interface AskResponse {
  answer: string;                    // Generated answer (1-5 sentences) or refusal message
  sources: string[];                 // List of source identifiers (chunk IDs or 'selected_text')
  matched_chunks: ChunkReference[];  // Full chunk details with text and metadata
  grounded: boolean;                 // Whether answer is grounded in book content
  retrieval_quality: string | null;  // Quality assessment: 'Good', 'Partial', 'Poor', or null
}
```

**Field Descriptions**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `answer` | `string` | Yes | Generated answer or refusal message (e.g., "Not available in the book") |
| `sources` | `string[]` | Yes | List of source chunk IDs (e.g., `['chunk-123', 'chunk-456']`) |
| `matched_chunks` | `ChunkReference[]` | Yes | Detailed chunk data for display |
| `grounded` | `boolean` | Yes | True if answer is from book content, false otherwise |
| `retrieval_quality` | `string \| null` | Yes | Retrieval quality ('Good', 'Partial', 'Poor') or null for selected text |

**Validation Rules** (backend guarantees, documented for frontend):
- `answer` must not be empty
- `sources` length must match `matched_chunks` length
- If `grounded === false`, `answer` should be refusal message
- `retrieval_quality` must be `'Good'`, `'Partial'`, `'Poor'`, or `null`

**Example**:
```typescript
// Successful response
{
  answer: 'ROS 2 provides real-time capabilities, improved security features, and enhanced cross-platform support.',
  sources: ['chunk-123', 'chunk-456'],
  matched_chunks: [
    {
      chunk_id: 'chunk-123',
      text: 'ROS 2 introduces several key improvements...',
      page: 45,
      chapter: 'Chapter 3',
      section: 'ROS 2 Overview'
    },
    {
      chunk_id: 'chunk-456',
      text: 'Security is a major focus in ROS 2...',
      page: 47,
      chapter: 'Chapter 3',
      section: 'Security Features'
    }
  ],
  grounded: true,
  retrieval_quality: 'Good'
}

// Refusal response
{
  answer: 'This information is not available in the book.',
  sources: [],
  matched_chunks: [],
  grounded: false,
  retrieval_quality: null
}
```

---

## Entity Relationships

```
ChatMessage (user) ──────────────> ChatMessage (assistant)
                                           │
                                           ├──> SourceReference (many)
                                           │
                                           └──> ChunkReference (many)

SelectedTextContext ──> AskRequest.selected_text

AskRequest ──> Backend /ask ──> AskResponse
                                    │
                                    ├──> ChatMessage (assistant)
                                    │
                                    └──> ChunkReference (many)
```

**Flow**:
1. User selects text → `SelectedTextContext` created
2. User enters question → `AskRequest` constructed (with or without `selected_text`)
3. Backend responds → `AskResponse` received
4. Response converted → `ChatMessage` (assistant role) with `sources` and `chunks`
5. Display → `AnswerDisplay` renders message with `SourcesList` and `ChunkDisplay`

---

## State Management

### React Context State

```typescript
interface ChatbotState {
  messages: ChatMessage[];             // Conversation history (max 100 messages)
  loading: boolean;                    // Is request in progress?
  error: string | null;                // Current error message (if any)
  selectedText: SelectedTextContext | null; // Current text selection (if any)
}
```

**State Transitions**:

| Event | State Change |
|-------|--------------|
| User sends query | `loading: true`, add user `ChatMessage` |
| Backend responds (success) | `loading: false`, add assistant `ChatMessage` with sources/chunks |
| Backend responds (error) | `loading: false`, add assistant `ChatMessage` with error |
| User selects text | `selectedText: SelectedTextContext` |
| User clears selection | `selectedText: null` |
| User clears error | `error: null` |

---

## Validation Summary

### Client-Side Validation (Before API Call)

1. **Empty Query**: Check `question.trim().length > 0`
2. **Question Length**: Check `question.length <= 1000`
3. **Selected Text Length**: Check `selected_text.length <= 2000` (spec SC-007)

### Backend Validation (Documented for Awareness)

1. **Question**: 1-1000 chars, not empty, trimmed
2. **Selected Text**: Max 10000 chars, trimmed if provided
3. **Response Fields**: All required fields present, types correct

---

## TypeScript Type Definitions File

All entities will be defined in `my-book/src/types/chatbot.ts`:

```typescript
// my-book/src/types/chatbot.ts

export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  selectedTextUsed?: string;
  sources?: SourceReference[];
  chunks?: ChunkReference[];
  error?: string;
}

export interface SourceReference {
  chunkId: string;
  label: string;
  path?: string;
  anchor?: string;
}

export interface ChunkReference {
  chunk_id: string;
  text: string;
  page: number | null;
  chapter: string | null;
  section: string | null;
}

export interface SelectedTextContext {
  text: string;
  startOffset: number;
  endOffset: number;
  capturedAt: Date;
}

export interface AskRequest {
  question: string;
  selected_text?: string;
}

export interface AskResponse {
  answer: string;
  sources: string[];
  matched_chunks: ChunkReference[];
  grounded: boolean;
  retrieval_quality: string | null;
}

export interface ChatbotState {
  messages: ChatMessage[];
  loading: boolean;
  error: string | null;
  selectedText: SelectedTextContext | null;
}
```

---

## Next Steps

1. Generate OpenAPI contract documenting the backend `/ask` endpoint
2. Create quickstart.md with setup and testing instructions
3. Run `/sp.tasks` to generate implementation tasks
