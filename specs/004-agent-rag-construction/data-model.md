# Data Model: Agent Construction & RAG Integration

**Feature**: 004-agent-rag-construction
**Date**: 2025-12-18
**Purpose**: Define data structures for agent request/response handling

## Overview

This feature introduces API models for the /ask endpoint and internal data structures for agent orchestration. All models use Pydantic for type safety and validation.

## Request/Response Models

### AskRequest

**Purpose**: Incoming user question with optional selected text context

**Location**: `Backend/models/agent_models.py`

**Fields**:

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| question | str | Yes | min_length=1, max_length=1000 | User's question about the book |
| selected_text | Optional[str] | No | max_length=10000 | User-highlighted passage (overrides retrieval) |

**Validation Rules**:
- `question` must not be empty or whitespace-only
- `question` length capped at 1000 chars to prevent abuse
- `selected_text` if provided, capped at 10000 chars (reasonable passage size)
- Both fields stripped of leading/trailing whitespace

**Example**:
```json
{
  "question": "What is the main theme of Chapter 3?",
  "selected_text": null
}
```

```json
{
  "question": "Explain this passage",
  "selected_text": "The author describes the process of photosynthesis in detail..."
}
```

### AskResponse

**Purpose**: Agent answer with sources and grounding information

**Location**: `Backend/models/agent_models.py`

**Fields**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| answer | str | Yes | Agent's generated answer (1-5 sentences) |
| sources | List[str] | Yes | List of source identifiers (chunk IDs or "selected_text") |
| matched_chunks | List[ChunkReference] | Yes | Full chunk details with text and metadata |
| grounded | bool | Yes | Always True for successful responses (False only for errors) |
| retrieval_quality | Optional[str] | No | "Good"/"Partial"/"Poor" from validation assistant (null if selected text) |

**Validation Rules**:
- `answer` must not be empty
- `sources` must have at least 1 element
- `matched_chunks` must align with `sources` (same count when from retrieval)
- `retrieval_quality` only present when retrieval used (null for selected text)

**Example (standard query)**:
```json
{
  "answer": "Chapter 3 explores the theme of resilience through the protagonist's journey of overcoming adversity.",
  "sources": ["chunk_123", "chunk_124"],
  "matched_chunks": [
    {
      "chunk_id": "chunk_123",
      "text": "The protagonist faces significant challenges in Chapter 3...",
      "page": 45,
      "chapter": "Chapter 3",
      "section": "Rising Action"
    },
    {
      "chunk_id": "chunk_124",
      "text": "Despite setbacks, the character demonstrates remarkable resilience...",
      "page": 47,
      "chapter": "Chapter 3",
      "section": "Climax"
    }
  ],
  "grounded": true,
  "retrieval_quality": "Good"
}
```

**Example (selected text)**:
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

**Example (refusal)**:
```json
{
  "answer": "This information is not available in the book.",
  "sources": [],
  "matched_chunks": [],
  "grounded": false,
  "retrieval_quality": "Poor"
}
```

### ChunkReference

**Purpose**: Detailed information about a content chunk used in the answer

**Location**: `Backend/models/agent_models.py`

**Fields**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| chunk_id | str | Yes | Unique identifier (from Qdrant or "selected_text") |
| text | str | Yes | Full text of the chunk (up to 2000 chars) |
| page | Optional[int] | No | Page number in the book (null for selected text or web content) |
| chapter | Optional[str] | No | Chapter name/number (null if not available) |
| section | Optional[str] | No | Section name (null if not available) |

**Validation Rules**:
- `chunk_id` must not be empty
- `text` must not be empty, max 2000 chars (truncate if longer with "..." indicator)
- `page` if present, must be positive integer
- Metadata fields (chapter, section) can be null

**Example**:
```json
{
  "chunk_id": "chunk_456",
  "text": "The scientific method involves observation, hypothesis formation, experimentation, and analysis.",
  "page": 78,
  "chapter": "Chapter 5: Scientific Inquiry",
  "section": "Methodology"
}
```

## Internal Models

### AgentContext

**Purpose**: Internal data structure passed to OpenAI Agent

**Location**: `Backend/agent_rag.py` (not exported)

**Fields**:

| Field | Type | Description |
|-------|------|-------------|
| context_text | str | Formatted context (retrieved chunks or selected text) |
| context_type | Literal["retrieval", "selected_text"] | Source of context |
| validation_result | Optional[ValidationResult] | Output from retrieval validation assistant (null if selected text) |
| source_metadata | List[Dict] | Metadata for each chunk (chunk_id, page, chapter, section) |

**Purpose**: Encapsulates all context needed for agent to generate answer. Not exposed via API.

### ValidationResult

**Purpose**: Output from retrieval validation assistant (003-retrieval-validation)

**Location**: Imported from `Backend/retrieval_validation/validator.py`

**Fields** (from 003-retrieval-validation spec):

| Field | Type | Description |
|-------|------|-------------|
| relevant_chunks | List[str] | List of relevant chunk IDs |
| answer_present | bool | Whether answer is in retrieved data |
| evidence | List[str] | Quoted lines from chunks |
| retrieval_quality | Literal["Good", "Partial", "Poor"] | Quality assessment |

## Entity Relationships

```
User Request
    ↓
AskRequest (API input)
    ↓
[if selected_text] → AgentContext (selected text) → OpenAI Agent
    ↓
[else] → Cohere Embed → Qdrant Retrieve → ValidationResult
    ↓
AgentContext (retrieved chunks + validation) → OpenAI Agent
    ↓
Agent generates answer
    ↓
AskResponse (API output)
    ├── answer (str)
    ├── sources (List[str])
    ├── matched_chunks (List[ChunkReference])
    ├── grounded (bool)
    └── retrieval_quality (Optional[str])
```

## State Transitions

The agent is stateless - no conversation history. Each request is independent.

**Request Lifecycle**:

1. **Receive Request** → Validate AskRequest schema
2. **Context Routing**:
   - If `selected_text` present → **Selected Text Path**
   - Else → **Retrieval Path**
3. **Selected Text Path**:
   - Create AgentContext with selected_text
   - Skip retrieval and validation
   - → **Agent Generation**
4. **Retrieval Path**:
   - Embed question (Cohere)
   - Retrieve chunks (Qdrant)
   - Validate retrieval (003-retrieval-validation)
   - If `answer_present == False` → **Refusal Response**
   - Else create AgentContext with chunks
   - → **Agent Generation**
5. **Agent Generation**:
   - Initialize OpenAI Agent with system prompt
   - Pass AgentContext
   - Generate answer (JSON structured output)
   - → **Response Construction**
6. **Response Construction**:
   - Extract answer text
   - Map sources (chunk IDs or "selected_text")
   - Create ChunkReference list
   - Set grounded=True
   - Return AskResponse
7. **Refusal Response**:
   - answer = "This information is not available in the book."
   - sources = []
   - matched_chunks = []
   - grounded = False
   - Return AskResponse
8. **Error Handling**:
   - Catch all exceptions
   - Return 500 with error message (no sensitive details)
   - Log full error context for debugging

## Validation Rules Summary

| Entity | Validation |
|--------|------------|
| AskRequest | question non-empty, lengths within limits |
| AskResponse | answer non-empty, sources not empty (unless refusal), chunks align with sources |
| ChunkReference | chunk_id and text non-empty, page positive if present |
| AgentContext | context_text non-empty, source_metadata aligns with context type |

## Data Flow Diagram

```
┌─────────────────┐
│  User Question  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  AskRequest     │
│  - question     │
│  - selected_text│
└────────┬────────┘
         │
    ┌────┴─────┐
    │          │
    ▼          ▼
[Selected]  [Standard]
    │          │
    │      ┌───┴────┐
    │      │ Cohere │
    │      │ Embed  │
    │      └───┬────┘
    │          │
    │      ┌───┴────┐
    │      │ Qdrant │
    │      │Retrieve│
    │      └───┬────┘
    │          │
    │    ┌─────┴─────────┐
    │    │Validation     │
    │    │Assistant      │
    │    └─────┬─────────┘
    │          │
    │      answer_present?
    │       /         \
    │     Yes         No
    │      │          │
    │      │      ┌───┴────┐
    │      │      │Refusal │
    │      │      │Response│
    │      │      └───┬────┘
    │      │          │
    └──────┴──────────┴─────┐
                             │
                    ┌────────▼────────┐
                    │ AgentContext    │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │ OpenAI Agent    │
                    │ Generation      │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │  AskResponse    │
                    │  - answer       │
                    │  - sources      │
                    │  - matched_     │
                    │    chunks       │
                    │  - grounded     │
                    │  - retrieval_   │
                    │    quality      │
                    └─────────────────┘
```

## Database Entities

**Note**: This feature does not introduce new database entities. It uses existing Qdrant collections from 002-content-embedding-pipeline.

**Qdrant Collection Schema** (from 002-content-embedding-pipeline):
- Collection name: `book_content` (assumed)
- Vector dimension: 1024 (Cohere embed-english-v3.0)
- Metadata fields: chunk_id, text, chapter, section, page, source_url

## Next Steps

Proceed to:
1. Create OpenAPI contract (contracts/ask_endpoint.yaml)
2. Generate quickstart.md for developer onboarding
