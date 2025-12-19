# Feature Specification: Agent Construction & RAG Integration

**Feature Branch**: `004-agent-rag-construction`
**Created**: 2025-12-18
**Status**: Draft
**Input**: User description: "Agent Construction & RAG Integration - Build a controlled RAG agent that answers user questions strictly from the book's content."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Answer Questions from Retrieved Book Chunks (Priority: P1)

As a reader, I want to ask questions about the book and receive accurate answers derived strictly from the book's content, so that I can quickly find information without having to search through the entire text manually.

**Why this priority**: This is the core value proposition of the RAG chatbot. Without accurate question answering based on book content, the entire system provides no value to end users.

**Independent Test**: Can be fully tested by submitting a question that has a clear answer in the book, verifying that the agent retrieves relevant chunks from Qdrant, and delivers an answer grounded in those chunks with source references.

**Acceptance Scenarios**:

1. **Given** a user question "What is the main theme of Chapter 3?", **When** the agent processes the question and retrieves relevant chunks from Qdrant, **Then** it returns an answer based solely on Chapter 3 content with chunk IDs and page numbers as sources
2. **Given** a user question about a topic covered in the book, **When** the agent processes the question, **Then** the answer includes direct quotes or paraphrases from retrieved chunks and lists all matched chunk references
3. **Given** a user question that requires information from multiple sections of the book, **When** the agent synthesizes information from multiple retrieved chunks, **Then** the answer integrates information from all relevant chunks and cites each source

---

### User Story 2 - Answer Questions from User-Selected Text (Priority: P1)

As a reader, I want to highlight specific text in the book and ask questions about that selection, so that I can get precise answers about a passage without the system searching elsewhere in the book.

**Why this priority**: Enables focused, context-aware interaction. Users often need clarification about specific passages they're currently reading, making this a high-value interaction pattern.

**Independent Test**: Can be tested by providing a user-selected text passage and a question about it, then verifying the agent answers exclusively from the selection without retrieving additional chunks from Qdrant.

**Acceptance Scenarios**:

1. **Given** user-selected text from page 45 and a question "What does this passage mean?", **When** the agent processes the request, **Then** it answers based only on the selected text without querying Qdrant or using external knowledge
2. **Given** user-selected text containing a definition and a question "Explain this concept", **When** the agent processes the request, **Then** the answer is grounded in the selected text and includes the exact quote as evidence
3. **Given** user-selected text and a question that cannot be answered from the selection alone, **When** the agent processes the request, **Then** it responds "This information is not available in the selected text" without retrieving additional content

---

### User Story 3 - Refuse Out-of-Scope Questions (Priority: P1)

As a system administrator, I need the agent to refuse questions that are not about the book's content, so that users receive only accurate, book-grounded information and the agent doesn't hallucinate or provide external knowledge.

**Why this priority**: Critical for maintaining trust and accuracy. Preventing hallucinations and out-of-scope responses is a non-negotiable requirement for a controlled RAG system.

**Independent Test**: Can be tested by submitting questions clearly outside the book's scope (current events, personal advice, unrelated topics) and verifying the agent responds with the exact refusal message without attempting to answer.

**Acceptance Scenarios**:

1. **Given** a question "What's the weather today?", **When** the agent processes the question, **Then** it responds exactly "This information is not available in the book" without providing external information
2. **Given** a question about a topic definitely not in the book (e.g., "How do I bake a cake?" when the book is about physics), **When** the agent processes the question, **Then** it responds with the refusal message and does not attempt retrieval
3. **Given** a question that seems related but has no answer in retrieved chunks, **When** the agent processes the question and validates retrieval results, **Then** it responds "This information is not available in the book" instead of inferring an answer

---

### User Story 4 - Provide Short, Clear, Factual Answers (Priority: P2)

As a reader, I want concise, direct answers to my questions so that I can quickly understand the information without reading lengthy explanations.

**Why this priority**: User experience optimization. While not core functionality, answer quality directly impacts user satisfaction and adoption.

**Independent Test**: Can be tested by submitting questions and measuring answer length, clarity, and factual accuracy using human evaluation against source chunks.

**Acceptance Scenarios**:

1. **Given** a simple factual question, **When** the agent generates an answer, **Then** the response is 1-3 sentences and directly addresses the question without unnecessary elaboration
2. **Given** a question requiring a detailed explanation, **When** the agent generates an answer, **Then** the response summarizes key points from the book in 3-5 sentences maximum
3. **Given** any question, **When** the agent generates an answer, **Then** the response contains no opinions, interpretations, or information not explicitly stated in the source chunks

---

### Edge Cases

- What happens when the user asks a question but no chunks are retrieved from Qdrant? → Agent responds "This information is not available in the book" without attempting to generate an answer.
- What happens when retrieved chunks are irrelevant (low retrieval quality from validation)? → Agent relies on the validation assistant's assessment and responds with the refusal message if no relevant information is confirmed.
- What happens when the user selects text but asks an unrelated question? → Agent still uses only the selected text (no retrieval) and responds with the refusal message if the answer isn't in the selection.
- What happens when the user selects text that includes the answer but also asks about something outside that selection? → Agent strictly limits its response to the selected text, answering what it can and noting what's not available in the selection.
- What happens when the user submits an empty or malformed question? → Agent requests clarification with a prompt like "Please provide a question about the book."
- What happens when the session times out or the user starts a new conversation? → Agent maintains no memory between sessions; each question is treated as independent with no conversation history.
- What happens when retrieved chunks contain contradictory information? → Agent presents both perspectives as they appear in the book and notes the different viewpoints, citing sources for each.
- What happens when the Qdrant database is unavailable during retrieval? → Agent reports a system error and asks the user to try again later without fabricating information.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST accept a user question (text string) as input via /ask FastAPI endpoint
- **FR-002**: System MUST accept user-selected text as optional override input for context-limited answering
- **FR-003**: System MUST query Qdrant vector database for relevant chunks when no selected text is provided
- **FR-004**: System MUST use Cohere embeddings (matching the content embedding pipeline) for query vectorization
- **FR-005**: System MUST integrate with the retrieval validation assistant (003-retrieval-validation) to assess chunk relevance and answer presence before responding
- **FR-006**: System MUST answer questions using ONLY provided context (retrieved chunks or selected text)
- **FR-007**: System MUST NOT use inference, guessing, or external knowledge to supplement answers
- **FR-008**: System MUST NOT use training data or world knowledge beyond what's in the provided context
- **FR-009**: System MUST respond exactly "This information is not available in the book" when answer is not in the provided context
- **FR-010**: System MUST respond with the refusal message when the question is out of scope (not about the book)
- **FR-011**: System MUST prioritize user-selected text as highest-priority context, skipping Qdrant retrieval when selection is provided
- **FR-012**: System MUST generate short (1-5 sentences), clear, factual answers grounded in the text
- **FR-013**: System MUST include source references (chunk IDs, page numbers, or "selected text") in every answer
- **FR-014**: System MUST include matched chunks (text snippets) in the response for verification
- **FR-015**: System MUST maintain no conversation memory beyond the current single-turn interaction
- **FR-016**: System MUST reset context after each question-answer interaction
- **FR-017**: System MUST use OpenAI Agents SDK for agent construction and orchestration
- **FR-018**: System MUST expose functionality through FastAPI backend at /ask endpoint
- **FR-019**: System MUST validate input questions for emptiness and prompt for clarification if needed
- **FR-020**: System MUST handle Qdrant unavailability gracefully with clear error messages

### Key Entities

- **User Question**: The text query submitted by the reader. Contains the information need and defines the retrieval and answering scope.
- **Selected Text Context**: Optional user-highlighted passage from the book. When provided, overrides Qdrant retrieval and constrains answering to this text only.
- **Query Embedding**: Vector representation of the user question generated using Cohere embeddings. Used for semantic search in Qdrant.
- **Retrieved Chunks**: Set of text segments returned by Qdrant based on query embedding similarity. Contains chunk IDs, text content, metadata (chapter, page, section).
- **Validation Result**: Output from the retrieval validation assistant indicating chunk relevance, answer presence, evidence quotes, and retrieval quality.
- **Agent Answer**: The final response generated by the OpenAI agent. Contains answer text, source references (chunk IDs/pages), matched chunk text snippets, and grounding indicators.
- **RAG Agent**: The OpenAI Agents SDK-based component that orchestrates retrieval, validation, and answer generation while enforcing book-grounding constraints.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users receive answers to questions in under 5 seconds end-to-end (including retrieval and generation)
- **SC-002**: 95% of answers are rated "accurate and relevant" by human evaluators when compared to source chunks
- **SC-003**: 0% hallucination rate measured across 100 test questions (agent never generates information not in source chunks)
- **SC-004**: Agent correctly refuses out-of-scope questions 100% of the time in a test set of 50 questions (25 in-scope, 25 out-of-scope)
- **SC-005**: Selected-text override works correctly 100% of the time (no Qdrant retrieval when selection provided, answer strictly from selection)
- **SC-006**: 100% of answers include source references (chunk IDs, page numbers, or "selected text" indicator)
- **SC-007**: 100% of answers include matched chunks for verification
- **SC-008**: Users rate answer clarity as 4/5 or higher on average (measured via user survey after 2 weeks of usage)
- **SC-009**: Agent maintains zero conversation memory across sequential questions (verified by testing context leakage)
- **SC-010**: System handles Qdrant failures gracefully without crashes or hallucinations 100% of the time

## Assumptions

- The content embedding pipeline (002-content-embedding-pipeline) is complete and Qdrant contains indexed book content with Cohere embeddings
- The retrieval validation assistant (003-retrieval-validation) is operational and returns structured validation results
- Users interact with the agent through a frontend that calls the /ask FastAPI endpoint (frontend design is out of scope)
- The book content is in English; multi-language support is not required initially
- OpenAI API access and rate limits are sufficient for expected query volume
- Cohere API access matches the embedding model used in the content pipeline
- Selected text is provided as plain text string by the frontend when users highlight passages
- Single-turn interactions are sufficient; multi-turn conversation is not required for Phase 1
- Users understand the agent's limitations (book-only knowledge) through frontend messaging
- System operates in a trusted environment; authentication and rate limiting are handled upstream

## Scope

### In Scope
- Building the RAG agent using OpenAI Agents SDK
- Integrating Qdrant retrieval with Cohere embeddings
- Integrating retrieval validation assistant for quality control
- Implementing selected-text override for context-limited answering
- Enforcing strict book-grounding constraints (no hallucinations)
- Generating short, clear, factual answers with source references
- Exposing /ask FastAPI endpoint for question answering
- Handling out-of-scope questions with refusal message
- Single-turn interaction (no conversation memory)
- Error handling for Qdrant unavailability

### Out of Scope
- Frontend/UI development (handled separately)
- Multi-turn conversation and dialogue management
- User authentication and session management
- Rate limiting and abuse prevention
- Conversation history storage
- Answer personalization based on user preferences
- Multi-language support
- Voice or audio input/output
- Real-time streaming responses
- Analytics and usage tracking (handled by monitoring infrastructure)
- Custom prompt engineering per user
- Fine-tuning or training custom models

## Dependencies

- Qdrant vector database must be operational with indexed book content (from 002-content-embedding-pipeline)
- Retrieval validation assistant (003-retrieval-validation) must be available as a callable component
- Cohere API access for query embedding (matching content pipeline embedding model)
- OpenAI API access for agent orchestration and answer generation
- FastAPI framework for backend endpoint
- Frontend or client application to call /ask endpoint (out of scope, but required for end-to-end functionality)

## Constraints

- Must answer ONLY from book content (retrieved chunks or selected text)
- Must maintain 0% hallucination rate (non-negotiable accuracy requirement)
- Must refuse out-of-scope questions explicitly
- Must not retain conversation memory across questions
- Must complete answers within 5-second latency target
- Must use OpenAI Agents SDK (technology constraint per requirements)
- Must use FastAPI for backend (technology constraint per requirements)
- Must use Qdrant with Cohere embeddings (consistency with content pipeline)
- Answer length constrained to 1-5 sentences for clarity
- Selected text takes absolute priority over retrieval when provided
