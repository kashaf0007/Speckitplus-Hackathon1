---
id: 002
title: Book RAG Chatbot Constitution Creation
stage: constitution
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-book-rag-chatbot
user: Kashaf Aman
command: /sp.constitution
labels: ["constitution", "rag", "chatbot", "book-only", "initial-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Prompt

1. Purpose

The chatbot is embedded inside the published book to help readers by answering questions only from the book's content.

2. Core Rules

The chatbot must not use external knowledge

If the answer is not in the book, it must reply:
"This information is not available in the book."

User-selected text has highest priority

No guessing, no hallucination

3. User Interaction

Users can ask questions about:

The whole book

A chapter

Selected text

Answers must be short, clear, and strictly based on retrieved text

4. Technology Stack

Frontend: Embedded chat inside published book

Backend: FastAPI

LLM Agent: OpenAI Agents SDK

Embeddings: Cohere

Vector DB: Qdrant Cloud (Free Tier)

5. RAG Flow

Normal question → Retrieve relevant book chunks from Qdrant → Answer

Selected text question → Answer only from selected text (no retrieval)

6. Agent Rule

Answer only from provided book text.
If not found, say:
"This information is not available in the book."

7. Goal

A strict, book-only, embedded RAG chatbot that improves reading without adding outside knowledge.

## Response snapshot

Created Book RAG Chatbot Constitution v1.0.0 with 7 core principles:

1. **Book-Only Knowledge (NON-NEGOTIABLE)** - Answer exclusively from book content
2. **No Hallucination Policy (NON-NEGOTIABLE)** - Clear "not found" responses when info unavailable
3. **Selected Text Priority (NON-NEGOTIABLE)** - User-selected text takes absolute priority
4. **Concise Response Format** - Short, clear, book-based answers
5. **RAG Flow Separation** - Two distinct paths: normal queries (with retrieval) vs selected text (no retrieval)
6. **Technology Stack Compliance** - FastAPI, OpenAI Agents SDK, Cohere, Qdrant Cloud
7. **Strict Agent Instructions** - Explicit instructions to enforce book-only responses

Added Technology Stack table, RAG Architecture data flow diagram, and Governance section with amendment process, versioning policy, and compliance review requirements.

## Outcome

- ✅ Impact: Initial constitution created establishing all project principles
- 🧪 Tests: N/A (documentation)
- 📁 Files: .specify/memory/constitution.md (created)
- 🔁 Next prompts: /sp.specify to create feature specification
- 🧠 Reflection: User provided clear 7-point requirements; mapped directly to 7 principles with NON-NEGOTIABLE markers for critical rules

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): Standard constitution creation
- Next experiment (smallest change to try): N/A
