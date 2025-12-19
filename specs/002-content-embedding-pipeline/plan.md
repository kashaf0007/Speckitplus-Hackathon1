# Implementation Plan: Content Embedding Pipeline

**Branch**: `002-content-embedding-pipeline` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-content-embedding-pipeline/spec.md`

## Summary

Build a content embedding pipeline that extracts text from the published book website (https://speckitplus-hackathon1.vercel.app/), generates vector embeddings using Cohere, and stores them in Qdrant Cloud for RAG-based retrieval. The pipeline implements the core data ingestion layer required by the Book RAG Chatbot.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: FastAPI, Cohere SDK, Qdrant Client, httpx, BeautifulSoup4, python-dotenv
**Storage**: Qdrant Cloud (Vector Database) - Collection: `rag_embedding`
**Testing**: pytest (manual validation for initial implementation)
**Target Platform**: Linux server / Local development
**Project Type**: Backend service (single project in `Backend/` directory)
**Performance Goals**: Process 100 chunks in <30 seconds, index 300-page book in <5 minutes
**Constraints**: Qdrant Cloud free tier limits, Cohere API rate limits
**Scale/Scope**: Single book (~7 chapters), expandable to multiple books

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Book-Only Knowledge | ✅ PASS | Pipeline indexes book content only |
| II. No Hallucination Policy | ✅ N/A | Embedding pipeline doesn't generate responses |
| III. Selected Text Priority | ✅ N/A | Handled by chatbot, not embedding pipeline |
| IV. Concise Response Format | ✅ N/A | Handled by chatbot, not embedding pipeline |
| V. RAG Flow Separation | ✅ PASS | Pipeline provides data for Path A (retrieval) |
| VI. Technology Stack Compliance | ✅ PASS | Using Cohere + Qdrant as mandated |
| VII. Strict Agent Instructions | ✅ N/A | Handled by chatbot agent |

**Gate Result**: ✅ PASSED - No violations

## Project Structure

### Documentation (this feature)

```text
specs/002-content-embedding-pipeline/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── embedding-api.md # Internal API contracts
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
Backend/
├── main.py              # Main pipeline script with all functions
├── pyproject.toml       # Dependencies (uv package manager)
├── .env                 # Environment variables (API keys)
├── .env.example         # Template for environment variables
└── .venv/               # Virtual environment
```

**Structure Decision**: Single backend project using existing `Backend/` directory. The pipeline is implemented as a standalone script (`main.py`) that can be run directly to ingest content. Future iterations may add FastAPI endpoints for API-based ingestion.

## Complexity Tracking

> No violations requiring justification.

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | - | - |
