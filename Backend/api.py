"""
FastAPI server for Book RAG Chatbot with Retrieval Validation

This provides a REST API for the chatbot with hallucination prevention.

API Endpoints:
- POST /ask: Main endpoint for asking questions about book content
- POST /query: Legacy query endpoint with retrieval validation
- GET /health: Health check
- GET /stats: Qdrant collection statistics
- GET /docs: Interactive API documentation (Swagger UI)
"""

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field
from typing import Optional
import logging

from chatbot import query_chatbot, cohere_client, qdrant_client
from retrieval_validation import validate_retrieval
from agent_rag import AgentOrchestrator, NoAnswerFoundError
from models.agent_models import AskRequest, AskResponse
from rate_limiter import get_rate_limiter

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# T057: Enhanced API documentation
API_DESCRIPTION = """
## Book RAG Chatbot API

A controlled RAG (Retrieval-Augmented Generation) agent that answers user questions
**strictly from book content** with 0% hallucination rate.

### Key Features

- **Strict Book-Grounding**: Answers only from retrieved book chunks or user-selected text
- **Hallucination Prevention**: Multi-layered constraints ensure 0% hallucination rate
- **Selected Text Override**: Users can highlight specific text for focused Q&A
- **Source Attribution**: Every answer includes chunk IDs, page numbers, and matched text

### Main Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/ask` | POST | Ask questions about book content (main endpoint) |
| `/query` | POST | Legacy query with retrieval validation metrics |
| `/health` | GET | Health check |
| `/stats` | GET | Qdrant collection statistics |

### Error Handling

The API returns structured error responses:
- `400 Bad Request`: Invalid input (empty question, etc.)
- `500 Internal Server Error`: Service unavailable (Qdrant, Gemini)

### Refusal Behavior

When the answer is not available in the book, the API returns:
```json
{
  "answer": "This information is not available in the book.",
  "sources": [],
  "matched_chunks": [],
  "grounded": false
}
```
"""

# Create FastAPI app with enhanced documentation
app = FastAPI(
    title="Book RAG Chatbot API",
    description=API_DESCRIPTION,
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc",
    openapi_tags=[
        {
            "name": "ask",
            "description": "Main endpoint for asking questions about book content"
        },
        {
            "name": "query",
            "description": "Legacy query endpoint with retrieval validation metrics"
        },
        {
            "name": "health",
            "description": "Health and status endpoints"
        }
    ]
)

# Add CORS middleware
# T058 Security: Restrict CORS in production - configure via environment variable
import os
ALLOWED_ORIGINS = os.getenv("ALLOWED_ORIGINS", "http://localhost:3000,http://127.0.0.1:3000,http://localhost:3001,http://127.0.0.1:3001,http://localhost:8000,http://127.0.0.1:8000").split(",")

app.add_middleware(
    CORSMiddleware,
    allow_origins=ALLOWED_ORIGINS,  # Restricted to specific origins
    allow_credentials=True,
    allow_methods=["GET", "POST"],  # Only needed methods
    allow_headers=["Content-Type", "Authorization"],  # Only needed headers
)


# Request/Response models
class QueryRequest(BaseModel):
    """Request model for chatbot queries"""
    query: str = Field(..., min_length=1, max_length=500, description="User's question")
    top_k: Optional[int] = Field(default=10, ge=1, le=50, description="Number of chunks to retrieve")
    include_evidence: Optional[bool] = Field(default=True, description="Include evidence quotes in response")

    class Config:
        json_schema_extra = {
            "example": {
                "query": "What are the key features of ROS 2?",
                "top_k": 10,
                "include_evidence": True
            }
        }


class QueryResponse(BaseModel):
    """Response model for chatbot queries"""
    answer: str = Field(..., description="Generated answer or error message")
    retrieval_quality: str = Field(..., description="Quality rating: Good, Partial, or Poor")
    relevant_chunks_count: int = Field(..., description="Number of relevant chunks found")
    evidence: list[str] = Field(default_factory=list, description="Evidence quotes (if requested)")
    processing_time_ms: float = Field(..., description="Total processing time in milliseconds")
    error: Optional[str] = Field(None, description="Error message if query failed")

    class Config:
        json_schema_extra = {
            "example": {
                "answer": "ROS 2 provides real-time capabilities, improved security, and cross-platform support...",
                "retrieval_quality": "Good",
                "relevant_chunks_count": 3,
                "evidence": [
                    "ROS 2 provides real-time capabilities...",
                    "Key features include security and cross-platform support..."
                ],
                "processing_time_ms": 342.5,
                "error": None
            }
        }


class HealthResponse(BaseModel):
    """Health check response"""
    status: str
    message: str


# API endpoints
@app.get("/", response_model=dict)
async def root():
    """Root endpoint with API information"""
    return {
        "name": "Book RAG Chatbot API",
        "version": "1.0.0",
        "description": "RAG chatbot with retrieval validation",
        "endpoints": {
            "health": "/health",
            "query": "/query (POST)",
            "docs": "/docs"
        }
    }


@app.get("/health", response_model=HealthResponse, tags=["health"])
async def health_check():
    """Health check endpoint to verify API is running."""
    return {
        "status": "healthy",
        "message": "Book RAG Chatbot API is running"
    }


@app.post(
    "/ask",
    response_model=AskResponse,
    tags=["ask"],
    summary="Ask a question about the book",
    response_description="Answer with sources and grounding information",
    responses={
        200: {
            "description": "Successful response with grounded answer or refusal",
            "content": {
                "application/json": {
                    "examples": {
                        "grounded_answer": {
                            "summary": "Grounded answer from book",
                            "value": {
                                "answer": "ROS 2 is a robotics middleware framework.",
                                "sources": ["chunk_123"],
                                "matched_chunks": [
                                    {
                                        "chunk_id": "chunk_123",
                                        "text": "ROS 2 is a set of software libraries...",
                                        "page": 1,
                                        "chapter": "Chapter 1",
                                        "section": "Introduction"
                                    }
                                ],
                                "grounded": True,
                                "retrieval_quality": "Good"
                            }
                        },
                        "refusal": {
                            "summary": "Refusal when answer not in book",
                            "value": {
                                "answer": "This information is not available in the book.",
                                "sources": [],
                                "matched_chunks": [],
                                "grounded": False,
                                "retrieval_quality": "Poor"
                            }
                        }
                    }
                }
            }
        },
        400: {
            "description": "Invalid input (empty question)",
            "content": {
                "application/json": {
                    "example": {"detail": "Please provide a question about the book."}
                }
            }
        },
        500: {
            "description": "Service unavailable",
            "content": {
                "application/json": {
                    "examples": {
                        "qdrant_error": {
                            "summary": "Qdrant unavailable",
                            "value": {"detail": "Unable to search the book. Please try again later."}
                        },
                        "generation_error": {
                            "summary": "Generation error",
                            "value": {"detail": "Unable to generate answer. Please try again."}
                        }
                    }
                }
            }
        }
    }
)
async def ask(request: AskRequest):
    """
    Ask a question about the book using the RAG agent.

    This endpoint implements the controlled RAG agent with 0% hallucination guarantee:

    **Flow:**
    1. If `selected_text` provided → answer only from selection (no retrieval)
    2. Otherwise → embed query → retrieve from Qdrant → validate → generate answer
    3. Return exact refusal message if answer not found in book content

    **Key Behaviors:**
    - Answers are 1-5 sentences, concise and factual
    - Every answer includes source references
    - Out-of-scope questions return the refusal message
    - Selected text takes absolute priority over retrieval

    **Example Request:**
    ```json
    {
        "question": "What is ROS 2?",
        "selected_text": null
    }
    ```
    """
    import time
    start_time = time.time()

    try:
        logger.info(f"Received /ask request: question='{request.question[:50]}...', has_selected_text={bool(request.selected_text)}")

        # Check rate limiter before processing
        rate_limiter = get_rate_limiter()
        can_proceed, rate_limit_error = await rate_limiter.acquire()

        if not can_proceed:
            logger.warning(f"Request rate limited: {rate_limit_error}")
            raise HTTPException(
                status_code=429,
                detail=rate_limit_error or "Too many requests. Please wait a moment."
            )

        # Initialize agent orchestrator
        orchestrator = AgentOrchestrator()

        # Prepare context (routing: selected text vs retrieval)
        try:
            # Validator wrapper to convert Qdrant ScoredPoint objects to dict format
            def qdrant_to_validation_chunks(query, qdrant_points):
                """Convert Qdrant ScoredPoint objects to validation-compatible dicts."""
                chunks = [
                    {
                        "chunk_id": str(point.id),
                        "text": point.payload.get("text", ""),
                        "metadata": {
                            "chapter": point.payload.get("chapter"),
                            "section": point.payload.get("section"),
                            "source_url": point.payload.get("source_url", ""),
                        },
                        "score": point.score if hasattr(point, 'score') else None
                    }
                    for point in qdrant_points
                ]
                return validate_retrieval(query, chunks)

            context = orchestrator.prepare_context(
                request,
                cohere_client=cohere_client,
                qdrant_client=qdrant_client,
                validator=qdrant_to_validation_chunks
            )
            logger.info(f"Context prepared: type={context.context_type}, sources={len(context.source_metadata)}")

        except NoAnswerFoundError as e:
            # Validation determined answer not present → return refusal
            logger.info(f"No answer found: {e}")
            response_data = orchestrator.create_refusal_response()
            duration_ms = (time.time() - start_time) * 1000
            logger.info(f"Returning refusal response (latency: {duration_ms:.0f}ms)")
            return AskResponse(**response_data)

        except Exception as e:
            # Retrieval/validation error → return appropriate error
            logger.error(f"Context preparation error: {e}", exc_info=True)

            if "Qdrant" in str(e) or "search" in str(e).lower():
                raise HTTPException(
                    status_code=500,
                    detail="Unable to search the book. Please try again later."
                )
            elif "Cohere" in str(e) or "embed" in str(e).lower():
                raise HTTPException(
                    status_code=500,
                    detail="Unable to process your question. Please try again."
                )
            else:
                raise HTTPException(
                    status_code=500,
                    detail="An error occurred. Please try again."
                )

        # Generate answer with agent
        try:
            result = await orchestrator.generate_answer(
                question=request.question,
                context=context
            )

            duration_ms = (time.time() - start_time) * 1000
            logger.info(
                f"Answer generated successfully - "
                f"grounded={result['grounded']}, "
                f"sources={len(result['sources'])}, "
                f"latency={duration_ms:.0f}ms"
            )

            return AskResponse(**result)

        except Exception as e:
            logger.error(f"Answer generation error: {e}", exc_info=True)
            # Check for rate limit / quota exceeded errors
            error_str = str(e).lower()
            if "429" in str(e) or "quota" in error_str or "rate" in error_str or "resourceexhausted" in error_str:
                # Mark rate limiter as limited to prevent further requests
                rate_limiter = get_rate_limiter()
                # Extract retry delay if available, default to 60 seconds
                retry_after = 60.0
                if "retry in" in error_str:
                    try:
                        import re
                        match = re.search(r'retry in (\d+)', error_str)
                        if match:
                            retry_after = float(match.group(1))
                    except Exception:
                        pass
                rate_limiter.record_failure(is_rate_limit=True, retry_after=retry_after)

                raise HTTPException(
                    status_code=429,
                    detail="API rate limit exceeded. Please wait a moment and try again."
                )
            raise HTTPException(
                status_code=500,
                detail="Unable to generate answer. Please try again."
            )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Unexpected error in /ask endpoint: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail="An unexpected error occurred. Please try again."
        )


@app.post("/query", response_model=QueryResponse, tags=["query"], summary="Query with retrieval validation (legacy)")
async def query(request: QueryRequest):
    """
    Query the RAG chatbot with retrieval validation (legacy endpoint).

    This endpoint provides detailed retrieval validation metrics:
    1. Retrieves relevant chunks from Qdrant
    2. Validates retrieval results (relevance, answer presence, quality)
    3. Generates answer only if validation passes
    4. Returns answer with quality metrics and evidence

    **Note:** For new integrations, prefer the `/ask` endpoint.
    """
    try:
        logger.info(f"Received query: {request.query}")

        # Query chatbot with validation
        response = query_chatbot(
            query=request.query,
            top_k=request.top_k,
            include_evidence=request.include_evidence
        )

        logger.info(
            f"Query complete - Quality: {response['retrieval_quality']}, "
            f"Time: {response['processing_time_ms']:.2f}ms"
        )

        return QueryResponse(**response)

    except Exception as e:
        # T058 Security: Don't leak internal error details to clients
        logger.error(f"Query failed: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail="Query processing failed. Please try again later."
        )


@app.get("/rate-limit-status", response_model=dict, tags=["health"], summary="Get rate limiter status")
async def rate_limit_status():
    """
    Get current rate limiter status.

    Returns information about API usage and rate limiting status.
    """
    rate_limiter = get_rate_limiter()
    return rate_limiter.get_status()


@app.get("/stats", response_model=dict, tags=["health"], summary="Get Qdrant collection statistics")
async def stats():
    """
    Get collection statistics from Qdrant.

    Returns information about the indexed book content including
    total chunks, vector size, and collection status.
    """
    try:
        from chatbot import qdrant_client, COLLECTION_NAME

        collection_info = qdrant_client.get_collection(COLLECTION_NAME)

        return {
            "collection": COLLECTION_NAME,
            "total_chunks": collection_info.points_count,
            "vector_size": 1024,
            "distance_metric": "COSINE",
            "status": "ready"
        }

    except Exception as e:
        # T058 Security: Don't leak internal error details to clients
        logger.error(f"Stats retrieval failed: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail="Failed to retrieve stats. Please try again later."
        )


# Error handlers
@app.exception_handler(404)
async def not_found_handler(request: Request, exc):
    """Handle 404 errors"""
    return JSONResponse(
        status_code=404,
        content={
            "error": "Not Found",
            "message": "The requested endpoint does not exist",
            "available_endpoints": ["/", "/health", "/query", "/ask", "/stats", "/docs"]
        }
    )


@app.exception_handler(500)
async def internal_error_handler(request: Request, exc):
    """Handle 500 errors"""
    # T058 Security: Don't leak internal error details to clients
    logger.error(f"Internal error: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={
            "error": "Internal Server Error",
            "message": "An unexpected error occurred. Please try again later."
        }
    )


# Startup/shutdown events
@app.on_event("startup")
async def startup_event():
    """Run on application startup"""
    logger.info("=" * 60)
    logger.info("Book RAG Chatbot API Starting...")
    logger.info("=" * 60)
    logger.info("Endpoints available at:")
    logger.info("  - Health Check: http://127.0.0.1:8000/health")
    logger.info("  - Query: http://127.0.0.1:8000/query (POST)")
    logger.info("  - Stats: http://127.0.0.1:8000/stats")
    logger.info("  - Docs: http://127.0.0.1:8000/docs")
    logger.info("=" * 60)


@app.on_event("shutdown")
async def shutdown_event():
    """Run on application shutdown"""
    logger.info("Book RAG Chatbot API shutting down...")


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "api:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
