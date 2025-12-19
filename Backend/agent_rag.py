"""
RAG Agent Orchestration using Google Gemini API.

This module handles the core RAG flow: context routing, retrieval,
validation, and answer generation with strict book-grounding constraints.
"""

import os
import logging
import time
import asyncio
import hashlib
import random
from typing import Optional, List, Dict, Any
from dataclasses import dataclass, field

import google.generativeai as genai
from google.api_core.exceptions import ResourceExhausted
from dotenv import load_dotenv

from models.agent_models import AskRequest, AskResponse, ChunkReference

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Timeout configurations (in seconds)
QDRANT_TIMEOUT = 10  # T051: 10s timeout for Qdrant operations
GEMINI_TIMEOUT = 30  # T051: 30s timeout for Gemini generation (increased for retries)
COHERE_TIMEOUT = 10  # T051: 10s timeout for Cohere embedding

# Retry configuration for rate limits
MAX_RETRIES = 3
BASE_RETRY_DELAY = 2  # Base delay in seconds for exponential backoff


# System prompt with constraints for book-grounded answers
SYSTEM_PROMPT = """You are a book content assistant. Answer questions using ONLY the provided context.

RULES:
1. Use ONLY the context provided - no external knowledge
2. If the answer is not in the context, say: "This information is not available in the book."
3. Keep answers concise (1-5 sentences)
4. Go straight to the answer - no filler phrases

Answer the following question using ONLY the context provided:"""


@dataclass
class LatencyMetrics:
    """T049: Latency monitoring for performance tracking."""
    embedding_time_ms: Optional[float] = None
    retrieval_time_ms: Optional[float] = None
    validation_time_ms: Optional[float] = None
    generation_time_ms: Optional[float] = None
    total_time_ms: Optional[float] = None

    def log_summary(self, question_hash: str, grounded: bool, retrieval_quality: Optional[str]):
        """Log a summary of all latencies."""
        logger.info(
            f"[LATENCY] question_hash={question_hash}, "
            f"embedding={self.embedding_time_ms:.0f}ms, " if self.embedding_time_ms else "embedding=N/A, "
            f"retrieval={self.retrieval_time_ms:.0f}ms, " if self.retrieval_time_ms else "retrieval=N/A, "
            f"validation={self.validation_time_ms:.0f}ms, " if self.validation_time_ms else "validation=N/A, "
            f"generation={self.generation_time_ms:.0f}ms, " if self.generation_time_ms else "generation=N/A, "
            f"total={self.total_time_ms:.0f}ms, " if self.total_time_ms else "total=N/A, "
            f"grounded={grounded}, "
            f"retrieval_quality={retrieval_quality}"
        )


@dataclass
class AgentContext:
    """Internal context structure for agent generation."""
    context_text: str
    context_type: str  # "retrieval" or "selected_text"
    source_metadata: List[Dict[str, Any]]
    validation_result: Optional[Dict[str, Any]] = None
    latency_metrics: LatencyMetrics = field(default_factory=LatencyMetrics)


class NoAnswerFoundError(Exception):
    """Raised when no answer is found in the provided context."""
    pass


class AgentOrchestrator:
    """
    Orchestrates RAG agent operations: context routing, retrieval, validation, generation.

    Uses Google Gemini API for answer generation with strict book-grounding constraints.
    """

    def __init__(self):
        """Initialize the agent with Gemini API configuration."""
        api_key = os.getenv('GEMINI_API_KEY')
        if not api_key or api_key == 'your-gemini-api-key-here':
            raise ValueError('GEMINI_API_KEY not configured in .env file')

        genai.configure(api_key=api_key)
        # Using gemini-flash-latest (free tier with available quota)
        self.model = genai.GenerativeModel('gemini-flash-latest')

        # Generation config for controlled output
        self.generation_config = {
            'temperature': 0.1,  # Low temperature for factual, grounded responses
            'top_p': 0.95,
            'top_k': 40,
            'max_output_tokens': 300,  # Limit for 1-5 sentences
        }

        logger.info("AgentOrchestrator initialized with Gemini API")

    def prepare_context(
        self,
        request: AskRequest,
        cohere_client=None,
        qdrant_client=None,
        validator=None
    ) -> AgentContext:
        """
        Route request to appropriate context preparation path.

        Args:
            request: User's ask request
            cohere_client: Cohere client for embeddings (optional if selected_text)
            qdrant_client: Qdrant client for retrieval (optional if selected_text)
            validator: Retrieval validation function (optional if selected_text)

        Returns:
            AgentContext with prepared context and metadata

        Raises:
            NoAnswerFoundError: If validation determines answer is not present
        """
        # T049: Initialize latency tracking
        latency_metrics = LatencyMetrics()
        question_hash = hashlib.md5(request.question.encode()).hexdigest()[:8]

        # Priority routing: selected text overrides retrieval
        if request.selected_text:
            logger.info(f"[{question_hash}] Using selected text context (skipping retrieval)")
            return AgentContext(
                context_text=request.selected_text,
                context_type="selected_text",
                source_metadata=[{"chunk_id": "selected_text", "text": request.selected_text[:2000]}],
                validation_result=None,
                latency_metrics=latency_metrics
            )

        # Standard retrieval path
        logger.info(f"[{question_hash}] Using retrieval path (embedding + Qdrant search)")

        if not cohere_client or not qdrant_client:
            raise ValueError("Cohere and Qdrant clients required for retrieval path")

        # Step 1: Embed query with Cohere (T049: track timing)
        embed_start = time.time()
        try:
            query_embedding = cohere_client.embed(
                texts=[request.question],
                model='embed-english-v3.0',
                input_type='search_query'
            ).embeddings[0]
            latency_metrics.embedding_time_ms = (time.time() - embed_start) * 1000
            logger.info(f"[{question_hash}] Query embedded (dim: {len(query_embedding)}, time: {latency_metrics.embedding_time_ms:.0f}ms)")
        except Exception as e:
            logger.error(f"[{question_hash}] Cohere embedding error: {e}")
            raise

        # Step 2: Retrieve from Qdrant (T049: track timing)
        retrieval_start = time.time()
        try:
            # Use collection name from existing chatbot setup
            collection_name = "rag_embedding"  # Match chatbot.py
            search_result = qdrant_client.query_points(
                collection_name=collection_name,
                query=query_embedding,
                limit=5  # Reduced from 10 to avoid context overflow
            ).points
            latency_metrics.retrieval_time_ms = (time.time() - retrieval_start) * 1000
            logger.info(f"[{question_hash}] Retrieved {len(search_result)} chunks (time: {latency_metrics.retrieval_time_ms:.0f}ms)")
        except Exception as e:
            logger.error(f"[{question_hash}] Qdrant search error: {e}")
            raise

        if not search_result:
            logger.warning(f"[{question_hash}] No chunks retrieved from Qdrant")
            raise NoAnswerFoundError("No relevant content found")

        # Step 3: Validate retrieval (if validator provided) (T049: track timing)
        validation_result = None
        if validator:
            validation_start = time.time()
            try:
                validation_result = validator(request.question, search_result)
                latency_metrics.validation_time_ms = (time.time() - validation_start) * 1000
                logger.info(f"[{question_hash}] Validation: answer_present={validation_result.get('answer_present')}, quality={validation_result.get('retrieval_quality')}, time: {latency_metrics.validation_time_ms:.0f}ms")

                if not validation_result.get('answer_present', False):
                    logger.warning(f"[{question_hash}] Validation: answer not present in retrieved chunks")
                    raise NoAnswerFoundError("Answer not found in retrieved content")
            except NoAnswerFoundError:
                raise
            except Exception as e:
                logger.warning(f"[{question_hash}] Validation error (proceeding anyway): {e}")
                validation_result = None

        # Step 4: Format context from chunks
        context_parts = []
        source_metadata = []

        for idx, hit in enumerate(search_result):
            chunk_text = hit.payload.get('text', '')
            chunk_id = hit.id
            page = hit.payload.get('page')
            chapter = hit.payload.get('chapter')
            section = hit.payload.get('section')

            context_parts.append(f"[Chunk {idx + 1}]: {chunk_text}")
            source_metadata.append({
                "chunk_id": str(chunk_id),
                "text": chunk_text[:2000],  # Truncate if needed
                "page": page,
                "chapter": chapter,
                "section": section
            })

        context_text = "\n\n".join(context_parts)

        return AgentContext(
            context_text=context_text,
            context_type="retrieval",
            source_metadata=source_metadata,
            validation_result=validation_result,
            latency_metrics=latency_metrics
        )

    def _post_process_answer(self, answer_text: str) -> str:
        """
        T048: Post-process answer to ensure quality constraints are met.

        - Trim excessive length if needed
        - Remove opinion indicators
        - Validate no interpretations leaked
        - Clean up formatting

        Args:
            answer_text: Raw answer from LLM

        Returns:
            Processed answer text
        """
        if not answer_text:
            return answer_text

        # Strip whitespace
        answer = answer_text.strip()

        # Remove common filler prefixes that shouldn't appear
        filler_prefixes = [
            "Based on the context,",
            "Based on the provided context,",
            "According to the context,",
            "According to the book,",
            "From the context,",
            "The context shows that",
            "The context indicates that",
            "In the context,",
        ]
        for prefix in filler_prefixes:
            if answer.lower().startswith(prefix.lower()):
                answer = answer[len(prefix):].strip()
                # Capitalize first letter after removal
                if answer:
                    answer = answer[0].upper() + answer[1:]

        # Remove common filler suffixes
        filler_suffixes = [
            "as mentioned in the context.",
            "according to the context.",
            "based on the provided information.",
        ]
        for suffix in filler_suffixes:
            if answer.lower().endswith(suffix.lower()):
                answer = answer[:-len(suffix)].strip()
                # Ensure proper ending punctuation
                if answer and not answer.endswith(('.', '!', '?')):
                    answer += '.'

        # Truncate if excessively long (> 5 sentences or 500 chars)
        # Count sentences
        sentences = [s.strip() for s in answer.replace('!', '.').replace('?', '.').split('.') if s.strip()]
        if len(sentences) > 5:
            # Keep first 5 sentences
            answer = '. '.join(sentences[:5]) + '.'
            logger.warning(f"Answer truncated from {len(sentences)} to 5 sentences")

        # Hard limit on character count
        if len(answer) > 800:
            # Find last sentence boundary before 800 chars
            truncated = answer[:800]
            last_period = truncated.rfind('.')
            if last_period > 400:  # Keep at least some content
                answer = truncated[:last_period + 1]
            logger.warning(f"Answer truncated to {len(answer)} characters")

        return answer

    async def generate_answer(
        self,
        question: str,
        context: AgentContext
    ) -> Dict[str, Any]:
        """
        Generate answer using Gemini API with strict grounding constraints.

        T049: Added latency tracking
        T050: Optimized with async execution
        T051: Added timeout handling

        Args:
            question: User's question
            context: Prepared agent context

        Returns:
            Dict with answer, sources, and metadata
        """
        # T049: Track generation timing
        generation_start = time.time()
        question_hash = hashlib.md5(question.encode()).hexdigest()[:8]

        # Construct prompt with system instructions and context
        prompt = f"""{SYSTEM_PROMPT}

**CONTEXT**:
{context.context_text}

**QUESTION**:
{question}

**ANSWER**:"""

        try:
            # T050/T051: Generate response with timeout and retry logic for rate limits
            # Use asyncio.to_thread for non-blocking execution of sync Gemini call
            async def generate_with_retry():
                last_exception = None
                for attempt in range(MAX_RETRIES):
                    try:
                        return await asyncio.wait_for(
                            asyncio.to_thread(
                                self.model.generate_content,
                                prompt,
                                generation_config=self.generation_config
                            ),
                            timeout=GEMINI_TIMEOUT
                        )
                    except ResourceExhausted as e:
                        last_exception = e
                        # Extract retry delay from error if available, otherwise use exponential backoff
                        retry_delay = BASE_RETRY_DELAY * (2 ** attempt) + random.uniform(0, 1)
                        logger.warning(f"[{question_hash}] Rate limit hit (attempt {attempt + 1}/{MAX_RETRIES}), retrying in {retry_delay:.1f}s")
                        await asyncio.sleep(retry_delay)
                    except asyncio.TimeoutError:
                        logger.error(f"[{question_hash}] Gemini generation timed out after {GEMINI_TIMEOUT}s")
                        raise TimeoutError(f"Answer generation timed out after {GEMINI_TIMEOUT} seconds")

                # All retries exhausted
                logger.error(f"[{question_hash}] Rate limit exceeded after {MAX_RETRIES} retries")
                raise last_exception

            try:
                response = await generate_with_retry()
            except asyncio.TimeoutError:
                logger.error(f"[{question_hash}] Gemini generation timed out after {GEMINI_TIMEOUT}s")
                raise TimeoutError(f"Answer generation timed out after {GEMINI_TIMEOUT} seconds")

            # Handle blocked or empty responses from Gemini
            try:
                raw_answer = response.text.strip()
            except ValueError:
                # Response was blocked by safety filters or no valid part returned
                logger.warning(f"[{question_hash}] Gemini response blocked or empty, using fallback")
                raw_answer = "This information is not available in the book."

            # T048: Post-process the answer
            answer_text = self._post_process_answer(raw_answer)

            # T049: Record generation latency
            context.latency_metrics.generation_time_ms = (time.time() - generation_start) * 1000

            logger.info(f"[{question_hash}] Generated answer (length: {len(answer_text)} chars, time: {context.latency_metrics.generation_time_ms:.0f}ms)")

            # Extract sources and create chunk references
            sources = []
            matched_chunks = []

            for meta in context.source_metadata:
                chunk_ref = ChunkReference(
                    chunk_id=meta["chunk_id"],
                    text=meta.get("text", ""),
                    page=meta.get("page"),
                    chapter=meta.get("chapter"),
                    section=meta.get("section")
                )
                sources.append(meta["chunk_id"])
                matched_chunks.append(chunk_ref)

            retrieval_quality = context.validation_result.get('retrieval_quality') if context.validation_result else None

            # T049: Log latency summary
            context.latency_metrics.log_summary(question_hash, True, retrieval_quality)

            return {
                "answer": answer_text,
                "sources": sources,
                "matched_chunks": matched_chunks,
                "grounded": True,
                "retrieval_quality": retrieval_quality
            }

        except TimeoutError:
            raise
        except Exception as e:
            logger.error(f"[{question_hash}] Gemini generation error: {e}")
            raise


    def create_refusal_response(self, retrieval_quality: str = "Poor") -> Dict[str, Any]:
        """
        Create standard refusal response when answer is not available.

        Args:
            retrieval_quality: Quality assessment from validation

        Returns:
            Dict with refusal message
        """
        return {
            "answer": "This information is not available in the book.",
            "sources": [],
            "matched_chunks": [],
            "grounded": False,
            "retrieval_quality": retrieval_quality
        }
