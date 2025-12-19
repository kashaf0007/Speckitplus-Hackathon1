"""
Book RAG Chatbot - Query Interface with Retrieval Validation

This module provides the query interface for the Book RAG chatbot,
integrating retrieval validation to prevent hallucinations.
"""

import os
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient

# Import our validation module
from retrieval_validation import validate_retrieval, ValidationError, CohereAPIError

load_dotenv()

# Configuration
COLLECTION_NAME = "rag_embedding"
DEFAULT_TOP_K = 10  # Number of chunks to retrieve
RELEVANCE_THRESHOLD = 0.3  # Threshold for relevance

# Initialize clients
cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    timeout=60,  # Increased timeout for slow network
)


def query_chatbot(
    query: str,
    top_k: int = DEFAULT_TOP_K,
    include_evidence: bool = True
) -> dict:
    """
    Query the RAG chatbot with retrieval validation.

    This function orchestrates the complete RAG pipeline:
    1. Generate query embedding
    2. Retrieve chunks from Qdrant
    3. Validate retrieval results
    4. Generate answer (if validation passes)

    Args:
        query: User's question
        top_k: Number of chunks to retrieve (default: 10)
        include_evidence: Whether to include evidence in response

    Returns:
        dict: Response with structure:
            {
                "answer": str,
                "retrieval_quality": str,
                "relevant_chunks_count": int,
                "evidence": list[str] (optional),
                "processing_time_ms": float,
                "error": str (if error occurred)
            }

    Example:
        >>> response = query_chatbot("What are the key features of ROS 2?")
        >>> print(response["answer"])
        >>> print(f"Quality: {response['retrieval_quality']}")
    """
    try:
        print(f"\n{'='*60}")
        print(f"Query: {query}")
        print(f"{'='*60}")

        # Step 1: Generate query embedding
        print("\n[1/4] Generating query embedding...")
        query_embedding = cohere_client.embed(
            texts=[query],
            model="embed-english-v3.0",
            input_type="search_query",
        ).embeddings[0]
        print(f"  [OK] Embedding generated (dimension: {len(query_embedding)})")

        # Step 2: Retrieve chunks from Qdrant
        print(f"\n[2/4] Retrieving top {top_k} chunks from Qdrant...")
        results = qdrant_client.query_points(
            collection_name=COLLECTION_NAME,
            query=query_embedding,
            limit=top_k,
        ).points

        if not results:
            return {
                "answer": "No information found in the knowledge base. Please try a different query.",
                "retrieval_quality": "Poor",
                "relevant_chunks_count": 0,
                "evidence": [],
                "processing_time_ms": 0.0,
                "error": "No chunks retrieved"
            }

        print(f"  [OK] Retrieved {len(results)} chunks")

        # Convert Qdrant results to validation format
        chunks = [
            {
                "chunk_id": str(result.id),
                "text": result.payload.get("text", ""),
                "metadata": {
                    "chapter": result.payload.get("chapter", "Unknown"),
                    "section": result.payload.get("section", "Unknown"),
                    "source_url": result.payload.get("source_url", ""),
                },
                "score": result.score
            }
            for result in results
        ]

        # Step 3: Validate retrieval
        print(f"\n[3/4] Validating retrieval results...")
        try:
            validation_result = validate_retrieval(
                query=query,
                chunks=chunks,
                relevance_threshold=RELEVANCE_THRESHOLD,
                cohere_client=cohere_client
            )

            print(f"  [OK] Validation complete:")
            print(f"    - Relevant chunks: {len(validation_result['relevant_chunks'])}")
            print(f"    - Answer present: {validation_result['answer_present']}")
            print(f"    - Quality: {validation_result['retrieval_quality']}")
            print(f"    - Processing time: {validation_result['processing_time_ms']:.2f}ms")

        except (ValidationError, CohereAPIError) as e:
            print(f"  [WARN] Validation failed: {e}")
            # Fall back to using all chunks without validation
            validation_result = {
                "relevant_chunks": [
                    {"chunk_id": c["chunk_id"], "relevance_score": c.get("score", 0.5)}
                    for c in chunks
                ],
                "answer_present": True,  # Fail open
                "evidence": [],
                "retrieval_quality": "Unknown",
                "quality_reasoning": "Validation unavailable",
                "processing_time_ms": 0.0
            }

        # Check if answer is present
        if not validation_result["answer_present"]:
            return {
                "answer": "No relevant information found in retrieved data. The knowledge base may not contain information about this topic.",
                "retrieval_quality": validation_result["retrieval_quality"],
                "relevant_chunks_count": len(validation_result["relevant_chunks"]),
                "evidence": [],
                "processing_time_ms": validation_result["processing_time_ms"],
                "error": None
            }

        # Check quality
        if validation_result["retrieval_quality"] == "Poor":
            return {
                "answer": "Unable to find sufficient relevant information. Please try rephrasing your query or asking about a different topic.",
                "retrieval_quality": "Poor",
                "relevant_chunks_count": len(validation_result["relevant_chunks"]),
                "evidence": [],
                "processing_time_ms": validation_result["processing_time_ms"],
                "error": None
            }

        # Step 4: Generate answer using validated chunks
        print(f"\n[4/4] Generating answer from validated chunks...")

        # Get texts from relevant chunks only
        relevant_chunk_ids = {
            chunk["chunk_id"] for chunk in validation_result["relevant_chunks"]
        }

        relevant_texts = [
            chunk["text"] for chunk in chunks
            if chunk["chunk_id"] in relevant_chunk_ids
        ]

        # Build context from relevant chunks
        context = "\n\n".join(relevant_texts)

        # For now, return the context as the answer
        # In a full implementation, you would use an LLM to generate a natural language answer
        answer = f"Based on the retrieved information:\n\n{context[:1000]}"  # Limit length
        if len(context) > 1000:
            answer += "\n\n[Answer truncated for brevity]"

        print(f"  [OK] Answer generated (length: {len(answer)} chars)")

        # Build response
        response = {
            "answer": answer,
            "retrieval_quality": validation_result["retrieval_quality"],
            "relevant_chunks_count": len(validation_result["relevant_chunks"]),
            "processing_time_ms": validation_result["processing_time_ms"],
            "error": None
        }

        # Include evidence if requested
        if include_evidence and validation_result.get("evidence"):
            response["evidence"] = [
                evidence["quote"]
                for evidence in validation_result["evidence"][:3]  # Top 3 quotes
            ]
            print(f"  [OK] Including {len(response['evidence'])} evidence quotes")
        else:
            response["evidence"] = []

        return response

    except Exception as e:
        print(f"\n[ERROR]: {e}")
        return {
            "answer": f"An error occurred while processing your query: {str(e)}",
            "retrieval_quality": "Unknown",
            "relevant_chunks_count": 0,
            "evidence": [],
            "processing_time_ms": 0.0,
            "error": str(e)
        }


def interactive_chatbot():
    """
    Interactive chatbot session for testing.

    Allows users to ask questions in a loop until they type 'quit' or 'exit'.
    """
    print("\n" + "="*60)
    print("Book RAG Chatbot - Interactive Mode")
    print("="*60)
    print("\nAsk questions about the book content.")
    print("Type 'quit' or 'exit' to end the session.\n")

    while True:
        try:
            # Get user input
            user_query = input("\n> Your question: ").strip()

            if not user_query:
                continue

            # Check for exit commands
            if user_query.lower() in ['quit', 'exit', 'q']:
                print("\nGoodbye!")
                break

            # Query chatbot
            response = query_chatbot(user_query, include_evidence=True)

            # Display response
            print(f"\n{'='*60}")
            print("Answer:")
            print(f"{'='*60}")
            print(response["answer"])

            # Display metadata
            print(f"\n{'='*60}")
            print("Metadata:")
            print(f"{'='*60}")
            print(f"  Quality: {response['retrieval_quality']}")
            print(f"  Relevant chunks: {response['relevant_chunks_count']}")
            print(f"  Processing time: {response['processing_time_ms']:.2f}ms")

            # Display evidence if available
            if response.get("evidence"):
                print(f"\nKey Evidence:")
                for i, evidence in enumerate(response["evidence"], 1):
                    print(f"  {i}. \"{evidence}\"")

        except KeyboardInterrupt:
            print("\n\nGoodbye!")
            break
        except Exception as e:
            print(f"\n[ERROR] Unexpected error: {e}")


def main():
    """
    Main function to demonstrate the chatbot with validation.
    """
    # Test queries
    test_queries = [
        "What is Physical AI?",
        "What are the key features of ROS 2?",
        "How does simulation help in robotics?",
        "What is quantum computing?"  # Should fail - not in knowledge base
    ]

    print("\n" + "="*60)
    print("Book RAG Chatbot - Validation Demo")
    print("="*60)

    for query in test_queries:
        response = query_chatbot(query, include_evidence=True)

        print(f"\n{'='*60}")
        print(f"Query: {query}")
        print(f"Quality: {response['retrieval_quality']}")
        print(f"Answer: {response['answer'][:200]}...")
        print(f"{'='*60}")

    # Start interactive mode
    print("\n\nStarting interactive mode...\n")
    interactive_chatbot()


if __name__ == "__main__":
    main()
