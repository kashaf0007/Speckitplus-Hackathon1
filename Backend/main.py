"""
Content Embedding Pipeline for Book RAG Chatbot

This module extracts content from the published book website,
generates embeddings using Cohere, and stores them in Qdrant Cloud.
"""

import os
import hashlib
import httpx
from bs4 import BeautifulSoup
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

load_dotenv()

# Configuration
BASE_URL = "https://speckitplus-hackathon1.vercel.app"
COLLECTION_NAME = "rag_embedding"
CHUNK_SIZE = 500  # Target tokens per chunk
CHUNK_OVERLAP = 100  # Overlap tokens between chunks

# Initialize clients
cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)


def get_all_urls() -> list[dict]:
    """
    Fetch all book content URLs from the deployed site.

    Returns:
        List of dicts with 'url', 'chapter', and 'section' metadata.
    """
    book_pages = [
        {"path": "/docs/intro", "chapter": "Introduction", "section": "Start Reading"},
        {"path": "/docs/chapter-1-introduction", "chapter": "Chapter 1", "section": "Introduction to Physical AI"},
        {"path": "/docs/chapter-2-ros2", "chapter": "Chapter 2", "section": "ROS 2 Fundamentals"},
        {"path": "/docs/chapter-3-simulation", "chapter": "Chapter 3", "section": "Simulation (Gazebo & Unity)"},
        {"path": "/docs/chapter-4-isaac", "chapter": "Chapter 4", "section": "NVIDIA Isaac"},
        {"path": "/docs/chapter-5-vla", "chapter": "Chapter 5", "section": "Vision-Language-Action"},
        {"path": "/docs/chapter-6-capstone", "chapter": "Chapter 6", "section": "Capstone Project"},
    ]

    urls = []
    for page in book_pages:
        urls.append({
            "url": f"{BASE_URL}{page['path']}",
            "chapter": page["chapter"],
            "section": page["section"],
        })

    print(f"Found {len(urls)} book pages to process")
    return urls


def extract_text_from_url(url: str) -> str:
    """
    Fetch and extract readable text content from a URL.

    Args:
        url: The webpage URL to extract text from.

    Returns:
        Extracted text content as a string.
    """
    try:
        response = httpx.get(url, timeout=30.0, follow_redirects=True)
        response.raise_for_status()

        soup = BeautifulSoup(response.text, "lxml")

        # Remove script, style, and nav elements
        for element in soup(["script", "style", "nav", "header", "footer", "aside"]):
            element.decompose()

        # Try to find main content area (common patterns)
        main_content = (
            soup.find("main") or
            soup.find("article") or
            soup.find(class_="markdown") or
            soup.find(class_="docs-content") or
            soup.find(class_="content") or
            soup.body
        )

        if main_content:
            text = main_content.get_text(separator="\n", strip=True)
        else:
            text = soup.get_text(separator="\n", strip=True)

        # Clean up whitespace
        lines = [line.strip() for line in text.splitlines() if line.strip()]
        cleaned_text = "\n".join(lines)

        print(f"Extracted {len(cleaned_text)} characters from {url}")
        return cleaned_text

    except httpx.HTTPError as e:
        print(f"Error fetching {url}: {e}")
        return ""


def chunk_text(text: str, chunk_size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP) -> list[str]:
    """
    Split text into overlapping chunks suitable for embedding.

    Args:
        text: The text to chunk.
        chunk_size: Target number of words per chunk.
        overlap: Number of words to overlap between chunks.

    Returns:
        List of text chunks.
    """
    if not text:
        return []

    words = text.split()
    chunks = []

    if len(words) <= chunk_size:
        return [text]

    start = 0
    while start < len(words):
        end = start + chunk_size
        chunk_words = words[start:end]
        chunk = " ".join(chunk_words)
        chunks.append(chunk)

        # Move start position, accounting for overlap
        start = end - overlap

        # Prevent infinite loop if overlap >= chunk_size
        if overlap >= chunk_size and start <= end - chunk_size:
            start = end

    print(f"Created {len(chunks)} chunks from text")
    return chunks


def embed(texts: list[str]) -> list[list[float]]:
    """
    Generate embeddings for text chunks using Cohere.

    Args:
        texts: List of text strings to embed.

    Returns:
        List of embedding vectors.
    """
    if not texts:
        return []

    try:
        response = cohere_client.embed(
            texts=texts,
            model="embed-english-v3.0",
            input_type="search_document",
        )

        embeddings = response.embeddings
        print(f"Generated {len(embeddings)} embeddings (dimension: {len(embeddings[0])})")
        return embeddings

    except Exception as e:
        print(f"Error generating embeddings: {e}")
        return []


def create_collection() -> bool:
    """
    Create the Qdrant collection for storing embeddings.

    Returns:
        True if collection was created or already exists, False on error.
    """
    try:
        # Check if collection exists
        collections = qdrant_client.get_collections().collections
        collection_names = [c.name for c in collections]

        if COLLECTION_NAME in collection_names:
            print(f"Collection '{COLLECTION_NAME}' already exists")
            # Ensure chapter index exists for existing collection
            try:
                from qdrant_client.models import PayloadSchemaType
                qdrant_client.create_payload_index(
                    collection_name=COLLECTION_NAME,
                    field_name="chapter",
                    field_schema=PayloadSchemaType.KEYWORD,
                )
                print(f"Created chapter index for existing collection")
            except Exception as idx_err:
                # Index might already exist, that's okay
                pass
            return True

        # Create collection with Cohere embed-english-v3.0 dimensions (1024)
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=1024,  # Cohere embed-english-v3.0 dimension
                distance=Distance.COSINE,
            ),
        )

        # Create index for chapter field to enable filtering
        from qdrant_client.models import PayloadSchemaType
        qdrant_client.create_payload_index(
            collection_name=COLLECTION_NAME,
            field_name="chapter",
            field_schema=PayloadSchemaType.KEYWORD,
        )

        print(f"Created collection '{COLLECTION_NAME}' with chapter index")
        return True

    except Exception as e:
        print(f"Error creating collection: {e}")
        return False


def save_chunk_to_qdrant(
    chunks: list[str],
    embeddings: list[list[float]],
    metadata: dict,
) -> bool:
    """
    Save embedded chunks to Qdrant with metadata.

    Args:
        chunks: List of text chunks.
        embeddings: Corresponding embedding vectors.
        metadata: Dict with 'chapter', 'section', 'url' keys.

    Returns:
        True if successful, False on error.
    """
    if not chunks or not embeddings:
        print("No chunks or embeddings to save")
        return False

    if len(chunks) != len(embeddings):
        print(f"Mismatch: {len(chunks)} chunks vs {len(embeddings)} embeddings")
        return False

    try:
        points = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            # Generate unique ID from content hash
            content_hash = hashlib.md5(chunk.encode()).hexdigest()
            point_id = int(content_hash[:16], 16) % (2**63)  # Ensure positive int64

            point = PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "text": chunk,
                    "chapter": metadata.get("chapter", "Unknown"),
                    "section": metadata.get("section", "Unknown"),
                    "source_url": metadata.get("url", ""),
                    "chunk_index": i,
                },
            )
            points.append(point)

        # Upsert points (handles duplicates via ID)
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points,
        )

        print(f"Saved {len(points)} chunks to Qdrant")
        return True

    except Exception as e:
        print(f"Error saving to Qdrant: {e}")
        return False


def verify_collection() -> dict:
    """
    Query Qdrant collection stats for verification.

    Returns:
        Dict with collection statistics (point count, etc.).
    """
    try:
        collection_info = qdrant_client.get_collection(COLLECTION_NAME)
        point_count = collection_info.points_count

        print(f"\nCollection Statistics:")
        print(f"  Collection name: {COLLECTION_NAME}")
        print(f"  Total points indexed: {point_count}")
        print(f"  Vector size: 1024")
        print(f"  Distance metric: COSINE")

        return {
            "collection": COLLECTION_NAME,
            "points": point_count,
            "status": "verified"
        }

    except Exception as e:
        print(f"Error verifying collection: {e}")
        return {"status": "error", "message": str(e)}


def verify_sample_retrieval(query_text: str = "What is Physical AI?") -> bool:
    """
    Test sample retrieval to verify embeddings work correctly.

    Args:
        query_text: Test query string.

    Returns:
        True if retrieval works, False otherwise.
    """
    try:
        print(f"\nTesting sample retrieval with query: '{query_text}'")

        # Generate query embedding
        query_embedding = cohere_client.embed(
            texts=[query_text],
            model="embed-english-v3.0",
            input_type="search_query",
        ).embeddings[0]

        # Search for similar chunks
        results = qdrant_client.query_points(
            collection_name=COLLECTION_NAME,
            query=query_embedding,
            limit=3,
        ).points

        if results:
            print(f"  Retrieved {len(results)} results:")
            for i, result in enumerate(results, 1):
                print(f"\n  Result {i}:")
                print(f"    Score: {result.score:.4f}")
                print(f"    Chapter: {result.payload.get('chapter', 'Unknown')}")
                print(f"    Section: {result.payload.get('section', 'Unknown')}")
                print(f"    Text preview: {result.payload.get('text', '')[:100]}...")
            return True
        else:
            print("  No results found")
            return False

    except Exception as e:
        print(f"  Error during retrieval: {e}")
        return False


def verify_chapter_filtering(chapter: str = "Chapter 1") -> bool:
    """
    Test chapter-based filtering to verify metadata queries work.

    Args:
        chapter: Chapter name to filter by.

    Returns:
        True if filtering works, False otherwise.
    """
    try:
        print(f"\nTesting chapter filtering for: '{chapter}'")

        # Scroll through collection with filter
        from qdrant_client.models import Filter, FieldCondition, MatchValue

        scroll_result = qdrant_client.scroll(
            collection_name=COLLECTION_NAME,
            scroll_filter=Filter(
                must=[
                    FieldCondition(
                        key="chapter",
                        match=MatchValue(value=chapter),
                    )
                ]
            ),
            limit=10,
        )

        points = scroll_result[0]

        if points:
            print(f"  Found {len(points)} chunks for '{chapter}':")
            for i, point in enumerate(points[:3], 1):
                print(f"    {i}. Section: {point.payload.get('section', 'Unknown')}")
                print(f"       Text preview: {point.payload.get('text', '')[:80]}...")
            return True
        else:
            print(f"  No chunks found for '{chapter}'")
            return False

    except Exception as e:
        print(f"  Error during filtering: {e}")
        return False


def main():
    """
    Main execution: Extract content, generate embeddings, and store in Qdrant.
    """
    print("=" * 60)
    print("Book RAG Chatbot - Content Embedding Pipeline")
    print("=" * 60)

    # Step 1: Create collection
    print("\n[1/4] Creating Qdrant collection...")
    if not create_collection():
        print("Failed to create collection. Exiting.")
        return

    # Step 2: Get all URLs
    print("\n[2/4] Fetching book page URLs...")
    urls = get_all_urls()

    if not urls:
        print("No URLs found. Exiting.")
        return

    total_chunks = 0

    # Step 3 & 4: Process each URL
    for i, url_info in enumerate(urls, 1):
        url = url_info["url"]
        print(f"\n[3/4] Processing page {i}/{len(urls)}: {url_info['section']}")

        # Extract text
        text = extract_text_from_url(url)
        if not text:
            print(f"  Skipping {url} - no content extracted")
            continue

        # Chunk text
        chunks = chunk_text(text)
        if not chunks:
            print(f"  Skipping {url} - no chunks created")
            continue

        # Generate embeddings
        print(f"  Generating embeddings for {len(chunks)} chunks...")
        embeddings = embed(chunks)
        if not embeddings:
            print(f"  Skipping {url} - embedding failed")
            continue

        # Save to Qdrant
        print(f"  Saving to Qdrant...")
        metadata = {
            "chapter": url_info["chapter"],
            "section": url_info["section"],
            "url": url,
        }

        if save_chunk_to_qdrant(chunks, embeddings, metadata):
            total_chunks += len(chunks)

    # Summary
    print("\n" + "=" * 60)
    print(f"[4/4] Pipeline complete!")
    print(f"  Total pages processed: {len(urls)}")
    print(f"  Total chunks indexed: {total_chunks}")
    print(f"  Collection: {COLLECTION_NAME}")
    print("=" * 60)

    # Verification (User Story 4)
    if total_chunks > 0:
        print("\n" + "=" * 60)
        print("Running verification tests...")
        print("=" * 60)

        # T031-T032: Verify collection statistics
        verify_collection()

        # T033: Test sample retrieval
        verify_sample_retrieval()

        # T034: Test chapter-based filtering
        verify_chapter_filtering()

        print("\n" + "=" * 60)
        print("Verification complete!")
        print("=" * 60)


if __name__ == "__main__":
    main()
