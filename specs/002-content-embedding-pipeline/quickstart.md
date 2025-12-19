# Quickstart: Content Embedding Pipeline

**Feature**: 002-content-embedding-pipeline
**Date**: 2025-12-17

## Prerequisites

1. **Python 3.11+** installed
2. **uv** package manager installed
3. **Cohere API Key** - Get from https://dashboard.cohere.com/api-keys
4. **Qdrant Cloud Account** - Get from https://cloud.qdrant.io/

## Setup

### 1. Navigate to Backend Directory

```bash
cd Backend
```

### 2. Install Dependencies

```bash
uv sync
```

This installs:
- cohere
- qdrant-client
- httpx
- beautifulsoup4
- lxml
- python-dotenv

### 3. Configure Environment Variables

Create `.env` file from template:

```bash
cp .env.example .env
```

Edit `.env` with your credentials:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=https://your-cluster-id.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key_here
```

### 4. Run the Pipeline

```bash
uv run python main.py
```

## Expected Output

```
============================================================
Book RAG Chatbot - Content Embedding Pipeline
============================================================

[1/4] Creating Qdrant collection...
Collection 'rag_embedding' already exists

[2/4] Fetching book page URLs...
Found 7 book pages to process

[3/4] Processing page 1/7: Start Reading
Extracted 1234 characters from https://speckitplus-hackathon1.vercel.app/docs/intro
Created 3 chunks from text
Generated 3 embeddings (dimension: 1024)
Saved 3 chunks to Qdrant

[3/4] Processing page 2/7: Introduction to Physical AI
...

============================================================
[4/4] Pipeline complete!
  Total pages processed: 7
  Total chunks indexed: 25
  Collection: rag_embedding
============================================================
```

## Verification

### Check Qdrant Collection

Using Qdrant Cloud Dashboard:
1. Go to https://cloud.qdrant.io/
2. Select your cluster
3. Navigate to Collections → `rag_embedding`
4. Verify point count matches indexed chunks

### Test Search Query

```python
from qdrant_client import QdrantClient
import cohere
import os

# Initialize clients
cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

# Embed query
query = "What is Physical AI?"
query_embedding = cohere_client.embed(
    texts=[query],
    model="embed-english-v3.0",
    input_type="search_query",
).embeddings[0]

# Search
results = qdrant_client.search(
    collection_name="rag_embedding",
    query_vector=query_embedding,
    limit=3,
)

# Print results
for r in results:
    print(f"Score: {r.score:.4f}")
    print(f"Chapter: {r.payload['chapter']}")
    print(f"Text: {r.payload['text'][:200]}...")
    print("---")
```

## Troubleshooting

### Error: "COHERE_API_KEY not set"
- Ensure `.env` file exists in `Backend/` directory
- Verify key is correctly formatted (no quotes around value)

### Error: "Failed to create collection"
- Check QDRANT_URL is correct (include port 6333)
- Verify QDRANT_API_KEY has write permissions

### Error: "Error fetching URL"
- Check internet connectivity
- Verify https://speckitplus-hackathon1.vercel.app/ is accessible

### Empty chunks extracted
- The target page may have JavaScript-rendered content
- Check the HTML structure matches expected selectors

## Re-indexing Content

To re-index (updates existing embeddings):

```bash
uv run python main.py
```

The pipeline uses content-based IDs, so duplicate content will be overwritten rather than duplicated.

## Next Steps

After successful indexing:
1. Build the chatbot query interface (Feature 003)
2. Implement FastAPI endpoints for search
3. Add admin UI for content management
