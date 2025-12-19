# Testing and Validation Guide

This document outlines the remaining validation tasks that require manual execution with live API credentials.

## Prerequisites

Before running tests, ensure:
1. `.env` file is created (copy from `.env.example`)
2. All API keys are configured:
   - `COHERE_API_KEY` - from https://dashboard.cohere.com/api-keys
   - `QDRANT_URL` - from https://cloud.qdrant.io/
   - `QDRANT_API_KEY` - from https://cloud.qdrant.io/
3. Dependencies are installed: `cd Backend && uv sync`

## Test Execution

### T043: Validate Performance Goals

**Success Criteria** (from spec.md):
- **SC-001**: Index 300-page book in <5 minutes
- **SC-002**: Process 100 chunks in <30 seconds

**Test Steps**:
```bash
cd Backend
time uv run python main.py
```

**Expected Results**:
- Pipeline completes for all 7 chapters
- Total time < 5 minutes (for full book)
- Console output shows chunk processing rate
- Verify 100 chunks process in <30 seconds

**Validation Checklist**:
- [ ] Total execution time < 5 minutes
- [ ] Chunks process at rate > 200 per minute (100 chunks in <30s)
- [ ] No timeout errors
- [ ] All 7 pages processed successfully

---

### T044: Test with All 7 Book Chapters

**Test Steps**:
```bash
cd Backend
uv run python main.py
```

**Expected Output**:
```
============================================================
Book RAG Chatbot - Content Embedding Pipeline
============================================================

[1/4] Creating Qdrant collection...
Collection 'rag_embedding' already exists (or created successfully)

[2/4] Fetching book page URLs...
Found 7 book pages to process

[3/4] Processing page 1/7: Start Reading
Extracted XXXX characters from https://speckitplus-hackathon1.vercel.app/docs/intro
Created X chunks from text
Generated X embeddings (dimension: 1024)
Saved X chunks to Qdrant

[3/4] Processing page 2/7: Introduction to Physical AI
...
[continues for all 7 pages]

============================================================
[4/4] Pipeline complete!
  Total pages processed: 7
  Total chunks indexed: XXX
  Collection: rag_embedding
============================================================

============================================================
Running verification tests...
============================================================

📊 Collection Statistics:
  Collection name: rag_embedding
  Total points indexed: XXX
  Vector size: 1024
  Distance metric: COSINE

🔍 Testing sample retrieval with query: 'What is Physical AI?'
  ✅ Retrieved 3 results:
  ...

🔍 Testing chapter filtering for: 'Chapter 1'
  ✅ Found X chunks for 'Chapter 1':
  ...

============================================================
Verification complete!
============================================================
```

**Validation Checklist**:
- [ ] All 7 pages processed without errors
- [ ] Each page shows: characters extracted, chunks created, embeddings generated, chunks saved
- [ ] Final summary shows total pages = 7
- [ ] Verification tests all pass
- [ ] No HTTP errors or API failures

---

### T045: Verify Metadata Preservation

**Success Criteria** (from spec.md SC-004):
- Metadata correctly preserved for 100% of chunks
- Spot-check 10% of chunks for accuracy

**Test Steps**:
After running the pipeline, use Qdrant Cloud Dashboard:

1. Go to https://cloud.qdrant.io/
2. Select your cluster
3. Navigate to Collections → `rag_embedding`
4. Click "Explore" to view random points
5. Inspect payload for multiple points

**Expected Payload Structure**:
```json
{
  "text": "...",
  "chapter": "Chapter 1",
  "section": "Introduction to Physical AI",
  "source_url": "https://speckitplus-hackathon1.vercel.app/docs/chapter-1-introduction",
  "chunk_index": 0
}
```

**Validation Checklist**:
- [ ] `chapter` field present and correct for all sampled points
- [ ] `section` field present and matches chapter content
- [ ] `source_url` field present and valid
- [ ] `chunk_index` field present and sequential
- [ ] `text` field contains actual chunk content
- [ ] Spot-check 10% of points (at least 10-20 random points)

---

### T046: Test Duplicate Detection

**Test Steps**:
```bash
# Run pipeline first time
cd Backend
uv run python main.py

# Note the total chunks count
# Expected: ~XXX chunks

# Run pipeline second time (immediately after)
uv run python main.py
```

**Expected Results**:
- Second run completes successfully
- Total chunks count remains the same (no duplicates added)
- Points are upserted (overwritten with same content hash IDs)
- No errors about duplicate IDs

**Validation in Qdrant Dashboard**:
1. Check collection point count before first run
2. Run pipeline first time, note point count
3. Run pipeline second time
4. Verify point count is identical to step 2

**Validation Checklist**:
- [ ] First run indexes all chunks
- [ ] Second run completes without errors
- [ ] Point count unchanged after second run
- [ ] MD5-based IDs prevent duplicates

---

### T049: Run Quickstart.md Validation End-to-End

**Test Steps**:
Follow every step in `quickstart.md` from scratch:

1. **Setup**:
   ```bash
   cd Backend
   uv sync
   cp .env.example .env
   # Edit .env with real credentials
   ```

2. **Run Pipeline**:
   ```bash
   uv run python main.py
   ```

3. **Verify in Qdrant Dashboard** (per quickstart.md)

4. **Test Search Query** (run the Python snippet from quickstart.md):
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

**Validation Checklist**:
- [ ] `uv sync` installs all dependencies successfully
- [ ] `.env` file created and populated correctly
- [ ] Pipeline runs successfully following quickstart steps
- [ ] Qdrant Dashboard shows collection with indexed points
- [ ] Test query script runs and returns relevant results
- [ ] Results have score, chapter, and text fields
- [ ] All troubleshooting scenarios in quickstart.md tested (if issues arise)

---

## Summary Report Template

After completing all tests, fill out:

```
# Pipeline Validation Report

**Date**: [DATE]
**Tester**: [NAME]

## T043: Performance Validation
- Total execution time: [X] minutes
- Chunks per minute: [X]
- Result: [PASS/FAIL]

## T044: All Chapters Test
- Pages processed: [X/7]
- Total chunks indexed: [X]
- Result: [PASS/FAIL]

## T045: Metadata Verification
- Points inspected: [X]
- Metadata correct: [X/X]
- Result: [PASS/FAIL]

## T046: Duplicate Detection
- First run chunks: [X]
- Second run chunks: [X]
- Duplicates prevented: [YES/NO]
- Result: [PASS/FAIL]

## T049: Quickstart Validation
- Setup successful: [YES/NO]
- Pipeline runs: [YES/NO]
- Query test works: [YES/NO]
- Result: [PASS/FAIL]

## Overall Status
- [ ] All tests passed
- [ ] Pipeline ready for production
- [ ] Issues found: [DESCRIBE]
```

---

## Notes

- These tests require live API access and cannot be automated without credentials
- Estimated total test time: 15-20 minutes
- Cohere free tier: sufficient for testing
- Qdrant Cloud free tier: sufficient for book-sized content
- If any test fails, check:
  1. API credentials are valid
  2. Network connectivity
  3. API rate limits not exceeded
  4. Console output for specific errors
