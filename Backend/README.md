# Book RAG Chatbot Backend

A controlled RAG (Retrieval-Augmented Generation) agent that answers user questions strictly from book content with 0% hallucination rate.

## Features

- **Strict Book-Grounding**: Answers only from retrieved book chunks or user-selected text
- **Hallucination Prevention**: Multi-layered constraints ensure 0% hallucination rate
- **Selected Text Override**: Users can highlight specific text for focused Q&A
- **Source Attribution**: Every answer includes chunk IDs, page numbers, and matched text
- **Retrieval Validation**: Integration with validation assistant to verify answer presence

## API Endpoints

### POST /ask

The main endpoint for asking questions about the book content.

#### Request Body

```json
{
  "question": "What is ROS 2?",
  "selected_text": null
}
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `question` | string | Yes | User's question (1-1000 chars) |
| `selected_text` | string | No | User-highlighted passage (max 10000 chars) |

#### Response

```json
{
  "answer": "ROS 2 is a robotics middleware framework that provides hardware abstraction, device drivers, and communication middleware for building robot applications.",
  "sources": ["chunk_123", "chunk_456"],
  "matched_chunks": [
    {
      "chunk_id": "chunk_123",
      "text": "ROS 2 (Robot Operating System 2) is a set of software libraries...",
      "page": 1,
      "chapter": "Chapter 1",
      "section": "Introduction"
    }
  ],
  "grounded": true,
  "retrieval_quality": "Good"
}
```

| Field | Type | Description |
|-------|------|-------------|
| `answer` | string | Generated answer (1-5 sentences) or refusal message |
| `sources` | array | List of source chunk IDs |
| `matched_chunks` | array | Full chunk details with text and metadata |
| `grounded` | boolean | Whether answer is grounded in book content |
| `retrieval_quality` | string | Quality rating: "Good", "Partial", "Poor", or null |

#### Refusal Response

When the answer is not available in the book:

```json
{
  "answer": "This information is not available in the book.",
  "sources": [],
  "matched_chunks": [],
  "grounded": false,
  "retrieval_quality": "Poor"
}
```

### Usage Examples

#### Standard Query (curl)

```bash
curl -X POST "http://localhost:8000/ask" \
  -H "Content-Type: application/json" \
  -d '{"question": "What are the main features of ROS 2?"}'
```

#### Query with Selected Text (curl)

```bash
curl -X POST "http://localhost:8000/ask" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What does this passage explain?",
    "selected_text": "ROS 2 provides real-time support, cross-platform compatibility, and improved security through SROS2."
  }'
```

#### Python Example

```python
import requests

# Standard query
response = requests.post(
    "http://localhost:8000/ask",
    json={"question": "How does ROS 2 handle communication?"}
)
result = response.json()

print(f"Answer: {result['answer']}")
print(f"Sources: {result['sources']}")
print(f"Grounded: {result['grounded']}")

# Query with selected text
response = requests.post(
    "http://localhost:8000/ask",
    json={
        "question": "Explain this concept",
        "selected_text": "DDS (Data Distribution Service) enables publish-subscribe patterns."
    }
)
```

#### JavaScript/Fetch Example

```javascript
// Standard query
const response = await fetch('http://localhost:8000/ask', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    question: 'What is a ROS 2 node?'
  })
});

const result = await response.json();
console.log('Answer:', result.answer);
console.log('Sources:', result.sources);
```

### Error Responses

| Status | Condition | Response |
|--------|-----------|----------|
| 400 | Empty question | `{"detail": "Please provide a question about the book."}` |
| 500 | Qdrant unavailable | `{"detail": "Unable to search the book. Please try again later."}` |
| 500 | Generation error | `{"detail": "Unable to generate answer. Please try again."}` |

## Other Endpoints

### GET /health

Health check endpoint.

```bash
curl http://localhost:8000/health
```

Response:
```json
{
  "status": "healthy",
  "message": "Book RAG Chatbot API is running"
}
```

### POST /query

Legacy query endpoint with retrieval validation metrics.

```bash
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "top_k": 10}'
```

### GET /stats

Get collection statistics from Qdrant.

```bash
curl http://localhost:8000/stats
```

### GET /docs

Interactive API documentation (Swagger UI).

```
http://localhost:8000/docs
```

## Running the Server

```bash
# From the Backend directory
cd Backend

# Install dependencies
pip install -r requirements.txt

# Set environment variables
export GEMINI_API_KEY=your-gemini-api-key
export COHERE_API_KEY=your-cohere-api-key
export QDRANT_URL=your-qdrant-url
export QDRANT_API_KEY=your-qdrant-api-key

# Run the server
python api.py
# or
uvicorn api:app --host 0.0.0.0 --port 8000 --reload
```

## Environment Variables

| Variable | Description |
|----------|-------------|
| `GEMINI_API_KEY` | Google Gemini API key for answer generation |
| `COHERE_API_KEY` | Cohere API key for query embedding |
| `QDRANT_URL` | Qdrant vector database URL |
| `QDRANT_API_KEY` | Qdrant API key |

## Testing

```bash
# Run all tests
pytest Backend/tests/ -v

# Run with coverage
pytest Backend/tests/ -v --cov=Backend --cov-report=html

# Run specific test suites
pytest Backend/tests/unit/ -v
pytest Backend/tests/integration/ -v
pytest Backend/tests/contract/ -v
```

## Architecture

```
Backend/
├── api.py                 # FastAPI application with endpoints
├── agent_rag.py           # RAG agent orchestration
├── chatbot.py             # Cohere/Qdrant client utilities
├── models/
│   └── agent_models.py    # Pydantic request/response models
├── retrieval_validation/  # Validation assistant integration
└── tests/
    ├── unit/              # Unit tests
    ├── integration/       # Integration tests
    └── contract/          # API contract tests
```
