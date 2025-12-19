"""
Test fixtures for sample chunks

Provides reusable test data for retrieval validation tests.
"""

# Sample chunks for relevance testing
RELEVANT_CHUNKS_ROS2 = [
    {
        "chunk_id": "ros2_ch001",
        "text": "ROS 2 is a set of software libraries and tools for building robot applications. "
                "It provides real-time capabilities, improved security, and cross-platform support.",
        "metadata": {"chapter": "Chapter 2", "section": "ROS 2 Fundamentals"},
        "score": 0.92
    },
    {
        "chunk_id": "ros2_ch002",
        "text": "Key features of ROS 2 include: distributed architecture, real-time performance, "
                "multiple middleware support (DDS), and enhanced security features.",
        "metadata": {"chapter": "Chapter 2", "section": "Key Features"},
        "score": 0.88
    },
    {
        "chunk_id": "ros2_ch003",
        "text": "ROS 2 differs from ROS 1 in several ways: it uses DDS for communication, "
                "supports real-time systems, and has better security and cross-platform capabilities.",
        "metadata": {"chapter": "Chapter 2", "section": "ROS 1 vs ROS 2"},
        "score": 0.85
    }
]

# Irrelevant chunks for negative testing
IRRELEVANT_CHUNKS = [
    {
        "chunk_id": "unrelated_001",
        "text": "Cooking recipes for pasta dishes. Italian cuisine is known for its delicious pasta "
                "varieties including carbonara, bolognese, and pesto.",
        "metadata": {"chapter": "Chapter 10", "section": "Cooking"},
        "score": 0.12
    },
    {
        "chunk_id": "unrelated_002",
        "text": "The history of ancient Rome spans from 753 BC to 476 AD. The Roman Empire "
                "was one of the largest empires in world history.",
        "metadata": {"chapter": "Chapter 15", "section": "History"},
        "score": 0.08
    }
]

# Mixed relevant and irrelevant chunks
MIXED_CHUNKS = RELEVANT_CHUNKS_ROS2[:2] + IRRELEVANT_CHUNKS

# Chunks with answer present
CHUNKS_WITH_ANSWER = [
    {
        "chunk_id": "answer_001",
        "text": "The company was founded in 1995 by John Smith and Jane Doe. "
                "Initially operating out of a small garage, it has grown to become a global leader.",
        "metadata": {"chapter": "Chapter 1", "section": "Company History"},
        "score": 0.95
    },
    {
        "chunk_id": "answer_002",
        "text": "Our pricing tiers include: Basic ($10/month), Pro ($25/month), and Enterprise (custom pricing). "
                "Each tier offers different feature sets and support levels.",
        "metadata": {"chapter": "Chapter 5", "section": "Pricing"},
        "score": 0.91
    }
]

# Chunks without answer
CHUNKS_WITHOUT_ANSWER = [
    {
        "chunk_id": "noanswer_001",
        "text": "The product features include a modern UI, real-time collaboration, and cloud storage. "
                "Users can access the platform from any device with an internet connection.",
        "metadata": {"chapter": "Chapter 3", "section": "Features"},
        "score": 0.75
    },
    {
        "chunk_id": "noanswer_002",
        "text": "Our customer support team is available 24/7 via email and chat. "
                "We typically respond to inquiries within 2 hours during business hours.",
        "metadata": {"chapter": "Chapter 4", "section": "Support"},
        "score": 0.70
    }
]

# Empty chunks for edge case testing
EMPTY_CHUNKS = []

# Chunk with very high relevance
HIGH_RELEVANCE_CHUNK = [
    {
        "chunk_id": "high_rel_001",
        "text": "ROS 2 (Robot Operating System 2) is the next-generation robotic middleware. "
                "It addresses many of ROS 1's shortcomings and provides enhanced real-time capabilities, "
                "improved security through DDS, and better cross-platform support for Windows, Linux, and macOS.",
        "metadata": {"chapter": "Chapter 2", "section": "Introduction to ROS 2"},
        "score": 0.98
    }
]

# Chunk with low relevance
LOW_RELEVANCE_CHUNK = [
    {
        "chunk_id": "low_rel_001",
        "text": "Software development requires understanding of programming languages, algorithms, and data structures. "
                "Common languages include Python, Java, C++, and JavaScript.",
        "metadata": {"chapter": "Chapter 20", "section": "Software Development"},
        "score": 0.25
    }
]

# Malformed chunk (for error testing)
MALFORMED_CHUNKS = [
    {
        "chunk_id": "",  # Empty chunk_id
        "text": "Some text here",
        "metadata": {},
        "score": 0.5
    },
    {
        "chunk_id": "valid_001",
        "text": "",  # Empty text
        "metadata": {},
        "score": 0.5
    }
]

# Duplicate chunk IDs (for error testing)
DUPLICATE_CHUNK_IDS = [
    {
        "chunk_id": "duplicate_001",
        "text": "First instance of duplicate",
        "metadata": {},
        "score": 0.8
    },
    {
        "chunk_id": "duplicate_001",  # Duplicate ID
        "text": "Second instance of duplicate",
        "metadata": {},
        "score": 0.7
    }
]

# Test queries
TEST_QUERIES = {
    "ros2_features": "What are the key features of ROS 2?",
    "company_founding": "What year was the company founded?",
    "pricing": "What are the pricing tiers?",
    "password_reset": "How do I reset my password?",
    "quantum_computing": "What is quantum computing?",
    "photosynthesis": "What is photosynthesis?",
    "empty": "",
    "short": "Hi",
    "very_long": "What are " + ("all the features and capabilities and benefits " * 50)  # Exceeds 500 chars
}
