"""
Fix Qdrant collection by recreating it with proper indexes
"""
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PayloadSchemaType

load_dotenv()

COLLECTION_NAME = "rag_embedding"

# Initialize client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

print("Deleting old collection...")
try:
    qdrant_client.delete_collection(COLLECTION_NAME)
    print(f"Deleted collection '{COLLECTION_NAME}'")
except Exception as e:
    print(f"Collection might not exist: {e}")

print("\nCreating new collection with proper configuration...")
try:
    # Create collection
    qdrant_client.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
            size=1024,
            distance=Distance.COSINE,
        ),
    )
    print(f"Created collection '{COLLECTION_NAME}'")

    # Create index for chapter field
    qdrant_client.create_payload_index(
        collection_name=COLLECTION_NAME,
        field_name="chapter",
        field_schema=PayloadSchemaType.KEYWORD,
    )
    print("Created chapter index")

    print("\nCollection ready! Run 'uv run main.py' to populate it.")

except Exception as e:
    print(f"Error: {e}")
