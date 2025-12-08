"""
Qdrant Vector Database Client

Manages connections and operations with Qdrant vector database for embedding storage and retrieval.
"""

import logging
from typing import Optional, List, Dict, Any
import os
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from qdrant_client.http import models

logger = logging.getLogger(__name__)


class QdrantDatabase:
    """Qdrant vector database client wrapper."""

    def __init__(self):
        """Initialize Qdrant client."""
        self.url = os.getenv("QDRANT_URL", "http://localhost:6333")
        self.api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = "ros2_book_content"

        try:
            self.client = QdrantClient(
                url=self.url,
                api_key=self.api_key if self.api_key else None,
            )
            logger.info(f"Connected to Qdrant at {self.url}")
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {e}")
            self.client = None

    def ensure_collection(self) -> bool:
        """
        Ensure the collection exists. Create if missing.

        Returns:
            True if collection exists or was created, False otherwise
        """
        if not self.client:
            return False

        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.collection_name in collection_names:
                logger.info(f"Collection '{self.collection_name}' already exists")
                return True

            # Create collection if missing
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=1536,  # OpenAI text-embedding-3-small vector size
                    distance=Distance.COSINE
                ),
            )
            logger.info(f"Created collection '{self.collection_name}'")
            return True

        except Exception as e:
            logger.error(f"Error managing collection: {e}")
            return False

    async def search(
        self,
        query_vector: List[float],
        limit: int = 20,
        score_threshold: float = 0.5
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in the collection.

        Args:
            query_vector: Query embedding vector
            limit: Maximum number of results
            score_threshold: Minimum similarity score

        Returns:
            List of matching documents with metadata
        """
        if not self.client:
            logger.error("Qdrant client not initialized")
            return []

        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit,
                score_threshold=score_threshold,
            )

            documents = []
            for point in results:
                doc = {
                    "id": point.id,
                    "score": point.score,
                    **point.payload
                }
                documents.append(doc)

            logger.debug(f"Found {len(documents)} matching documents")
            return documents

        except Exception as e:
            logger.error(f"Search failed: {e}")
            return []

    async def upsert(self, points: List[Dict[str, Any]]) -> bool:
        """
        Upsert points into the collection.

        Args:
            points: List of point dictionaries with id, vector, and payload

        Returns:
            True if successful, False otherwise
        """
        if not self.client:
            logger.error("Qdrant client not initialized")
            return False

        try:
            qdrant_points = []
            for point in points:
                qdrant_points.append(
                    PointStruct(
                        id=point["id"],
                        vector=point["vector"],
                        payload=point.get("payload", {})
                    )
                )

            self.client.upsert(
                collection_name=self.collection_name,
                points=qdrant_points,
            )

            logger.info(f"Upserted {len(points)} points")
            return True

        except Exception as e:
            logger.error(f"Upsert failed: {e}")
            return False

    async def get_collection_info(self) -> Optional[Dict[str, Any]]:
        """Get collection statistics and info."""
        if not self.client:
            return None

        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                "name": collection_info.name,
                "points_count": collection_info.points_count,
                "vectors_config": collection_info.config.params.vectors,
            }
        except Exception as e:
            logger.error(f"Failed to get collection info: {e}")
            return None

    async def health_check(self) -> bool:
        """Check Qdrant server health."""
        if not self.client:
            return False

        try:
            self.client.get_collections()
            return True
        except Exception as e:
            logger.error(f"Qdrant health check failed: {e}")
            return False


# Global instance
_qdrant_db: Optional[QdrantDatabase] = None


def get_qdrant() -> QdrantDatabase:
    """Get or create global Qdrant database instance."""
    global _qdrant_db
    if _qdrant_db is None:
        _qdrant_db = QdrantDatabase()
    return _qdrant_db
