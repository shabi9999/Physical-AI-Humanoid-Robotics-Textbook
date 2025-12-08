"""
Vector Retrieval Service

Performs semantic search using Qdrant to retrieve relevant chunks for RAG.
"""

import logging
from typing import List, Dict, Any
from src.db.qdrant import get_qdrant
from src.services.embeddings import get_embeddings_service

logger = logging.getLogger(__name__)


class RetrievalService:
    """Handles semantic search and chunk retrieval."""

    def __init__(self):
        """Initialize retrieval service."""
        self.qdrant = get_qdrant()
        self.embeddings = get_embeddings_service()

    async def retrieve_chunks(
        self,
        query: str,
        top_k: int = 20,
        score_threshold: float = 0.5,
    ) -> List[Dict[str, Any]]:
        """
        Retrieve relevant chunks for a query.

        Args:
            query: User query text
            top_k: Number of chunks to retrieve
            score_threshold: Minimum similarity score

        Returns:
            List of relevant chunks with metadata and scores
        """
        if not query.strip():
            logger.warning("Empty query provided")
            return []

        try:
            # Generate query embedding
            query_vector = await self.embeddings.embed_text(query)
            if not query_vector:
                logger.error("Failed to generate query embedding")
                return []

            # Search vector database
            chunks = await self.qdrant.search(
                query_vector=query_vector,
                limit=top_k,
                score_threshold=score_threshold,
            )

            logger.info(f"Retrieved {len(chunks)} chunks for query: {query[:50]}...")
            return chunks

        except Exception as e:
            logger.error(f"Retrieval failed: {e}")
            return []

    async def retrieve_by_chapter(
        self,
        chapter: str,
        limit: int = 100,
    ) -> List[Dict[str, Any]]:
        """
        Retrieve all chunks from a specific chapter.

        Note: This is a placeholder. Full implementation would require
        direct Qdrant filtering capabilities.

        Args:
            chapter: Chapter identifier
            limit: Max chunks to retrieve

        Returns:
            List of chunks from the chapter
        """
        logger.info(f"Retrieving chunks from chapter: {chapter}")
        # Implementation would involve direct Qdrant filtering
        # For now, return empty list
        return []

    async def retrieve_similar(
        self,
        text: str,
        top_k: int = 10,
    ) -> List[Dict[str, Any]]:
        """
        Find chunks similar to a given text.

        Args:
            text: Reference text
            top_k: Number of similar chunks

        Returns:
            List of similar chunks
        """
        return await self.retrieve_chunks(text, top_k=top_k)

    async def batch_retrieve(
        self,
        queries: List[str],
        top_k: int = 10,
    ) -> List[List[Dict[str, Any]]]:
        """
        Retrieve chunks for multiple queries.

        Args:
            queries: List of query strings
            top_k: Results per query

        Returns:
            List of result lists
        """
        results = []
        for query in queries:
            chunks = await self.retrieve_chunks(query, top_k=top_k)
            results.append(chunks)

        logger.info(f"Batch retrieved {len(results)} query results")
        return results

    async def health_check(self) -> bool:
        """Check retrieval service health."""
        try:
            # Check Qdrant health
            qdrant_healthy = await self.qdrant.health_check()
            # Check embeddings service health
            embeddings_healthy = await self.embeddings.health_check()

            return qdrant_healthy and embeddings_healthy
        except Exception as e:
            logger.error(f"Health check failed: {e}")
            return False


# Global instance
_retrieval_service: RetrievalService = None


def get_retrieval_service() -> RetrievalService:
    """Get or create global retrieval service instance."""
    global _retrieval_service
    if _retrieval_service is None:
        _retrieval_service = RetrievalService()
    return _retrieval_service
