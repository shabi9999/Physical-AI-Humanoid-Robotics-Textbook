"""
OpenAI Embeddings Service

Generates embeddings for text using OpenAI's text-embedding-3-small model.
"""

import logging
import os
from typing import List
from openai import AsyncOpenAI

logger = logging.getLogger(__name__)


class EmbeddingsService:
    """Service for generating embeddings using OpenAI API."""

    def __init__(self):
        """Initialize OpenAI client."""
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            logger.error("OPENAI_API_KEY not set in environment")
            self.client = None
        else:
            self.client = AsyncOpenAI(api_key=api_key)

    async def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to embed

        Returns:
            Embedding vector (1536-dimensional for text-embedding-3-small)
        """
        if not self.client:
            logger.error("OpenAI client not initialized")
            return []

        try:
            response = await self.client.embeddings.create(
                model="text-embedding-3-small",
                input=text,
            )

            embedding = response.data[0].embedding
            logger.debug(f"Generated embedding for text ({len(text)} chars)")
            return embedding

        except Exception as e:
            logger.error(f"Embedding generation failed: {e}")
            return []

    async def embed_texts(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts.

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors
        """
        if not self.client or not texts:
            logger.error("OpenAI client not initialized or empty texts")
            return []

        try:
            response = await self.client.embeddings.create(
                model="text-embedding-3-small",
                input=texts,
            )

            embeddings = [item.embedding for item in response.data]
            logger.info(f"Generated {len(embeddings)} embeddings")
            return embeddings

        except Exception as e:
            logger.error(f"Batch embedding generation failed: {e}")
            return []

    async def health_check(self) -> bool:
        """Test OpenAI API connectivity."""
        if not self.client:
            return False

        try:
            # Try to embed a simple test string
            await self.embed_text("test")
            return True
        except Exception as e:
            logger.error(f"OpenAI health check failed: {e}")
            return False


# Global instance
_embeddings_service: EmbeddingsService = None


def get_embeddings_service() -> EmbeddingsService:
    """Get or create global embeddings service instance."""
    global _embeddings_service
    if _embeddings_service is None:
        _embeddings_service = EmbeddingsService()
    return _embeddings_service
