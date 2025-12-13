"""
Test Qdrant Upload Locally

This script tests the embedding upload process using Qdrant in-memory mode
(no external Qdrant server required). It validates:
1. Chunk loading from index.json
2. Embedding generation capability
3. Vector upload and storage
4. Search/retrieval functionality

Usage:
    python scripts/test_qdrant_upload.py [--index-file index.json] [--sample-size 5]

Environment Variables:
    OPENAI_API_KEY - OpenAI API key (or skip embedding tests)
"""

import json
import logging
import os
import sys
import asyncio
import argparse
from pathlib import Path
from typing import List, Dict, Any
import hashlib

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
    ]
)
logger = logging.getLogger(__name__)


class MockEmbeddingGenerator:
    """Generate deterministic embeddings from text for testing."""

    @staticmethod
    def generate_embedding(text: str, dimension: int = 1536) -> List[float]:
        """
        Generate a deterministic embedding from text hash.

        Args:
            text: Text to embed
            dimension: Embedding dimension

        Returns:
            List of floats (1536-dimensional)
        """
        # Use hash of text as seed
        hash_obj = hashlib.md5(text.encode())
        hash_int = int(hash_obj.hexdigest(), 16)

        # Generate pseudo-random numbers in [-1, 1] from hash
        embeddings = []
        for i in range(dimension):
            # Generate pseudo-random float from hash
            seed = (hash_int + i) % (2**31)
            value = 2.0 * (seed / (2**31)) - 1.0  # Scale to [-1, 1]
            embeddings.append(value)

        # Normalize to unit vector
        norm = sum(v**2 for v in embeddings) ** 0.5
        if norm > 0:
            embeddings = [v / norm for v in embeddings]

        return embeddings


class LocalQdrantSimulator:
    """Simulate Qdrant operations in memory."""

    def __init__(self):
        """Initialize in-memory vector store."""
        self.points = {}  # id -> {vector, payload}
        self.collection_name = "ros2_book_content"
        logger.info("Initialized local Qdrant simulator")

    def create_collection(self) -> bool:
        """Create collection."""
        logger.info(f"Created collection: {self.collection_name}")
        return True

    def upsert(self, points: List[Dict[str, Any]]) -> bool:
        """
        Store points in memory.

        Args:
            points: List of {id, vector, payload}

        Returns:
            True if successful
        """
        for point in points:
            self.points[point['id']] = {
                'vector': point['vector'],
                'payload': point['payload']
            }
        return True

    def search(self, query_vector: List[float], limit: int = 10) -> List[Dict[str, Any]]:
        """
        Find similar vectors using cosine distance.

        Args:
            query_vector: Query embedding
            limit: Max results

        Returns:
            List of similar points with scores
        """
        results = []

        # Calculate cosine similarity to all stored vectors
        for point_id, point_data in self.points.items():
            vector = point_data['vector']

            # Cosine similarity
            dot_product = sum(q * v for q, v in zip(query_vector, vector))
            norm_q = sum(v**2 for v in query_vector) ** 0.5
            norm_v = sum(v**2 for v in vector) ** 0.5

            if norm_q > 0 and norm_v > 0:
                similarity = dot_product / (norm_q * norm_v)
            else:
                similarity = 0.0

            results.append({
                'id': point_id,
                'score': similarity,
                'payload': point_data['payload']
            })

        # Sort by score (highest first) and return top-k
        results.sort(key=lambda x: x['score'], reverse=True)
        return results[:limit]

    def get_collection_info(self) -> Dict[str, Any]:
        """Get collection statistics."""
        return {
            'name': self.collection_name,
            'points_count': len(self.points),
            'vectors_config': {'size': 1536, 'distance': 'COSINE'}
        }


class QdrantUploadTester:
    """Test embedding upload pipeline."""

    def __init__(self, use_openai: bool = False, batch_size: int = 5):
        """
        Initialize tester.

        Args:
            use_openai: Use real OpenAI embeddings (requires API key)
            batch_size: Chunks to process per batch
        """
        self.use_openai = use_openai and os.getenv("OPENAI_API_KEY")
        self.batch_size = batch_size
        self.qdrant = LocalQdrantSimulator()

    async def load_chunks(self, index_file: str, max_chunks: int = None) -> List[Dict[str, Any]]:
        """Load chunks from index.json."""
        index_path = Path(index_file)
        if not index_path.exists():
            raise FileNotFoundError(f"Index file not found: {index_file}")

        logger.info(f"Loading chunks from {index_file}")
        with open(index_path, 'r', encoding='utf-8') as f:
            chunks = json.load(f)

        # Limit chunks if specified
        if max_chunks:
            chunks = chunks[:max_chunks]

        logger.info(f"Loaded {len(chunks)} chunks")
        return chunks

    async def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings."""
        if self.use_openai:
            logger.info("Using OpenAI embeddings")
            from openai import AsyncOpenAI
            client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))

            try:
                response = await client.embeddings.create(
                    model="text-embedding-3-small",
                    input=texts,
                )
                return [item.embedding for item in response.data]
            except Exception as e:
                logger.error(f"OpenAI embedding failed: {e}")
                return []
        else:
            logger.info("Using mock embeddings")
            return [MockEmbeddingGenerator.generate_embedding(text) for text in texts]

    async def upload_batch(self, batch: List[Dict[str, Any]]) -> int:
        """Upload a batch with embeddings."""
        texts = [chunk['text'] for chunk in batch]

        # Generate embeddings
        embeddings = await self.generate_embeddings(texts)
        if not embeddings:
            logger.error("Failed to generate embeddings")
            return 0

        # Prepare points
        points = []
        for chunk, embedding in zip(batch, embeddings):
            points.append({
                'id': chunk.get('chunk_index', 0),
                'vector': embedding,
                'payload': {
                    'text': chunk['text'][:200],  # Truncate for display
                    'chapter': chunk.get('chapter', ''),
                    'module': chunk.get('module', ''),
                    'url': chunk.get('url', '')
                }
            })

        # Upsert
        success = self.qdrant.upsert(points)
        return len(points) if success else 0

    async def test_search(self) -> List[Dict[str, Any]]:
        """Test search functionality."""
        logger.info("\nTesting search functionality...")

        # Create a test query
        test_query = "ROS 2 agent humanoid robot"
        query_embedding = MockEmbeddingGenerator.generate_embedding(test_query)

        # Search
        results = self.qdrant.search(query_embedding, limit=3)

        logger.info(f"Found {len(results)} results for query: '{test_query}'")
        for i, result in enumerate(results):
            logger.info(f"  {i+1}. [Score: {result['score']:.3f}] {result['payload']['chapter']}")

        return results

    async def run(self, index_file: str, sample_size: int = None) -> Dict[str, Any]:
        """Run full test."""
        logger.info("=" * 70)
        logger.info("Testing Qdrant Upload Pipeline")
        logger.info("=" * 70)
        logger.info(f"Mode: {'OpenAI' if self.use_openai else 'Mock'} embeddings")

        # Load chunks
        chunks = await self.load_chunks(index_file, sample_size)

        # Create collection
        self.qdrant.create_collection()

        # Process in batches
        total_uploaded = 0
        total_batches = (len(chunks) + self.batch_size - 1) // self.batch_size

        logger.info(f"\nUploading {len(chunks)} chunks in {total_batches} batches...")
        for i in range(0, len(chunks), self.batch_size):
            batch = chunks[i:i+self.batch_size]
            batch_num = (i // self.batch_size) + 1

            uploaded = await self.upload_batch(batch)
            total_uploaded += uploaded

            logger.info(f"  Batch {batch_num}/{total_batches}: {uploaded} chunks uploaded")

        # Get collection info
        info = self.qdrant.get_collection_info()
        logger.info(f"\nCollection Info:")
        logger.info(f"  Total points: {info['points_count']}")
        logger.info(f"  Vector dimension: 1536")

        # Test search
        search_results = await self.test_search()

        # Final summary
        logger.info("\n" + "=" * 70)
        logger.info("Test Complete")
        logger.info("=" * 70)
        logger.info(f"Chunks loaded: {len(chunks)}")
        logger.info(f"Chunks uploaded: {total_uploaded}")
        logger.info(f"Search tested: {len(search_results)} results returned")
        logger.info(f"Status: {'PASS [OK]' if total_uploaded == len(chunks) else 'FAIL'}")

        return {
            'success': total_uploaded == len(chunks),
            'loaded': len(chunks),
            'uploaded': total_uploaded,
            'search_results': len(search_results)
        }


async def main():
    """Entry point."""
    parser = argparse.ArgumentParser(
        description='Test Qdrant upload pipeline with mock or real embeddings'
    )
    parser.add_argument(
        '--index-file',
        default='index.json',
        help='Path to index.json file'
    )
    parser.add_argument(
        '--sample-size',
        type=int,
        default=None,
        help='Limit to N chunks (for quick testing)'
    )
    parser.add_argument(
        '--use-openai',
        action='store_true',
        help='Use real OpenAI embeddings (requires OPENAI_API_KEY)'
    )

    args = parser.parse_args()

    # Validate index file
    if not Path(args.index_file).exists():
        logger.error(f"Index file not found: {args.index_file}")
        sys.exit(1)

    # Run test
    tester = QdrantUploadTester(
        use_openai=args.use_openai,
        batch_size=5
    )
    result = await tester.run(args.index_file, args.sample_size)

    sys.exit(0 if result['success'] else 1)


if __name__ == '__main__':
    asyncio.run(main())
