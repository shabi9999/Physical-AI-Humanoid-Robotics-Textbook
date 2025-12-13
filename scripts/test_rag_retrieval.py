"""
Test RAG Retrieval System

This script tests the complete RAG (Retrieval-Augmented Generation) pipeline by:
1. Loading chunks from index.json
2. Creating mock embeddings
3. Simulating Qdrant vector search
4. Testing retrieval accuracy with domain-specific queries
5. Validating citation/source tracking

Usage:
    python scripts/test_rag_retrieval.py [--sample-size 10] [--verbose]

Tests RAG retrieval accuracy without requiring a live Qdrant instance or OpenAI API.
"""

import json
import logging
import sys
import asyncio
from pathlib import Path
from typing import List, Dict, Any
import hashlib
from collections import defaultdict

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class MockEmbeddingGenerator:
    """Generate deterministic embeddings from text for testing."""

    @staticmethod
    def generate_embedding(text: str, dimension: int = 1536) -> List[float]:
        """Generate deterministic embedding from text hash."""
        hash_obj = hashlib.md5(text.encode())
        hash_int = int(hash_obj.hexdigest(), 16)

        embeddings = []
        for i in range(dimension):
            seed = (hash_int + i) % (2**31)
            value = 2.0 * (seed / (2**31)) - 1.0
            embeddings.append(value)

        # Normalize
        norm = sum(v**2 for v in embeddings) ** 0.5
        if norm > 0:
            embeddings = [v / norm for v in embeddings]

        return embeddings

    @staticmethod
    def cosine_similarity(vec1: List[float], vec2: List[float]) -> float:
        """Calculate cosine similarity between two vectors."""
        if len(vec1) != len(vec2):
            return 0.0

        dot_product = sum(a * b for a, b in zip(vec1, vec2))
        norm1 = sum(v**2 for v in vec1) ** 0.5
        norm2 = sum(v**2 for v in vec2) ** 0.5

        if norm1 > 0 and norm2 > 0:
            return dot_product / (norm1 * norm2)
        return 0.0


class RAGRetrievalTester:
    """Test RAG retrieval system with domain queries."""

    def __init__(self):
        """Initialize tester."""
        self.chunks = []
        self.chunk_embeddings = {}
        self.test_queries = [
            ("How do I create a ROS 2 node?", "chapter1-ros2-core"),
            ("What is an agent and how does it integrate with ROS 2?", "chapter2-agent-bridge"),
            ("How do I create and visualize a humanoid URDF model?", "chapter3-urdf-model"),
            ("What are the differences between topics and services?", "chapter1-ros2-core"),
            ("How do I subscribe to sensor data in ROS 2?", "chapter2-agent-bridge"),
            ("What are links and joints in URDF?", "chapter3-urdf-model"),
            ("How do I publish control commands?", "chapter2-agent-bridge"),
        ]

    async def load_chunks(self, index_file: str, max_chunks: int = None) -> bool:
        """Load chunks from index.json."""
        index_path = Path(index_file)
        if not index_path.exists():
            logger.error(f"Index file not found: {index_file}")
            return False

        logger.info(f"Loading chunks from {index_file}")
        with open(index_path, 'r', encoding='utf-8') as f:
            self.chunks = json.load(f)

        if max_chunks:
            self.chunks = self.chunks[:max_chunks]

        logger.info(f"Loaded {len(self.chunks)} chunks")

        # Generate embeddings for all chunks
        logger.info("Generating embeddings for all chunks...")
        for i, chunk in enumerate(self.chunks):
            text = chunk.get('text', '')
            embedding = MockEmbeddingGenerator.generate_embedding(text)
            chunk_id = chunk.get('chunk_index', i)
            self.chunk_embeddings[chunk_id] = {
                'embedding': embedding,
                'chunk': chunk,
                'index': i
            }

        logger.info(f"Generated embeddings for {len(self.chunk_embeddings)} chunks")
        return True

    async def test_query(self, query: str, expected_chapter: str) -> Dict[str, Any]:
        """Test a single RAG query."""
        # Generate query embedding
        query_embedding = MockEmbeddingGenerator.generate_embedding(query)

        # Find most similar chunks
        similarities = []
        for chunk_id, data in self.chunk_embeddings.items():
            sim = MockEmbeddingGenerator.cosine_similarity(
                query_embedding,
                data['embedding']
            )
            chunk = data['chunk']

            similarities.append({
                'chunk_id': chunk_id,
                'similarity': sim,
                'chapter': chunk.get('chapter', 'unknown'),
                'text': chunk.get('text', '')[:200],
                'url': chunk.get('url', '')
            })

        # Sort by similarity
        similarities.sort(key=lambda x: x['similarity'], reverse=True)
        top_results = similarities[:5]

        # Check if top result matches expected chapter
        if top_results:
            top_chapter = top_results[0]['chapter']
            matched = top_chapter == expected_chapter
            mrr = 1.0 / (next((i + 1 for i, r in enumerate(top_results) if r['chapter'] == expected_chapter), len(top_results) + 1))
        else:
            matched = False
            mrr = 0.0

        return {
            'query': query,
            'expected_chapter': expected_chapter,
            'top_results': top_results,
            'matched': matched,
            'mrr': mrr,
            'top_chapter': top_results[0]['chapter'] if top_results else 'N/A'
        }

    async def run(self, index_file: str, sample_size: int = None, verbose: bool = False) -> Dict[str, Any]:
        """Run RAG retrieval tests."""
        logger.info("=" * 70)
        logger.info("Testing RAG Retrieval System")
        logger.info("=" * 70)

        # Load chunks
        if not await self.load_chunks(index_file, sample_size):
            return {'success': False, 'error': 'Failed to load chunks'}

        # Run queries
        logger.info(f"\nRunning {len(self.test_queries)} domain queries...")
        results = []
        matched_count = 0
        total_mrr = 0.0

        for query, expected_chapter in self.test_queries:
            result = await self.test_query(query, expected_chapter)
            results.append(result)

            # Log result
            match_indicator = "[PASS]" if result['matched'] else "[FAIL]"
            logger.info(f"\n{match_indicator} Query: {query}")
            logger.info(f"  Expected: {expected_chapter}")
            logger.info(f"  Got: {result['top_chapter']}")
            logger.info(f"  MRR: {result['mrr']:.3f}")

            if verbose:
                logger.info(f"  Top 3 results:")
                for i, res in enumerate(result['top_results'][:3]):
                    logger.info(f"    {i+1}. [{res['similarity']:.3f}] {res['chapter']} - {res['text']}")

            if result['matched']:
                matched_count += 1
            total_mrr += result['mrr']

        # Calculate metrics
        accuracy = matched_count / len(results) if results else 0.0
        mean_mrr = total_mrr / len(results) if results else 0.0

        # Summary
        logger.info("\n" + "=" * 70)
        logger.info("RAG Retrieval Test Results")
        logger.info("=" * 70)

        logger.info(f"\nQueries tested: {len(results)}")
        logger.info(f"Correct chapters returned: {matched_count}/{len(results)}")
        logger.info(f"Accuracy: {accuracy:.1%}")
        logger.info(f"Mean Reciprocal Rank (MRR): {mean_mrr:.3f}")

        # Group results by chapter
        logger.info(f"\nResults by Expected Chapter:")
        chapter_results = defaultdict(list)
        for result in results:
            chapter_results[result['expected_chapter']].append(result)

        for chapter in sorted(chapter_results.keys()):
            chapter_results_list = chapter_results[chapter]
            chapter_matches = sum(1 for r in chapter_results_list if r['matched'])
            logger.info(f"  {chapter}: {chapter_matches}/{len(chapter_results_list)} correct")

        # Determine overall success
        success = accuracy >= 0.7  # Target 70%+ accuracy

        logger.info(f"\nTarget accuracy: >= 70%")
        logger.info(f"Actual accuracy: {accuracy:.1%}")
        logger.info(f"Status: {'PASS [OK]' if success else 'FAIL'}")

        return {
            'success': success,
            'queries_tested': len(results),
            'correct': matched_count,
            'accuracy': accuracy,
            'mean_mrr': mean_mrr,
            'chapter_results': dict(chapter_results)
        }


async def main():
    """Entry point."""
    import argparse

    parser = argparse.ArgumentParser(
        description='Test RAG retrieval system with domain-specific queries'
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
        '--verbose',
        action='store_true',
        help='Show detailed results for each query'
    )

    args = parser.parse_args()

    # Validate index file
    if not Path(args.index_file).exists():
        logger.error(f"Index file not found: {args.index_file}")
        sys.exit(1)

    # Run tests
    tester = RAGRetrievalTester()
    result = await tester.run(
        args.index_file,
        args.sample_size,
        args.verbose
    )

    sys.exit(0 if result['success'] else 1)


if __name__ == '__main__':
    asyncio.run(main())
