"""
Upload Embeddings to Qdrant Vector Database

This script takes the indexed chunks from index.json and uploads them to Qdrant
with OpenAI embeddings. It handles batch processing, error recovery, and validation.

Usage:
    python scripts/upload_embeddings.py [--index-file index.json] [--batch-size 50]

Environment Variables Required:
    OPENAI_API_KEY - OpenAI API key for embeddings
    QDRANT_URL - Qdrant instance URL (default: http://localhost:6333)
    QDRANT_API_KEY - Qdrant API key (optional)
"""

import json
import logging
import os
import sys
import asyncio
import argparse
from pathlib import Path
from typing import List, Dict, Any
from datetime import datetime

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('upload_embeddings.log')
    ]
)
logger = logging.getLogger(__name__)

# Add backend to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "backend"))

from src.services.embeddings import EmbeddingsService
from src.db.qdrant import QdrantDatabase
from openai import AsyncOpenAI


class EmbeddingUploader:
    """Upload chunks to Qdrant with embeddings."""

    def __init__(self, batch_size: int = 50):
        """
        Initialize uploader.

        Args:
            batch_size: Number of chunks to process per batch
        """
        self.batch_size = batch_size
        self.embeddings_service = EmbeddingsService()
        self.qdrant_db = QdrantDatabase()
        self.chunks_processed = 0
        self.chunks_failed = 0
        self.total_tokens = 0

    async def load_chunks(self, index_file: str) -> List[Dict[str, Any]]:
        """
        Load chunks from index.json file.

        Args:
            index_file: Path to index.json file

        Returns:
            List of chunk dictionaries
        """
        index_path = Path(index_file)
        if not index_path.exists():
            raise FileNotFoundError(f"Index file not found: {index_file}")

        logger.info(f"Loading chunks from {index_file}")
        with open(index_path, 'r', encoding='utf-8') as f:
            chunks = json.load(f)

        logger.info(f"Loaded {len(chunks)} chunks")
        return chunks

    async def estimate_tokens(self, text: str) -> int:
        """
        Rough estimate of tokens in text (4 chars ≈ 1 token).

        Args:
            text: Text to estimate

        Returns:
            Estimated token count
        """
        return len(text) // 4

    async def upload_batch(self, batch: List[Dict[str, Any]]) -> tuple[int, int]:
        """
        Upload a batch of chunks with embeddings to Qdrant.

        Args:
            batch: List of chunks to upload

        Returns:
            Tuple of (successfully_uploaded, failed)
        """
        texts = [chunk['text'] for chunk in batch]
        chunk_ids = list(range(len(batch)))

        try:
            # Generate embeddings
            logger.info(f"Generating embeddings for {len(texts)} chunks...")
            embeddings = await self.embeddings_service.embed_texts(texts)

            if not embeddings or len(embeddings) != len(texts):
                logger.error(f"Failed to generate embeddings for batch")
                return 0, len(batch)

            # Prepare points for Qdrant
            points = []
            for i, (chunk, embedding) in enumerate(zip(batch, embeddings)):
                # Use chunk_index as point ID
                point_id = chunk.get('chunk_index', i)

                payload = {
                    'text': chunk['text'],
                    'section_heading': chunk.get('section_heading', ''),
                    'module': chunk.get('module', ''),
                    'chapter': chunk.get('chapter', ''),
                    'chapter_title': chunk.get('chapter_title', ''),
                    'url': chunk.get('url', ''),
                    'content_type': chunk.get('content_type', 'text'),
                    'file_path': chunk.get('file_path', ''),
                    'created_at': chunk.get('created_at', datetime.now().timestamp()),
                }

                points.append({
                    'id': point_id,
                    'vector': embedding,
                    'payload': payload
                })

            # Upload to Qdrant
            logger.info(f"Uploading {len(points)} points to Qdrant...")
            success = await self.qdrant_db.upsert(points)

            if success:
                logger.info(f"Successfully uploaded {len(points)} points")
                return len(points), 0
            else:
                logger.error(f"Failed to upload batch")
                return 0, len(batch)

        except Exception as e:
            logger.error(f"Error uploading batch: {e}")
            return 0, len(batch)

    async def run(self, index_file: str) -> Dict[str, Any]:
        """
        Main upload process.

        Args:
            index_file: Path to index.json file

        Returns:
            Summary statistics
        """
        logger.info("=" * 70)
        logger.info("Starting embedding upload to Qdrant")
        logger.info("=" * 70)

        # Load chunks
        chunks = await self.load_chunks(index_file)
        logger.info(f"Loaded {len(chunks)} chunks for processing")

        # Ensure Qdrant collection exists
        logger.info("Ensuring Qdrant collection exists...")
        if not self.qdrant_db.ensure_collection():
            logger.error("Failed to ensure collection exists")
            return {
                'success': False,
                'error': 'Could not create/access Qdrant collection'
            }

        # Check OpenAI API key
        if not os.getenv("OPENAI_API_KEY"):
            logger.error("OPENAI_API_KEY not set")
            return {
                'success': False,
                'error': 'OPENAI_API_KEY environment variable not set'
            }

        # Process in batches
        total_chunks = len(chunks)
        total_batches = (total_chunks + self.batch_size - 1) // self.batch_size

        logger.info(f"Processing {total_chunks} chunks in {total_batches} batches")

        for batch_num in range(total_batches):
            start_idx = batch_num * self.batch_size
            end_idx = min(start_idx + self.batch_size, total_chunks)
            batch = chunks[start_idx:end_idx]

            logger.info(f"\nBatch {batch_num + 1}/{total_batches} (chunks {start_idx}-{end_idx-1})")

            uploaded, failed = await self.upload_batch(batch)
            self.chunks_processed += uploaded
            self.chunks_failed += failed

            # Log progress
            logger.info(f"Progress: {self.chunks_processed}/{total_chunks} uploaded, {self.chunks_failed} failed")

        # Get collection info
        collection_info = await self.qdrant_db.get_collection_info()

        # Final report
        logger.info("\n" + "=" * 70)
        logger.info("Upload Complete")
        logger.info("=" * 70)
        logger.info(f"Total chunks processed: {total_chunks}")
        logger.info(f"Successfully uploaded: {self.chunks_processed}")
        logger.info(f"Failed uploads: {self.chunks_failed}")

        if collection_info:
            logger.info(f"\nCollection Info:")
            logger.info(f"  Name: {collection_info.get('name')}")
            logger.info(f"  Total points: {collection_info.get('points_count')}")
            logger.info(f"  Vector size: 1536 (text-embedding-3-small)")

        success = self.chunks_failed == 0
        logger.info(f"\nResult: {'SUCCESS' if success else 'PARTIAL FAILURE'}")

        return {
            'success': success,
            'total_chunks': total_chunks,
            'uploaded': self.chunks_processed,
            'failed': self.chunks_failed,
            'collection_info': collection_info
        }


async def main():
    """Entry point."""
    parser = argparse.ArgumentParser(
        description='Upload chunks to Qdrant with OpenAI embeddings'
    )
    parser.add_argument(
        '--index-file',
        default='index.json',
        help='Path to index.json file (default: index.json)'
    )
    parser.add_argument(
        '--batch-size',
        type=int,
        default=50,
        help='Number of chunks per batch (default: 50)'
    )

    args = parser.parse_args()

    # Validate index file exists
    if not Path(args.index_file).exists():
        logger.error(f"Index file not found: {args.index_file}")
        sys.exit(1)

    # Run uploader
    uploader = EmbeddingUploader(batch_size=args.batch_size)
    result = await uploader.run(args.index_file)

    # Exit with status
    sys.exit(0 if result['success'] else 1)


if __name__ == '__main__':
    asyncio.run(main())
