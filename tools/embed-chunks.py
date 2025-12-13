#!/usr/bin/env python3
"""
Embedding Generation Script for RAG System

Generates vector embeddings for chunks using OpenAI API.
Uses text-embedding-3-small model (1536 dimensions, efficient).

Features:
- Batch processing (20 chunks/batch to respect rate limits)
- Progress tracking and resumable processing
- Error handling and retry logic
- Cost estimation and tracking
- Output in JSONL format (chunks + embeddings)

Usage:
    python3 embed-chunks.py chunks.jsonl --output chunks-embedded.jsonl
    python3 embed-chunks.py chunks.jsonl --output chunks-embedded.jsonl --batch-size 10
    python3 embed-chunks.py chunks.jsonl --api-key $OPENAI_API_KEY

Requirements:
    - OPENAI_API_KEY environment variable
    - openai Python package (pip install openai)

Cost Estimate:
    - text-embedding-3-small: $0.02 per 1M tokens
    - 2,500 chunks × 600 tokens avg = 1.5M tokens = ~$0.03
"""

import json
import sys
import os
import time
import argparse
from pathlib import Path
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass, asdict
import hashlib


@dataclass
class EmbeddingConfig:
    """Configuration for embedding generation."""
    MODEL = 'text-embedding-3-small'
    EMBEDDING_DIMENSION = 1536
    BATCH_SIZE = 20  # Respect rate limits (200 requests per minute)
    RETRY_MAX_ATTEMPTS = 3
    RETRY_DELAY_SECONDS = 2


class OpenAIEmbedder:
    """Handles embedding generation via OpenAI API."""

    def __init__(self, api_key: Optional[str] = None, config: EmbeddingConfig = None):
        self.api_key = api_key or os.getenv('OPENAI_API_KEY')
        if not self.api_key:
            raise ValueError(
                'OPENAI_API_KEY environment variable not set. '
                'Set it or pass --api-key argument.'
            )

        self.config = config or EmbeddingConfig()
        self.total_tokens_used = 0
        self.total_requests = 0
        self.cost_estimate = 0

        # Import openai at runtime
        try:
            from openai import OpenAI
            self.client = OpenAI(api_key=self.api_key)
        except ImportError:
            raise ImportError(
                'openai package not installed. Install with: pip install openai'
            )

    def embed_batch(self, texts: List[str], retry_count: int = 0) -> Optional[List[List[float]]]:
        """
        Generate embeddings for a batch of texts.

        Args:
            texts: List of text strings to embed
            retry_count: Current retry attempt

        Returns:
            List of embeddings (each embedding is a 1536-dim vector)
        """
        try:
            response = self.client.embeddings.create(
                model=self.config.MODEL,
                input=texts,
                encoding_format='float'
            )

            # Extract embeddings
            embeddings = [item.embedding for item in response.data]

            # Track usage
            self.total_tokens_used += response.usage.prompt_tokens
            self.total_requests += 1

            # Update cost estimate ($0.02 per 1M tokens)
            self.cost_estimate = (self.total_tokens_used / 1_000_000) * 0.02

            return embeddings

        except Exception as e:
            if retry_count < self.config.RETRY_MAX_ATTEMPTS:
                print(f"  ⚠️  Request failed, retrying in {self.config.RETRY_DELAY_SECONDS}s...")
                print(f"      Error: {str(e)[:100]}")
                time.sleep(self.config.RETRY_DELAY_SECONDS)
                return self.embed_batch(texts, retry_count + 1)
            else:
                print(f"  ❌ Failed after {self.config.RETRY_MAX_ATTEMPTS} retries")
                print(f"      Error: {str(e)}")
                return None


class ChunkEmbedder:
    """Orchestrates reading chunks and generating embeddings."""

    def __init__(self, api_key: Optional[str] = None):
        self.embedder = OpenAIEmbedder(api_key)
        self.config = EmbeddingConfig()

    def read_chunks_jsonl(self, filepath: Path) -> List[Dict]:
        """Read chunks from JSONL file."""
        chunks = []
        with open(filepath, 'r') as f:
            for line_num, line in enumerate(f, 1):
                try:
                    chunk = json.loads(line)
                    chunks.append(chunk)
                except json.JSONDecodeError as e:
                    print(f"⚠️  Line {line_num}: Invalid JSON, skipping")
                    continue
        return chunks

    def embed_chunks(
        self,
        input_file: Path,
        output_file: Path,
        checkpoint_file: Optional[Path] = None
    ) -> Tuple[int, int]:
        """
        Embed all chunks from input file, writing results to output file.

        Args:
            input_file: Path to chunks.jsonl
            output_file: Path to output chunks-embedded.jsonl
            checkpoint_file: Optional file to track progress

        Returns:
            Tuple of (total_chunks, successfully_embedded)
        """
        print(f"\n📖 Reading chunks from {input_file}...")
        chunks = self.read_chunks_jsonl(input_file)
        total_chunks = len(chunks)
        print(f"✅ Loaded {total_chunks} chunks")

        # Load checkpoint if resuming
        processed_ids = set()
        if checkpoint_file and checkpoint_file.exists():
            print(f"📌 Loading checkpoint from {checkpoint_file}...")
            with open(checkpoint_file, 'r') as f:
                processed_ids = set(line.strip() for line in f)
            print(f"   Resuming from {len(processed_ids)} processed chunks")

        # Process chunks in batches
        output_count = 0
        batch_start = 0

        print(f"\n🔄 Embedding chunks in batches of {self.config.BATCH_SIZE}...")

        while batch_start < total_chunks:
            batch_end = min(batch_start + self.config.BATCH_SIZE, total_chunks)
            batch = chunks[batch_start:batch_end]

            # Filter out already processed chunks
            batch_to_process = [
                c for c in batch
                if c.get('id') not in processed_ids
            ]

            if not batch_to_process:
                batch_start = batch_end
                continue

            # Extract texts and prepare for embedding
            texts = [c['content'] for c in batch_to_process]
            chunk_ids = [c['id'] for c in batch_to_process]

            print(f"\n  Processing batch {batch_start // self.config.BATCH_SIZE + 1} "
                  f"(chunks {batch_start + 1}-{batch_end})...")

            # Generate embeddings
            embeddings = self.embedder.embed_batch(texts)

            if embeddings is None:
                print(f"  ⚠️  Batch failed, skipping chunks {batch_start + 1}-{batch_end}")
                batch_start = batch_end
                continue

            # Write results
            with open(output_file, 'a') as f:
                for chunk_idx, embedding in enumerate(embeddings):
                    original_chunk = batch_to_process[chunk_idx]

                    # Add embedding to chunk
                    result = {**original_chunk, 'embedding': embedding}

                    f.write(json.dumps(result) + '\n')
                    processed_ids.add(original_chunk['id'])
                    output_count += 1

            # Update checkpoint
            if checkpoint_file:
                with open(checkpoint_file, 'w') as f:
                    for chunk_id in processed_ids:
                        f.write(chunk_id + '\n')

            print(f"  ✅ Wrote {len(embeddings)} embeddings")
            print(f"     Cumulative: {output_count}/{total_chunks} chunks")
            print(f"     Tokens used: {self.embedder.total_tokens_used:,}")
            print(f"     Cost so far: ${self.embedder.cost_estimate:.3f}")

            batch_start = batch_end

            # Rate limiting: wait between batches (to respect rate limits)
            # OpenAI: 200 requests/min for free tier
            # 20 chunks/batch = 200 chunks/min = 10 batches/min = 1 batch/6 seconds
            if batch_start < total_chunks:
                time.sleep(1)  # 1 second between batches

        return total_chunks, output_count

    def generate_summary(
        self,
        input_file: Path,
        output_file: Path,
        summary_file: Path
    ):
        """Generate summary statistics."""
        with open(output_file, 'r') as f:
            embedded_chunks = [json.loads(line) for line in f]

        summary = {
            'total_chunks': len(embedded_chunks),
            'embedding_model': self.config.MODEL,
            'embedding_dimensions': self.config.EMBEDDING_DIMENSION,
            'total_tokens_used': self.embedder.total_tokens_used,
            'total_api_requests': self.embedder.total_requests,
            'estimated_cost': round(self.embedder.cost_estimate, 4),
            'chapters': self._count_by_chapter(embedded_chunks),
            'difficulty_distribution': self._count_by_difficulty(embedded_chunks),
            'output_file': str(output_file)
        }

        with open(summary_file, 'w') as f:
            json.dump(summary, f, indent=2)

        return summary

    @staticmethod
    def _count_by_chapter(chunks: List[Dict]) -> Dict[str, int]:
        """Count chunks by chapter."""
        counts = {}
        for chunk in chunks:
            chapter = chunk.get('chapter_id', 'unknown')
            counts[chapter] = counts.get(chapter, 0) + 1
        return dict(sorted(counts.items()))

    @staticmethod
    def _count_by_difficulty(chunks: List[Dict]) -> Dict[str, int]:
        """Count chunks by difficulty level."""
        counts = {}
        for chunk in chunks:
            difficulty = chunk.get('difficulty_level', 'Unknown')
            counts[difficulty] = counts.get(difficulty, 0) + 1
        return counts


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Generate embeddings for chunks using OpenAI API'
    )
    parser.add_argument(
        'input_file',
        help='Input JSONL file with chunks'
    )
    parser.add_argument(
        '--output',
        default='chunks-embedded.jsonl',
        help='Output JSONL file with embeddings'
    )
    parser.add_argument(
        '--summary',
        default='embedding-summary.json',
        help='Summary statistics file'
    )
    parser.add_argument(
        '--api-key',
        help='OpenAI API key (defaults to OPENAI_API_KEY env var)'
    )
    parser.add_argument(
        '--batch-size',
        type=int,
        default=20,
        help='Batch size for API requests'
    )
    parser.add_argument(
        '--checkpoint',
        help='Checkpoint file for resuming (optional)'
    )

    args = parser.parse_args()

    # Validate input file
    input_path = Path(args.input_file)
    if not input_path.exists():
        print(f"❌ Input file not found: {input_path}")
        sys.exit(1)

    output_path = Path(args.output)
    summary_path = Path(args.summary)
    checkpoint_path = Path(args.checkpoint) if args.checkpoint else None

    # Clear output file if not resuming
    if not checkpoint_path or not checkpoint_path.exists():
        output_path.unlink(missing_ok=True)

    try:
        embedder = ChunkEmbedder(api_key=args.api_key)
        embedder.config.BATCH_SIZE = args.batch_size

        # Embed chunks
        total, processed = embedder.embed_chunks(
            input_path,
            output_path,
            checkpoint_path
        )

        # Generate summary
        print(f"\n📊 Generating summary...")
        summary = embedder.generate_summary(
            input_path,
            output_path,
            summary_path
        )

        # Print results
        print(f"\n{'='*80}")
        print(f"EMBEDDING GENERATION COMPLETE")
        print(f"{'='*80}")
        print(f"Total Chunks:                {total}")
        print(f"Embedded:                    {processed}")
        print(f"API Requests:                {embedder.embedder.total_requests}")
        print(f"Tokens Used:                 {embedder.embedder.total_tokens_used:,}")
        print(f"Estimated Cost:              ${embedder.embedder.cost_estimate:.4f}")
        print(f"\n✅ Output: {output_path}")
        print(f"✅ Summary: {summary_path}")

    except ValueError as e:
        print(f"❌ Configuration error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
