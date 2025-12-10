#!/usr/bin/env python3
"""
Database Loader for RAG System

Loads embedded chunks into:
1. Neon Postgres (relational metadata)
2. Qdrant (vector store for semantic search)

Features:
- Creates database schema (migrations)
- Loads chunks to Postgres chunks table
- Loads embeddings to Qdrant vector store
- Handles duplicates (skip or update)
- Validates data consistency
- Generates load report

Usage:
    python3 load-database.py chunks-embedded.jsonl \
        --postgres-url postgresql://user:pass@host/db \
        --qdrant-url http://localhost:6333

Requirements:
    - psycopg2 (pip install psycopg2-binary)
    - qdrant-client (pip install qdrant-client)

Environment Variables:
    DATABASE_URL — Postgres connection string
    QDRANT_URL — Qdrant server URL
"""

import json
import sys
import os
import argparse
from pathlib import Path
from typing import List, Dict, Optional, Tuple
from dataclasses import asdict
import uuid


class PostgresLoader:
    """Handles loading data to Neon Postgres."""

    # DDL for chunks table (from Phase 2 schema)
    CHUNKS_TABLE_DDL = """
    CREATE TABLE IF NOT EXISTS chunks (
        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
        chapter_id VARCHAR(64) NOT NULL,
        module_number INTEGER NOT NULL CHECK (module_number >= 1 AND module_number <= 4),
        section_name VARCHAR(256),
        heading VARCHAR(256),
        content TEXT NOT NULL,
        token_count INTEGER NOT NULL,
        keywords TEXT[] NOT NULL DEFAULT ARRAY[]::TEXT[],
        learning_objectives TEXT[] DEFAULT ARRAY[]::TEXT[],
        difficulty_level VARCHAR(32) NOT NULL CHECK (difficulty_level IN ('Beginner', 'Intermediate', 'Advanced')),
        url_anchor VARCHAR(512) NOT NULL,
        embedding vector(1536),
        created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
        updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
        version INTEGER DEFAULT 1
    );

    CREATE INDEX IF NOT EXISTS idx_chunks_module ON chunks(module_number);
    CREATE INDEX IF NOT EXISTS idx_chunks_difficulty ON chunks(difficulty_level);
    CREATE INDEX IF NOT EXISTS idx_chunks_chapter_id ON chunks(chapter_id);
    CREATE INDEX IF NOT EXISTS idx_chunks_keywords ON chunks USING GIN(keywords);
    CREATE INDEX IF NOT EXISTS idx_chunks_embedding ON chunks USING ivfflat(embedding vector_cosine_ops) WITH (lists = 100);
    """

    def __init__(self, postgres_url: Optional[str] = None):
        self.postgres_url = postgres_url or os.getenv('DATABASE_URL')
        if not self.postgres_url:
            raise ValueError(
                'Postgres URL not provided. Use --postgres-url or DATABASE_URL env var.'
            )

        try:
            import psycopg2
            from psycopg2.extras import execute_values
            self.psycopg2 = psycopg2
            self.execute_values = execute_values
            self.connection = None
        except ImportError:
            raise ImportError(
                'psycopg2 package not installed. Install with: pip install psycopg2-binary'
            )

    def connect(self):
        """Connect to Postgres."""
        try:
            self.connection = self.psycopg2.connect(self.postgres_url)
            print("✅ Connected to Postgres")
        except self.psycopg2.Error as e:
            print(f"❌ Failed to connect to Postgres: {e}")
            sys.exit(1)

    def disconnect(self):
        """Close connection."""
        if self.connection:
            self.connection.close()

    def execute(self, sql: str, params=None) -> bool:
        """Execute SQL statement."""
        try:
            cursor = self.connection.cursor()
            if params:
                cursor.execute(sql, params)
            else:
                cursor.execute(sql)
            self.connection.commit()
            return True
        except self.psycopg2.Error as e:
            print(f"  ⚠️  SQL Error: {e}")
            self.connection.rollback()
            return False

    def create_schema(self) -> bool:
        """Create chunks table and indexes."""
        print("\n🗄️  Creating Postgres schema...")

        # Split DDL into individual statements
        statements = [
            s.strip() for s in self.CHUNKS_TABLE_DDL.split(';')
            if s.strip()
        ]

        for statement in statements:
            if statement:
                print(f"  Executing: {statement[:60]}...")
                if not self.execute(statement + ';'):
                    return False

        print("✅ Schema created successfully")
        return True

    def load_chunks(self, chunks: List[Dict], skip_duplicates: bool = True) -> Tuple[int, int]:
        """
        Load chunks to Postgres.

        Args:
            chunks: List of chunk dicts with embeddings
            skip_duplicates: Skip chunks that already exist

        Returns:
            Tuple of (total_chunks, successfully_loaded)
        """
        if not chunks:
            return 0, 0

        print(f"\n📥 Loading {len(chunks)} chunks to Postgres...")

        # Prepare data for bulk insert
        rows = []
        for chunk in chunks:
            rows.append((
                uuid.UUID(chunk['id']),
                chunk['chapter_id'],
                chunk['module_number'],
                chunk['section_name'],
                chunk['heading'],
                chunk['content'],
                chunk['token_count'],
                chunk['keywords'],  # PostgreSQL array
                chunk.get('learning_objectives', []),
                chunk['difficulty_level'],
                chunk['url_anchor'],
                chunk.get('embedding'),  # Vector
            ))

        # Bulk insert with ON CONFLICT SKIP
        cursor = self.connection.cursor()
        sql = """
        INSERT INTO chunks (
            id, chapter_id, module_number, section_name, heading,
            content, token_count, keywords, learning_objectives,
            difficulty_level, url_anchor, embedding
        ) VALUES %s
        """

        if skip_duplicates:
            sql += " ON CONFLICT (id) DO NOTHING"
        else:
            sql += """ ON CONFLICT (id) DO UPDATE SET
                content = EXCLUDED.content,
                updated_at = CURRENT_TIMESTAMP,
                version = version + 1
            """

        try:
            self.execute_values(cursor, sql, rows, page_size=100)
            self.connection.commit()
            print(f"✅ Loaded {len(chunks)} chunks")
            return len(chunks), len(chunks)
        except Exception as e:
            print(f"❌ Error loading chunks: {e}")
            self.connection.rollback()
            return len(chunks), 0

    def verify_load(self) -> Dict:
        """Verify data was loaded correctly."""
        cursor = self.connection.cursor()

        # Count total chunks
        cursor.execute("SELECT COUNT(*) FROM chunks;")
        total_count = cursor.fetchone()[0]

        # Count by module
        cursor.execute("""
            SELECT module_number, COUNT(*)
            FROM chunks
            GROUP BY module_number
            ORDER BY module_number;
        """)
        by_module = dict(cursor.fetchall())

        # Count by difficulty
        cursor.execute("""
            SELECT difficulty_level, COUNT(*)
            FROM chunks
            GROUP BY difficulty_level;
        """)
        by_difficulty = dict(cursor.fetchall())

        # Average tokens
        cursor.execute("SELECT AVG(token_count), MIN(token_count), MAX(token_count) FROM chunks;")
        avg_tokens, min_tokens, max_tokens = cursor.fetchone()

        return {
            'total_chunks': total_count,
            'by_module': by_module,
            'by_difficulty': by_difficulty,
            'avg_tokens': round(float(avg_tokens or 0), 1),
            'min_tokens': int(min_tokens or 0),
            'max_tokens': int(max_tokens or 0)
        }


class QdrantLoader:
    """Handles loading embeddings to Qdrant."""

    def __init__(self, qdrant_url: Optional[str] = None):
        self.qdrant_url = qdrant_url or os.getenv('QDRANT_URL', 'http://localhost:6333')

        try:
            from qdrant_client import QdrantClient
            from qdrant_client.models import Distance, VectorParams, PointStruct
            self.QdrantClient = QdrantClient
            self.Distance = Distance
            self.VectorParams = VectorParams
            self.PointStruct = PointStruct
            self.client = QdrantClient(url=self.qdrant_url)
        except ImportError:
            raise ImportError(
                'qdrant-client package not installed. Install with: pip install qdrant-client'
            )

    def create_collection(self, collection_name: str = 'textbook_chunks') -> bool:
        """Create Qdrant collection."""
        print(f"\n📦 Creating Qdrant collection: {collection_name}...")

        try:
            # Check if collection exists
            collections = self.client.get_collections()
            if any(c.name == collection_name for c in collections.collections):
                print(f"   Collection already exists, skipping creation")
                return True

            # Create collection with 1536-dim vectors (OpenAI embedding size)
            self.client.create_collection(
                collection_name=collection_name,
                vectors_config=self.VectorParams(
                    size=1536,
                    distance=self.Distance.COSINE
                )
            )
            print(f"✅ Collection created: {collection_name}")
            return True
        except Exception as e:
            print(f"❌ Error creating collection: {e}")
            return False

    def load_embeddings(
        self,
        chunks: List[Dict],
        collection_name: str = 'textbook_chunks'
    ) -> Tuple[int, int]:
        """
        Load embeddings to Qdrant.

        Args:
            chunks: List of chunk dicts with embeddings
            collection_name: Qdrant collection name

        Returns:
            Tuple of (total_chunks, successfully_loaded)
        """
        if not chunks:
            return 0, 0

        print(f"\n📥 Loading {len(chunks)} embeddings to Qdrant...")

        points = []
        for chunk in chunks:
            if not chunk.get('embedding'):
                print(f"⚠️  Chunk {chunk['id']} missing embedding, skipping")
                continue

            point = self.PointStruct(
                id=chunk['id'][:16],  # Use first 16 chars of UUID for Qdrant ID
                vector=chunk['embedding'],
                payload={
                    'chunk_id': chunk['id'],
                    'chapter_id': chunk['chapter_id'],
                    'module_number': chunk['module_number'],
                    'section_name': chunk['section_name'],
                    'heading': chunk['heading'],
                    'url_anchor': chunk['url_anchor'],
                    'difficulty_level': chunk['difficulty_level'],
                    'keywords': chunk['keywords'],
                    'token_count': chunk['token_count']
                }
            )
            points.append(point)

        # Upsert points (insert or update)
        try:
            self.client.upsert(
                collection_name=collection_name,
                points=points,
                wait=True
            )
            print(f"✅ Loaded {len(points)} embeddings to Qdrant")
            return len(chunks), len(points)
        except Exception as e:
            print(f"❌ Error loading embeddings: {e}")
            return len(chunks), 0

    def verify_load(self, collection_name: str = 'textbook_chunks') -> Dict:
        """Verify embeddings were loaded."""
        try:
            collection_info = self.client.get_collection(collection_name)
            return {
                'collection_name': collection_name,
                'vector_count': collection_info.points_count,
                'vector_size': 1536,
                'distance_metric': 'cosine'
            }
        except Exception as e:
            print(f"⚠️  Error verifying Qdrant load: {e}")
            return {}


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Load embedded chunks to Postgres + Qdrant'
    )
    parser.add_argument(
        'input_file',
        help='Input JSONL file with embedded chunks'
    )
    parser.add_argument(
        '--postgres-url',
        help='Postgres URL (defaults to DATABASE_URL env var)'
    )
    parser.add_argument(
        '--qdrant-url',
        help='Qdrant URL (defaults to QDRANT_URL env var or http://localhost:6333)'
    )
    parser.add_argument(
        '--skip-postgres',
        action='store_true',
        help='Skip Postgres loading'
    )
    parser.add_argument(
        '--skip-qdrant',
        action='store_true',
        help='Skip Qdrant loading'
    )

    args = parser.parse_args()

    # Validate input
    input_path = Path(args.input_file)
    if not input_path.exists():
        print(f"❌ Input file not found: {input_path}")
        sys.exit(1)

    # Read chunks
    print(f"📖 Reading embedded chunks from {input_path}...")
    chunks = []
    with open(input_path, 'r') as f:
        for line_num, line in enumerate(f, 1):
            try:
                chunk = json.loads(line)
                chunks.append(chunk)
            except json.JSONDecodeError:
                print(f"⚠️  Line {line_num}: Invalid JSON, skipping")

    print(f"✅ Loaded {len(chunks)} chunks")

    # Load to Postgres
    postgres_stats = {}
    if not args.skip_postgres:
        try:
            pg_loader = PostgresLoader(args.postgres_url)
            pg_loader.connect()
            pg_loader.create_schema()
            total, loaded = pg_loader.load_chunks(chunks)
            postgres_stats = pg_loader.verify_load()
            pg_loader.disconnect()
        except (ValueError, ImportError) as e:
            print(f"⚠️  Postgres loading skipped: {e}")

    # Load to Qdrant
    qdrant_stats = {}
    if not args.skip_qdrant:
        try:
            qd_loader = QdrantLoader(args.qdrant_url)
            qd_loader.create_collection()
            total, loaded = qd_loader.load_embeddings(chunks)
            qdrant_stats = qd_loader.verify_load()
        except (ValueError, ImportError) as e:
            print(f"⚠️  Qdrant loading skipped: {e}")

    # Print summary
    print(f"\n{'='*80}")
    print(f"DATABASE LOAD SUMMARY")
    print(f"{'='*80}")
    print(f"Total Chunks:                {len(chunks)}")

    if postgres_stats:
        print(f"\nPostgres:")
        print(f"  Total chunks:              {postgres_stats.get('total_chunks', 'N/A')}")
        print(f"  By module:                 {postgres_stats.get('by_module', {})}")
        print(f"  By difficulty:             {postgres_stats.get('by_difficulty', {})}")
        print(f"  Avg tokens:                {postgres_stats.get('avg_tokens', 'N/A')}")

    if qdrant_stats:
        print(f"\nQdrant:")
        print(f"  Vectors loaded:            {qdrant_stats.get('vector_count', 'N/A')}")
        print(f"  Collection:                {qdrant_stats.get('collection_name', 'N/A')}")

    print(f"\n✅ Database load complete!")


if __name__ == '__main__':
    main()
