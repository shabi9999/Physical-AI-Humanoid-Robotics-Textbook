"""
Neon PostgreSQL Database Connection Pool

Manages async connection pooling to Neon serverless Postgres for user data and query history.
"""

import logging
import os
from typing import Optional, List, Dict, Any
import asyncpg

logger = logging.getLogger(__name__)


class PostgresDatabase:
    """PostgreSQL connection pool manager."""

    def __init__(self):
        """Initialize database configuration."""
        self.database_url = os.getenv("NEON_DATABASE_URL")
        self.pool: Optional[asyncpg.Pool] = None

    async def connect(self) -> bool:
        """
        Establish connection pool to Neon Postgres.

        Returns:
            True if successful, False otherwise
        """
        if not self.database_url:
            logger.error("NEON_DATABASE_URL not set in environment")
            return False

        try:
            self.pool = await asyncpg.create_pool(
                self.database_url,
                min_size=5,
                max_size=20,
                command_timeout=10,
            )
            logger.info("Connected to Neon Postgres")

            # Initialize schema
            await self._init_schema()
            return True

        except Exception as e:
            logger.error(f"Failed to connect to Postgres: {e}")
            return False

    async def disconnect(self) -> bool:
        """Close connection pool."""
        if self.pool:
            await self.pool.close()
            logger.info("Disconnected from Postgres")
            return True
        return False

    async def _init_schema(self) -> None:
        """Initialize database schema if not exists."""
        if not self.pool:
            return

        async with self.pool.acquire() as conn:
            # Create users table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS users (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    email VARCHAR(255) UNIQUE NOT NULL,
                    created_at TIMESTAMP DEFAULT NOW()
                );
            """)

            # Create query_history table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS query_history (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    user_id UUID REFERENCES users(id) ON DELETE SET NULL,
                    query TEXT NOT NULL,
                    response TEXT,
                    mode VARCHAR(20),
                    retrieved_chunks JSONB,
                    feedback_rating INT CHECK (feedback_rating BETWEEN 1 AND 5),
                    created_at TIMESTAMP DEFAULT NOW()
                );
            """)

            # Create indexes
            await conn.execute("""
                CREATE INDEX IF NOT EXISTS idx_query_history_user_id
                ON query_history(user_id);
            """)

            await conn.execute("""
                CREATE INDEX IF NOT EXISTS idx_query_history_created_at
                ON query_history(created_at DESC);
            """)

            logger.info("Database schema initialized")

    async def add_query_to_history(
        self,
        query: str,
        response: str,
        mode: str = "rag",
        retrieved_chunks: Optional[List[str]] = None,
        user_id: Optional[str] = None,
    ) -> bool:
        """Record a query in the history table."""
        if not self.pool:
            return False

        try:
            async with self.pool.acquire() as conn:
                await conn.execute("""
                    INSERT INTO query_history (user_id, query, response, mode, retrieved_chunks)
                    VALUES ($1, $2, $3, $4, $5)
                """, user_id, query, response, mode, retrieved_chunks)

            logger.debug(f"Recorded query to history")
            return True

        except Exception as e:
            logger.error(f"Failed to record query: {e}")
            return False

    async def get_query_history(
        self,
        user_id: Optional[str] = None,
        limit: int = 10,
    ) -> List[Dict[str, Any]]:
        """Retrieve query history."""
        if not self.pool:
            return []

        try:
            async with self.pool.acquire() as conn:
                if user_id:
                    rows = await conn.fetch("""
                        SELECT * FROM query_history
                        WHERE user_id = $1
                        ORDER BY created_at DESC
                        LIMIT $2
                    """, user_id, limit)
                else:
                    rows = await conn.fetch("""
                        SELECT * FROM query_history
                        ORDER BY created_at DESC
                        LIMIT $1
                    """, limit)

                return [dict(row) for row in rows]

        except Exception as e:
            logger.error(f"Failed to retrieve query history: {e}")
            return []

    async def add_feedback(
        self,
        query_id: str,
        rating: int,
    ) -> bool:
        """Add user feedback rating to a query."""
        if not self.pool or rating < 1 or rating > 5:
            return False

        try:
            async with self.pool.acquire() as conn:
                await conn.execute("""
                    UPDATE query_history
                    SET feedback_rating = $1
                    WHERE id = $2
                """, rating, query_id)

            logger.debug(f"Recorded feedback for query")
            return True

        except Exception as e:
            logger.error(f"Failed to record feedback: {e}")
            return False

    async def health_check(self) -> bool:
        """Check database connection health."""
        if not self.pool:
            return False

        try:
            async with self.pool.acquire() as conn:
                await conn.fetchval("SELECT 1")
            return True
        except Exception as e:
            logger.error(f"Database health check failed: {e}")
            return False


# Global instance
_postgres_db: Optional[PostgresDatabase] = None


def get_postgres() -> PostgresDatabase:
    """Get or create global Postgres database instance."""
    global _postgres_db
    if _postgres_db is None:
        _postgres_db = PostgresDatabase()
    return _postgres_db
