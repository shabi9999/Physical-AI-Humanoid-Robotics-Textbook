"""
RAG Query Endpoint

Implements the POST /api/query endpoint for RAG-based question answering.
"""

import logging
import time
from typing import Optional
from fastapi import APIRouter, HTTPException, BackgroundTasks
from src.models.query import QueryRequest, QueryResponse, SourceCitation
from src.services.retrieval import get_retrieval_service
from src.services.generation import get_generation_service
from src.db.postgres import get_postgres

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["RAG"])


# Rate limiting state (simplified; use Redis in production)
query_count = {}
RATE_LIMIT_WINDOW = 60  # seconds
RATE_LIMIT_QUERIES = 10


@router.post("/query", response_model=QueryResponse)
async def query_rag(request: QueryRequest, background_tasks: BackgroundTasks):
    """
    Process a RAG query and return an AI-generated response.

    Args:
        request: Query request with query text and optional user_id

    Returns:
        QueryResponse with answer, sources, and metadata

    Raises:
        HTTPException: If rate limited, error occurs, or service unavailable
    """
    start_time = time.time()

    try:
        # Rate limiting check
        user_id = request.user_id or "anonymous"
        current_time = int(time.time())

        if user_id not in query_count:
            query_count[user_id] = {"count": 0, "window_start": current_time}

        user_queries = query_count[user_id]

        # Reset window if expired
        if current_time - user_queries["window_start"] > RATE_LIMIT_WINDOW:
            user_queries["count"] = 0
            user_queries["window_start"] = current_time

        # Check rate limit
        if user_queries["count"] >= RATE_LIMIT_QUERIES:
            retry_after = RATE_LIMIT_WINDOW - (current_time - user_queries["window_start"])
            raise HTTPException(
                status_code=429,
                detail={
                    "error": "rate_limit_exceeded",
                    "message": f"Maximum {RATE_LIMIT_QUERIES} queries per minute",
                    "retry_after": retry_after,
                }
            )

        user_queries["count"] += 1

        # Get services
        retrieval_service = get_retrieval_service()
        generation_service = get_generation_service()

        # Retrieve chunks
        logger.info(f"Processing query from {user_id}: {request.query[:50]}...")

        if request.selected_text:
            # Use selected text as override
            chunks = [{"text": request.selected_text, "chapter": "User Selection", "url": "#"}]
            mode = "override"
        else:
            # Standard RAG retrieval
            chunks = await retrieval_service.retrieve_chunks(
                query=request.query,
                top_k=20,
                score_threshold=0.5,
            )
            mode = "rag"

        if not chunks:
            logger.warning(f"No chunks found for query: {request.query}")
            answer = "I couldn't find relevant information about that topic in the course materials. Please check the troubleshooting section or ask a more specific question."
            sources = []
        else:
            # Generate response
            answer = await generation_service.generate_response(
                query=request.query,
                chunks=chunks,
                temperature=0.7,
            )

            if not answer:
                logger.error("Failed to generate response")
                raise HTTPException(
                    status_code=500,
                    detail="Failed to generate response"
                )

            # Extract sources
            sources = await generation_service.extract_sources(chunks)

        query_time = (time.time() - start_time) * 1000

        response = QueryResponse(
            answer=answer,
            sources=[SourceCitation(**s) for s in sources],
            mode=mode,
            metadata={
                "query_time_ms": round(query_time),
                "chunks_retrieved": len(chunks),
            }
        )

        # Record query in background
        if user_id != "anonymous":
            background_tasks.add_task(
                record_query,
                user_id,
                request.query,
                answer,
                mode,
                [c.get("id") for c in chunks],
            )

        logger.info(f"Query processed in {query_time:.0f}ms, returned {len(sources)} sources")
        return response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Query processing failed: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail="An error occurred while processing your query"
        )


async def record_query(
    user_id: str,
    query: str,
    response: str,
    mode: str,
    chunk_ids: list,
):
    """Background task to record query in database."""
    try:
        postgres = get_postgres()
        await postgres.add_query_to_history(
            query=query,
            response=response,
            mode=mode,
            retrieved_chunks=chunk_ids,
            user_id=user_id,
        )
        logger.debug(f"Recorded query for user {user_id}")
    except Exception as e:
        logger.error(f"Failed to record query: {e}")


@router.get("/health")
async def health_check():
    """
    Health check endpoint for backend services.

    Returns:
        Health status of all services
    """
    try:
        retrieval_service = get_retrieval_service()
        generation_service = get_generation_service()
        postgres = get_postgres()

        # Check each service
        retrieval_healthy = await retrieval_service.health_check()
        generation_healthy = await generation_service.health_check()
        postgres_healthy = await postgres.health_check()

        overall_status = "healthy" if all([
            retrieval_healthy,
            generation_healthy,
            postgres_healthy,
        ]) else "degraded"

        return {
            "status": overall_status,
            "services": {
                "qdrant": "connected" if retrieval_healthy else "error",
                "openai": "connected" if generation_healthy else "error",
                "postgres": "connected" if postgres_healthy else "error",
            },
            "version": "1.0.0",
        }

    except Exception as e:
        logger.error(f"Health check failed: {e}")
        return {
            "status": "unhealthy",
            "services": {
                "qdrant": "unknown",
                "openai": "unknown",
                "postgres": "unknown",
            },
            "version": "1.0.0",
        }
