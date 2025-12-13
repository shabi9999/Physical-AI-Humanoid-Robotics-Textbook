"""
FastAPI RAG Backend

Main application entry point for ROS 2 course RAG chatbot backend.
"""

import logging
import os
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.api.query import router as query_router
from src.db.postgres import get_postgres

# Configure logging
logging.basicConfig(
    level=os.getenv("LOG_LEVEL", "info").upper(),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Application lifecycle management.

    Handles startup and shutdown of database connections.
    """
    # Startup
    logger.info("Starting ROS 2 RAG Backend...")

    postgres = get_postgres()
    postgres_connected = await postgres.connect()

    if not postgres_connected:
        logger.warning("Failed to connect to Postgres - some features may be unavailable")

    logger.info("Application startup complete")

    yield

    # Shutdown
    logger.info("Shutting down ROS 2 RAG Backend...")
    await postgres.disconnect()
    logger.info("Application shutdown complete")


# Create FastAPI application
app = FastAPI(
    title="ROS 2 Humanoid Robotics RAG Backend",
    description="AI-powered Q&A system for ROS 2 educational course",
    version="1.0.0",
    lifespan=lifespan,
)

# Configure CORS
frontend_url = os.getenv("FRONTEND_URL", "http://localhost:3000")
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        frontend_url,
        "http://localhost:3000",
        "http://localhost:8000",
        "*",  # Allow all origins in development
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routes
app.include_router(query_router)


@app.get("/")
async def root():
    """Root endpoint with API information."""
    return {
        "message": "ROS 2 RAG Backend",
        "docs": "/docs",
        "openapi": "/openapi.json",
    }


@app.get("/api/health")
async def health():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "service": "ros2-rag-backend",
    }


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=int(os.getenv("PORT", 8000)),
        reload=os.getenv("DEBUG", "False").lower() == "true",
    )
