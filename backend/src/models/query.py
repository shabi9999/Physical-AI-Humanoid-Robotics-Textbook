"""
RAG Query Models

Pydantic models for RAG API request/response validation.
"""

from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
from uuid import UUID


class QueryRequest(BaseModel):
    """RAG query request model."""

    query: str = Field(
        ...,
        min_length=1,
        max_length=500,
        description="User query text"
    )
    user_id: Optional[str] = Field(
        None,
        description="Optional user ID for tracking"
    )
    selected_text: Optional[str] = Field(
        None,
        description="User-selected text override for RAG"
    )

    class Config:
        example = {
            "query": "How do I create a ROS 2 node?",
            "user_id": "user-123",
            "selected_text": None,
        }


class SourceCitation(BaseModel):
    """Source citation model."""

    chapter: str = Field(..., description="Chapter title")
    url: str = Field(..., description="URL to chapter section")
    excerpt: str = Field(..., description="Excerpt from source")


class QueryResponse(BaseModel):
    """RAG query response model."""

    answer: str = Field(..., description="Generated response")
    sources: List[SourceCitation] = Field(..., description="Source citations")
    mode: str = Field(
        default="rag",
        description="Response mode (rag or override)"
    )
    metadata: Dict[str, Any] = Field(
        default_factory=dict,
        description="Response metadata"
    )

    class Config:
        example = {
            "answer": "To create a ROS 2 node, you use the rclpy library...",
            "sources": [
                {
                    "chapter": "Chapter 1: ROS 2 Core Concepts",
                    "url": "/module1/chapter1-ros2-core#creating-nodes",
                    "excerpt": "A ROS 2 node is created by inheriting...",
                }
            ],
            "mode": "rag",
            "metadata": {
                "query_time_ms": 2847,
                "chunks_retrieved": 3,
            },
        }


class HealthResponse(BaseModel):
    """Health check response model."""

    status: str = Field(..., description="Service status")
    services: Dict[str, str] = Field(..., description="Status of each service")
    version: str = Field(..., description="API version")


class ErrorResponse(BaseModel):
    """Error response model."""

    error: str = Field(..., description="Error code")
    message: str = Field(..., description="Error message")
    retry_after: Optional[int] = Field(
        None,
        description="Seconds to wait before retry (for rate limits)"
    )


class FeedbackRequest(BaseModel):
    """User feedback request model."""

    query_id: UUID = Field(..., description="Query ID to provide feedback for")
    rating: int = Field(
        ...,
        ge=1,
        le=5,
        description="Rating from 1 to 5"
    )
    comment: Optional[str] = Field(
        None,
        max_length=500,
        description="Optional feedback comment"
    )
