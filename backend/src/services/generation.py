"""
OpenAI Generation Service

Generates responses using OpenAI Agents API with retrieved context.
"""

import logging
import os
from typing import List, Dict, Any, Optional
from openai import AsyncOpenAI

logger = logging.getLogger(__name__)


class GenerationService:
    """Generates responses using OpenAI Agents."""

    def __init__(self):
        """Initialize OpenAI client."""
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            logger.error("OPENAI_API_KEY not set in environment")
            self.client = None
        else:
            self.client = AsyncOpenAI(api_key=api_key)

    def _format_context(self, chunks: List[Dict[str, Any]]) -> str:
        """
        Format retrieved chunks into context for the prompt.

        Args:
            chunks: Retrieved chunk documents

        Returns:
            Formatted context string
        """
        if not chunks:
            return "No relevant content found."

        context_parts = []
        for i, chunk in enumerate(chunks, 1):
            text = chunk.get("text", "")
            chapter = chunk.get("chapter_title", "Unknown Chapter")
            section = chunk.get("section_heading", "")

            header = f"[Source {i}: {chapter}"
            if section:
                header += f" - {section}"
            header += "]"

            context_parts.append(f"{header}\n{text}\n")

        return "\n---\n".join(context_parts)

    async def generate_response(
        self,
        query: str,
        chunks: List[Dict[str, Any]],
        temperature: float = 0.7,
    ) -> Optional[str]:
        """
        Generate response to query based on retrieved chunks.

        Args:
            query: User query
            chunks: Retrieved context chunks
            temperature: OpenAI temperature parameter

        Returns:
            Generated response or None if failed
        """
        if not self.client:
            logger.error("OpenAI client not initialized")
            return None

        try:
            context = self._format_context(chunks)

            system_prompt = """You are a helpful AI assistant for a ROS 2 robotics course.
Your role is to answer questions about ROS 2 fundamentals, Python agent bridging, and URDF modeling.

IMPORTANT RULES:
1. Only answer based on the provided course content
2. If the answer is not in the provided content, say "This topic is not covered in Module 1"
3. Always cite the relevant chapter/section when providing information
4. Use clear, beginner-friendly language
5. Include code examples when relevant
6. Be concise but complete in your explanations"""

            user_message = f"""Course Content:
{context}

Question: {query}

Please provide a helpful answer based on the course content above."""

            response = await self.client.chat.completions.create(
                model="gpt-4-turbo-preview",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_message},
                ],
                temperature=temperature,
                max_tokens=1000,
            )

            answer = response.choices[0].message.content
            logger.info(f"Generated response ({len(answer)} chars)")
            return answer

        except Exception as e:
            logger.error(f"Generation failed: {e}")
            return None

    async def extract_sources(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, str]]:
        """
        Extract source information from chunks.

        Args:
            chunks: Retrieved chunks

        Returns:
            List of source citations
        """
        sources = []
        seen = set()

        for chunk in chunks:
            chapter = chunk.get("chapter_title", "Unknown")
            url = chunk.get("url", "#")
            excerpt = chunk.get("text", "")[:200] + "..."

            # Avoid duplicates
            source_key = (chapter, url)
            if source_key not in seen:
                sources.append({
                    "chapter": chapter,
                    "url": url,
                    "excerpt": excerpt,
                })
                seen.add(source_key)

        return sources

    async def health_check(self) -> bool:
        """Test OpenAI API connectivity."""
        if not self.client:
            return False

        try:
            # Try to create a simple completion
            await self.client.chat.completions.create(
                model="gpt-4-turbo-preview",
                messages=[{"role": "user", "content": "test"}],
                max_tokens=5,
            )
            return True
        except Exception as e:
            logger.error(f"Generation service health check failed: {e}")
            return False


# Global instance
_generation_service: GenerationService = None


def get_generation_service() -> GenerationService:
    """Get or create global generation service instance."""
    global _generation_service
    if _generation_service is None:
        _generation_service = GenerationService()
    return _generation_service
