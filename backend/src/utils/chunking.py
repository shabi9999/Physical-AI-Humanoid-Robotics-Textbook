"""
Document Chunking Utility

Splits documents into semantic chunks with configurable overlap for RAG system.
"""

import logging
from typing import List, Dict, Any
import re

logger = logging.getLogger(__name__)


class DocumentChunker:
    """Chunks documents into semantic pieces for RAG."""

    def __init__(self, chunk_size: int = 512, overlap: float = 0.2):
        """
        Initialize chunker.

        Args:
            chunk_size: Target chunk size in tokens
            overlap: Overlap ratio (0.2 = 20%)
        """
        self.chunk_size = chunk_size
        self.overlap_tokens = int(chunk_size * overlap)

    @staticmethod
    def estimate_tokens(text: str) -> int:
        """
        Estimate token count using simple heuristic.

        Args:
            text: Text to estimate

        Returns:
            Approximate token count
        """
        # Rough estimate: 1 token ≈ 4 characters
        # This is a simple heuristic; use actual tokenizer for accuracy
        return len(text) // 4

    def _split_by_sentences(self, text: str) -> List[str]:
        """
        Split text by sentences.

        Args:
            text: Text to split

        Returns:
            List of sentences
        """
        # Simple sentence splitter
        sentences = re.split(r'(?<=[.!?])\s+', text)
        return [s.strip() for s in sentences if s.strip()]

    def chunk_document(
        self,
        content: str,
        metadata: Dict[str, Any] = None,
    ) -> List[Dict[str, Any]]:
        """
        Chunk a document into semantic sections.

        Args:
            content: Document content
            metadata: Document metadata (module, chapter, etc.)

        Returns:
            List of chunks with metadata
        """
        if metadata is None:
            metadata = {}

        chunks = []
        lines = content.split('\n')

        current_chunk = ""
        current_section = ""
        chunk_index = 0

        for i, line in enumerate(lines):
            # Track section headings
            if line.startswith('##') and not line.startswith('###'):
                current_section = line.strip('#').strip()

            current_chunk += line + "\n"

            # Split when chunk reaches target size
            if self.estimate_tokens(current_chunk) >= self.chunk_size:
                chunk_text = current_chunk.strip()
                if chunk_text:
                    chunks.append({
                        "text": chunk_text,
                        "section_heading": current_section,
                        "chunk_index": chunk_index,
                        **metadata,
                    })
                    chunk_index += 1

                # Keep overlap
                overlap_size = self.overlap_tokens * 4  # Convert back to chars
                current_chunk = current_chunk[-overlap_size:] if len(current_chunk) > overlap_size else ""

        # Add final chunk
        if current_chunk.strip():
            chunks.append({
                "text": current_chunk.strip(),
                "section_heading": current_section,
                "chunk_index": chunk_index,
                **metadata,
            })

        logger.info(f"Chunked document into {len(chunks)} pieces")
        return chunks

    def chunk_texts(
        self,
        texts: List[str],
        metadata_list: List[Dict[str, Any]] = None,
    ) -> List[Dict[str, Any]]:
        """
        Chunk multiple documents.

        Args:
            texts: List of document contents
            metadata_list: List of metadata dicts (one per text)

        Returns:
            Combined list of all chunks
        """
        if metadata_list is None:
            metadata_list = [{}] * len(texts)

        all_chunks = []
        for text, metadata in zip(texts, metadata_list):
            chunks = self.chunk_document(text, metadata)
            all_chunks.extend(chunks)

        return all_chunks


# Preset chunkers
def get_default_chunker() -> DocumentChunker:
    """Get default chunker (512 tokens, 20% overlap)."""
    return DocumentChunker(chunk_size=512, overlap=0.2)


def get_small_chunker() -> DocumentChunker:
    """Get small chunker (256 tokens, 15% overlap)."""
    return DocumentChunker(chunk_size=256, overlap=0.15)


def get_large_chunker() -> DocumentChunker:
    """Get large chunker (800 tokens, 25% overlap)."""
    return DocumentChunker(chunk_size=800, overlap=0.25)
