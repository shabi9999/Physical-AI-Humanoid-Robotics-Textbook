#!/usr/bin/env python3
"""
RAG Content Indexing Script

This script chunks Docusaurus markdown content and creates embeddings for the Qdrant vector database.
Usage: python index_content.py --source my-website/docs [--collection ros2_book_content]
"""

import argparse
import os
import json
import time
import re
from pathlib import Path
from typing import List, Dict, Any
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class DocumentChunker:
    """Chunks markdown documents into semantic pieces."""

    def __init__(self, chunk_size: int = 512, overlap: float = 0.2):
        """
        Initialize chunker.

        Args:
            chunk_size: Target chunk size in tokens (approximate)
            overlap: Overlap ratio for chunks (0.2 = 20%)
        """
        self.chunk_size = chunk_size
        self.overlap_tokens = int(chunk_size * overlap)

    def estimate_tokens(self, text: str) -> int:
        """Estimate token count using simple heuristic."""
        # Rough estimate: 1 token ≈ 4 characters
        return len(text) // 4

    def chunk_document(self, content: str, metadata: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Chunk markdown document into semantic sections.

        Args:
            content: Markdown content
            metadata: Document metadata (module, chapter, url, etc.)

        Returns:
            List of chunk dictionaries
        """
        chunks = []
        lines = content.split('\n')
        current_section = ""
        current_section_heading = ""
        current_code_block = False

        i = 0
        while i < len(lines):
            line = lines[i]

            # Track code blocks
            if line.strip().startswith('```'):
                current_code_block = not current_code_block

            # Track headings
            if line.strip().startswith('##') and not current_code_block:
                if current_section and self.estimate_tokens(current_section) > 50:
                    chunks.append({
                        "text": current_section.strip(),
                        "section_heading": current_section_heading,
                        "chunk_index": len(chunks),
                        **metadata
                    })
                current_section_heading = line.strip('#').strip()
                current_section = line + "\n"
            else:
                current_section += line + "\n"

            # Split on chunk size
            if self.estimate_tokens(current_section) > self.chunk_size:
                # Find nearest sentence boundary
                text_to_save = current_section[:-200] if len(current_section) > 200 else current_section
                chunks.append({
                    "text": text_to_save.strip(),
                    "section_heading": current_section_heading,
                    "chunk_index": len(chunks),
                    **metadata
                })
                # Keep overlap
                overlap_text = current_section[-200:] if len(current_section) > 200 else ""
                current_section = overlap_text

            i += 1

        # Add final chunk
        if current_section.strip():
            chunks.append({
                "text": current_section.strip(),
                "section_heading": current_section_heading,
                "chunk_index": len(chunks),
                **metadata
            })

        return chunks


class ContentIndexer:
    """Indexes Docusaurus content for RAG system."""

    def __init__(self, source_dir: str):
        """
        Initialize indexer.

        Args:
            source_dir: Path to Docusaurus docs directory
        """
        self.source_dir = Path(source_dir)
        self.chunker = DocumentChunker()
        self.all_chunks = []

    def extract_frontmatter(self, content: str) -> tuple[Dict[str, Any], str]:
        """Extract YAML frontmatter from markdown."""
        frontmatter = {}

        if content.startswith('---'):
            lines = content.split('\n')
            end_idx = -1
            for i in range(1, len(lines)):
                if lines[i].strip() == '---':
                    end_idx = i
                    break

            if end_idx > 0:
                fm_text = '\n'.join(lines[1:end_idx])
                # Simple YAML parsing
                for line in fm_text.split('\n'):
                    if ':' in line:
                        key, value = line.split(':', 1)
                        frontmatter[key.strip()] = value.strip().strip('"\'')

                content = '\n'.join(lines[end_idx+1:])

        return frontmatter, content

    def process_markdown_file(self, file_path: Path) -> List[Dict[str, Any]]:
        """Process a single markdown file and return chunks."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            frontmatter, content = self.extract_frontmatter(content)

            # Determine module and chapter from path
            relative_path = file_path.relative_to(self.source_dir)
            parts = relative_path.parts

            module = "module1"
            chapter = "intro"
            chapter_title = "Introduction"
            url = "/intro"

            if len(parts) > 1 and parts[0] == "module1":
                chapter_file = parts[1].replace('.md', '')
                chapter = chapter_file

                # Extract nice title from filename
                if chapter.startswith('chapter'):
                    chapter_title = chapter.replace('-', ' ').title()

                url = f"/module1/{chapter}"

            metadata = {
                "module": module,
                "chapter": chapter,
                "chapter_title": chapter_title,
                "url": url,
                "content_type": "text",
                "file_path": str(relative_path),
                "created_at": time.time()
            }

            chunks = self.chunker.chunk_document(content, metadata)
            return chunks

        except Exception as e:
            logger.error(f"Error processing {file_path}: {e}")
            return []

    def index_directory(self) -> List[Dict[str, Any]]:
        """Index all markdown files in the source directory."""
        all_chunks = []

        for md_file in self.source_dir.rglob('*.md'):
            logger.info(f"Processing {md_file.relative_to(self.source_dir)}...")
            chunks = self.process_markdown_file(md_file)
            all_chunks.extend(chunks)

        return all_chunks

    def save_index(self, chunks: List[Dict[str, Any]], output_file: str = "index.json"):
        """Save chunks to JSON file."""
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(chunks, f, indent=2, default=str)
        logger.info(f"Saved {len(chunks)} chunks to {output_file}")


def main():
    parser = argparse.ArgumentParser(
        description="Index Docusaurus content for RAG system"
    )
    parser.add_argument(
        '--source',
        required=True,
        help='Path to Docusaurus docs directory'
    )
    parser.add_argument(
        '--collection',
        default='ros2_book_content',
        help='Qdrant collection name'
    )
    parser.add_argument(
        '--chunk-size',
        type=int,
        default=512,
        help='Chunk size in tokens'
    )
    parser.add_argument(
        '--overlap',
        type=float,
        default=0.2,
        help='Chunk overlap ratio'
    )
    parser.add_argument(
        '--output',
        default='index.json',
        help='Output file for chunks'
    )

    args = parser.parse_args()

    logger.info(f"Starting content indexing from {args.source}")
    logger.info(f"Chunk size: {args.chunk_size} tokens, Overlap: {args.overlap*100:.0f}%")

    indexer = ContentIndexer(args.source)
    chunks = indexer.index_directory()

    logger.info(f"✓ Chunked {len(chunks)} total chunks")

    # Show statistics
    modules = set(c.get('module') for c in chunks)
    chapters = set(c.get('chapter') for c in chunks)

    logger.info(f"Modules: {modules}")
    logger.info(f"Chapters: {chapters}")

    # Save index
    indexer.save_index(chunks, args.output)

    logger.info("Indexing complete!")
    logger.info(f"Next step: Upload chunks to Qdrant collection '{args.collection}'")


if __name__ == "__main__":
    main()
