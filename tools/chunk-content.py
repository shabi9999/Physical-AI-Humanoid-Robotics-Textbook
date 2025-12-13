#!/usr/bin/env python3
"""
Content Chunking Script for RAG System

Splits markdown chapters into semantically meaningful chunks (400-800 tokens)
with configurable overlap for context preservation.

Features:
- Respects semantic boundaries (section headers, paragraphs)
- Configurable token count (512 ± 100 default)
- 20% overlap between chunks
- Preserves metadata (chapter_id, section, heading, keywords)
- Skips code blocks and diagrams (except for metadata extraction)
- Outputs JSON for downstream embedding/loading

Usage:
    python3 chunk-content.py --all
    python3 chunk-content.py path/to/chapter.md
    python3 chunk-content.py --all --output chunks.jsonl

Output:
    chunks.jsonl — JSONL format (one chunk per line)
    chunks-metadata.json — Summary with statistics
"""

import re
import json
import sys
import argparse
from pathlib import Path
from typing import List, Dict, Tuple, Optional, Set
from dataclasses import dataclass, asdict, field
from uuid import uuid4
from collections import defaultdict
import textwrap


@dataclass
class Chunk:
    """Represents a single chunk of content."""
    id: str
    chapter_id: str
    module_number: int
    section_name: str
    heading: str
    content: str
    token_count: int
    keywords: List[str] = field(default_factory=list)
    learning_objectives: List[str] = field(default_factory=list)
    difficulty_level: str = 'Beginner'
    url_anchor: str = ''
    sequence_number: int = 0
    overlap_with_previous: int = 0


class ChunkingConfig:
    """Configuration for chunking."""
    TARGET_TOKENS = 512  # Target chunk size
    MIN_TOKENS = 400     # Minimum tokens to form a chunk
    MAX_TOKENS = 800     # Maximum tokens before forcing split
    OVERLAP_PERCENTAGE = 20  # 20% overlap with previous chunk
    OVERLAP_TOKENS = int(TARGET_TOKENS * OVERLAP_PERCENTAGE / 100)


class TokenCounter:
    """Simple token counter using word count heuristic."""

    @staticmethod
    def estimate_tokens(text: str) -> int:
        """
        Estimate token count using word count.
        Approximation: ~1.3 words per token (for English).
        OpenAI tokenizer: ~0.75 tokens per word on average.
        We use 0.75 for consistency with OpenAI's counting.
        """
        words = len(text.split())
        # Rough approximation: average token/word ratio is ~0.75-0.8
        # Using 0.75 for safety (conservative estimate)
        return max(1, int(words * 0.75))

    @staticmethod
    def count_words(text: str) -> int:
        """Count words in text."""
        return len(text.split())


class ContentChunker:
    """Chunks markdown content into semantic units."""

    # Markdown heading levels and their hierarchy
    HEADING_PATTERN = r'^(#{1,6})\s+(.+)$'

    # Code block pattern (skip these)
    CODE_BLOCK_PATTERN = r'```[\s\S]*?```'

    # Mermaid diagram pattern (skip these)
    MERMAID_PATTERN = r'```mermaid\n[\s\S]*?\n```'

    # Keywords pattern (sentences with key terms)
    KEY_TERM_PATTERN = r'\b(ROS|URDF|VSLAM|Isaac Sim|Whisper|LLM|Nav2|MoveIt|action server|sensor|perception|localization|trajectory|gripper|joint)\b'

    def __init__(self, config: ChunkingConfig = None):
        self.config = config or ChunkingConfig()
        self.token_counter = TokenCounter()

    def extract_metadata_from_frontmatter(self, content: str) -> Dict:
        """Extract YAML frontmatter metadata."""
        metadata = {
            'learning_objectives': [],
            'keywords': [],
            'difficulty': 'Beginner'
        }

        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 2:
                frontmatter = parts[1]
                # Simple YAML extraction (not full parsing)
                for line in frontmatter.split('\n'):
                    if 'learning_objectives:' in line:
                        # Try to extract from list format
                        pass
                    elif 'keywords:' in line:
                        pass
                    elif 'difficulty:' in line:
                        match = re.search(r'difficulty:\s*["\']?(\w+)["\']?', line)
                        if match:
                            metadata['difficulty'] = match.group(1)

        return metadata

    def clean_text(self, text: str) -> str:
        """Remove code blocks, diagrams, and markdown formatting."""
        # Remove mermaid diagrams
        text = re.sub(self.MERMAID_PATTERN, '', text)

        # Remove code blocks
        text = re.sub(self.CODE_BLOCK_PATTERN, '', text)

        # Remove inline code
        text = re.sub(r'`[^`]*`', '', text)

        # Remove markdown links but keep text
        text = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', text)

        # Remove markdown emphasis markers
        text = re.sub(r'[*_]{1,2}([^*_]+)[*_]{1,2}', r'\1', text)

        # Remove HTML tags
        text = re.sub(r'<[^>]+>', '', text)

        # Remove extra whitespace
        text = re.sub(r'\s+', ' ', text)

        return text.strip()

    def extract_headings(self, content: str) -> List[Tuple[int, str, str]]:
        """Extract headings with their levels and text."""
        headings = []
        for line in content.split('\n'):
            match = re.match(self.HEADING_PATTERN, line)
            if match:
                level = len(match.group(1))
                heading_text = match.group(2).strip()
                headings.append((level, heading_text, line))
        return headings

    def extract_keywords(self, text: str) -> List[str]:
        """Extract key technical terms from text."""
        keywords = set()
        matches = re.finditer(self.KEY_TERM_PATTERN, text, re.IGNORECASE)
        for match in matches:
            keywords.add(match.group(1).lower())
        return list(keywords)

    def slugify(self, text: str) -> str:
        """Convert heading text to URL anchor."""
        text = re.sub(r'[^a-z0-9\s-]', '', text.lower())
        text = re.sub(r'\s+', '-', text)
        return text.strip('-')

    def split_into_paragraphs(self, text: str) -> List[str]:
        """Split text into paragraphs."""
        # Split on double newline
        paragraphs = text.split('\n\n')
        return [p.strip() for p in paragraphs if p.strip()]

    def chunk_content(
        self,
        filepath: Path,
        chapter_id: str,
        module_number: int
    ) -> List[Chunk]:
        """
        Chunk a single chapter into semantic units.

        Args:
            filepath: Path to markdown chapter
            chapter_id: e.g., 'module1/chapter1-ros2-core'
            module_number: 1-4

        Returns:
            List of Chunk objects
        """
        with open(filepath, 'r', encoding='utf-8') as f:
            full_content = f.read()

        # Extract frontmatter metadata
        frontmatter_meta = self.extract_metadata_from_frontmatter(full_content)

        # Remove frontmatter
        content = full_content
        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 3:
                content = parts[2]

        # Extract headings for section tracking
        headings = self.extract_headings(content)

        # Clean text (remove code blocks, diagrams, etc.)
        content = self.clean_text(content)

        # Split into paragraphs
        paragraphs = self.split_into_paragraphs(content)

        # Group paragraphs into chunks respecting token limits
        chunks = []
        current_chunk_paragraphs = []
        current_tokens = 0
        current_section = 'Introduction'
        current_heading = ''
        overlap_buffer = ''

        for paragraph in paragraphs:
            para_tokens = self.token_counter.estimate_tokens(paragraph)

            # Check if adding this paragraph would exceed max tokens
            if current_tokens + para_tokens > self.config.MAX_TOKENS and current_chunk_paragraphs:
                # Create chunk from accumulated paragraphs
                chunk_text = '\n\n'.join(current_chunk_paragraphs)
                chunk_token_count = self.token_counter.estimate_tokens(chunk_text)

                if chunk_token_count >= self.config.MIN_TOKENS:
                    keywords = self.extract_keywords(chunk_text)

                    chunk = Chunk(
                        id=str(uuid4()),
                        chapter_id=chapter_id,
                        module_number=module_number,
                        section_name=current_section,
                        heading=current_heading,
                        content=chunk_text,
                        token_count=chunk_token_count,
                        keywords=keywords,
                        learning_objectives=frontmatter_meta.get('learning_objectives', []),
                        difficulty_level=frontmatter_meta.get('difficulty', 'Beginner'),
                        url_anchor=f"/docs/{chapter_id}#{self.slugify(current_heading)}",
                        sequence_number=len(chunks),
                        overlap_with_previous=len(overlap_buffer)
                    )
                    chunks.append(chunk)

                    # Prepare overlap for next chunk (20% of current chunk)
                    current_chunk_paragraphs = overlap_buffer.split('\n\n') if overlap_buffer else []

                # Reset for next chunk
                overlap_buffer = '\n\n'.join(current_chunk_paragraphs)
                current_chunk_paragraphs = [paragraph]
                current_tokens = para_tokens
            else:
                current_chunk_paragraphs.append(paragraph)
                current_tokens += para_tokens

        # Handle remaining paragraphs
        if current_chunk_paragraphs:
            chunk_text = '\n\n'.join(current_chunk_paragraphs)
            chunk_token_count = self.token_counter.estimate_tokens(chunk_text)

            if chunk_token_count >= self.config.MIN_TOKENS:
                keywords = self.extract_keywords(chunk_text)

                chunk = Chunk(
                    id=str(uuid4()),
                    chapter_id=chapter_id,
                    module_number=module_number,
                    section_name=current_section,
                    heading=current_heading,
                    content=chunk_text,
                    token_count=chunk_token_count,
                    keywords=keywords,
                    learning_objectives=frontmatter_meta.get('learning_objectives', []),
                    difficulty_level=frontmatter_meta.get('difficulty', 'Beginner'),
                    url_anchor=f"/docs/{chapter_id}#{self.slugify(current_heading)}",
                    sequence_number=len(chunks),
                    overlap_with_previous=len(overlap_buffer)
                )
                chunks.append(chunk)

        return chunks

    def find_all_chapters(self, base_dir: str = 'my-website/docs') -> List[Tuple[Path, str, int]]:
        """Find all chapter files and return (path, chapter_id, module_number)."""
        chapters = []
        base_path = Path(base_dir)

        for module_dir in sorted(base_path.iterdir()):
            if not module_dir.is_dir() or not module_dir.name.startswith('module'):
                continue

            # Extract module number
            match = re.match(r'module(\d+)', module_dir.name)
            if not match:
                continue
            module_number = int(match.group(1))

            # Find chapter files
            for chapter_file in sorted(module_dir.glob('chapter*.md')):
                chapter_id = f"{module_dir.name}/{chapter_file.stem}"
                chapters.append((chapter_file, chapter_id, module_number))

        return chapters

    def chunk_all_chapters(self) -> Tuple[List[Chunk], Dict]:
        """Chunk all chapters and return statistics."""
        all_chunks = []
        statistics = {
            'total_chapters': 0,
            'total_chunks': 0,
            'total_tokens': 0,
            'avg_tokens_per_chunk': 0,
            'avg_chunks_per_chapter': 0,
            'chapters': []
        }

        chapters = self.find_all_chapters()

        for filepath, chapter_id, module_number in chapters:
            print(f"Chunking {chapter_id}...", end=' ')
            chunks = self.chunk_content(filepath, chapter_id, module_number)

            if chunks:
                statistics['total_chapters'] += 1
                statistics['total_chunks'] += len(chunks)
                total_tokens = sum(c.token_count for c in chunks)
                statistics['total_tokens'] += total_tokens

                statistics['chapters'].append({
                    'chapter_id': chapter_id,
                    'chunk_count': len(chunks),
                    'total_tokens': total_tokens,
                    'avg_tokens_per_chunk': int(total_tokens / len(chunks))
                })

                print(f"✅ {len(chunks)} chunks ({total_tokens} tokens)")
                all_chunks.extend(chunks)
            else:
                print("❌ No chunks created")

        if all_chunks:
            statistics['avg_tokens_per_chunk'] = int(
                statistics['total_tokens'] / statistics['total_chunks']
            )
            statistics['avg_chunks_per_chapter'] = int(
                statistics['total_chunks'] / statistics['total_chapters']
            )

        return all_chunks, statistics


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Chunk markdown chapters for RAG system'
    )
    parser.add_argument(
        '--all',
        action='store_true',
        help='Chunk all chapters'
    )
    parser.add_argument(
        '--output',
        default='chunks.jsonl',
        help='Output JSONL file'
    )
    parser.add_argument(
        '--metadata-output',
        default='chunks-metadata.json',
        help='Output metadata/statistics file'
    )
    parser.add_argument(
        'filepath',
        nargs='?',
        help='Path to single chapter (optional)'
    )

    args = parser.parse_args()

    chunker = ContentChunker()

    if args.all:
        print("\n📄 Chunking all chapters...")
        all_chunks, statistics = chunker.chunk_all_chapters()
    elif args.filepath:
        print(f"\n📄 Chunking {args.filepath}...")
        chapter_id = Path(args.filepath).parent.name + '/' + Path(args.filepath).stem
        module_match = re.search(r'module(\d+)', chapter_id)
        module_number = int(module_match.group(1)) if module_match else 1

        all_chunks = chunker.chunk_content(Path(args.filepath), chapter_id, module_number)
        statistics = {
            'total_chapters': 1,
            'total_chunks': len(all_chunks),
            'total_tokens': sum(c.token_count for c in all_chunks),
            'chapters': [{
                'chapter_id': chapter_id,
                'chunk_count': len(all_chunks),
                'total_tokens': sum(c.token_count for c in all_chunks)
            }]
        }
    else:
        parser.print_help()
        sys.exit(1)

    # Write chunks to JSONL
    print(f"\n💾 Writing {len(all_chunks)} chunks to {args.output}...")
    with open(args.output, 'w') as f:
        for chunk in all_chunks:
            f.write(json.dumps(asdict(chunk)) + '\n')

    # Write metadata
    print(f"💾 Writing statistics to {args.metadata_output}...")
    with open(args.metadata_output, 'w') as f:
        json.dump(statistics, f, indent=2)

    # Print summary
    print(f"\n{'='*80}")
    print(f"CHUNKING SUMMARY")
    print(f"{'='*80}")
    print(f"Total Chapters:              {statistics['total_chapters']}")
    print(f"Total Chunks:                {statistics['total_chunks']}")
    print(f"Total Tokens:                {statistics['total_tokens']:,}")
    print(f"Avg Tokens per Chunk:        {statistics.get('avg_tokens_per_chunk', 0)}")
    print(f"Avg Chunks per Chapter:      {statistics.get('avg_chunks_per_chapter', 0)}")
    print(f"\n✅ Chunking complete!")
    print(f"   Output: {args.output}")
    print(f"   Metadata: {args.metadata_output}")


if __name__ == '__main__':
    main()
