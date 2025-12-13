#!/usr/bin/env python3
"""
Readability Checker for Humanoid Robotics Textbook

Analyzes markdown chapters for:
- Flesch-Kincaid Grade Level
- Sentence and paragraph length distribution
- Jargon density (acronyms, technical terms)
- Reading time estimates

Usage:
    python3 readability-check.py
    python3 readability-check.py path/to/chapter.md
    python3 readability-check.py --all

Output:
    - Individual chapter reports (JSON or text)
    - Summary report across all chapters
    - Flags chapters >12 FK grade level for editing
"""

import re
import os
import json
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from collections import defaultdict


@dataclass
class ReadabilityMetrics:
    """Metrics for a single chapter."""
    filepath: str
    chapter_id: str
    word_count: int
    sentence_count: int
    syllable_count: int
    paragraph_count: int
    acronym_count: int
    technical_terms_count: int
    avg_sentence_length: float
    avg_paragraph_length: float
    flesch_kincaid_grade: float
    estimated_reading_time_minutes: float
    jargon_density: float
    warnings: List[str]


class ReadabilityAnalyzer:
    """Analyzes readability metrics for markdown chapters."""

    # Common technical terms for robotics textbook
    TECHNICAL_TERMS = {
        'ROS', 'URDF', 'VSLAM', 'Nav2', 'Isaac Sim', 'Whisper', 'LLM', 'API',
        'node', 'topic', 'service', 'action', 'message', 'callback', 'publisher',
        'subscriber', 'kinematic', 'trajectory', 'gripper', 'actuator', 'sensor',
        'localization', 'mapping', 'perception', 'motion planning', 'collision detection',
        'frame', 'transform', 'coordinate', 'embedding', 'vector', 'matrix',
        'optimization', 'algorithm', 'neural network', 'machine learning', 'inference',
        'semantic', 'synthetic data', 'domain randomization', 'sim-to-real', 'depth sensor',
        'IMU', 'LiDAR', 'camera', 'joint', 'link', 'DOF', 'IK', 'FK',
        'MoveIt', 'Gazebo', 'CoppeliaSim', 'Omniverse', 'photorealistic', 'physics engine'
    }

    # Common acronyms (case-insensitive matching)
    COMMON_ACRONYMS = {
        'ROS', 'URDF', 'VSLAM', 'NAV2', 'IMU', 'LIDAR', 'API', 'HTTP', 'JSON',
        'SQL', 'GPU', 'CPU', 'RAM', 'DOF', 'IK', 'FK', 'PID', 'UI', 'UX',
        'LLM', 'AI', 'ML', 'NLP', 'CV', 'DL', 'CNN', 'RNN', 'SLAM', 'GPS',
        'USB', 'TCP', 'UDP', 'SSH', 'TLS', 'SSL', 'JWT', 'IDE', 'CLI', 'CD',
        'CI', 'PR', 'AWS', 'GCP', 'SDK', 'API', 'REST', 'YAML', 'XML', 'FPS'
    }

    def __init__(self):
        self.markdown_dir = Path('my-website/docs')

    def count_syllables(self, word: str) -> int:
        """Estimate syllable count using simple heuristic."""
        word = word.lower()
        vowels = 'aeiouy'
        syllable_count = 0
        previous_was_vowel = False

        for char in word:
            is_vowel = char in vowels
            if is_vowel and not previous_was_vowel:
                syllable_count += 1
            previous_was_vowel = is_vowel

        # Adjustments
        if word.endswith('e'):
            syllable_count -= 1
        if word.endswith('le') and len(word) > 2 and word[-3] not in vowels:
            syllable_count += 1

        return max(1, syllable_count)

    def extract_text_from_markdown(self, content: str) -> str:
        """Extract readable text from markdown, removing code blocks and frontmatter."""
        # Remove YAML frontmatter
        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 3:
                content = parts[2]

        # Remove code blocks
        content = re.sub(r'```[\s\S]*?```', '', content)

        # Remove inline code
        content = re.sub(r'`[^`]*`', '', content)

        # Remove markdown links but keep text
        content = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', content)

        # Remove markdown emphasis markers
        content = re.sub(r'[*_]{1,2}([^*_]+)[*_]{1,2}', r'\1', content)

        # Remove headings but keep text
        content = re.sub(r'^#+\s+', '', content, flags=re.MULTILINE)

        # Remove HTML tags
        content = re.sub(r'<[^>]+>', '', content)

        # Remove mermaid diagrams
        content = re.sub(r'```mermaid[\s\S]*?```', '', content)

        return content

    def count_acronyms(self, text: str) -> int:
        """Count occurrences of technical acronyms."""
        count = 0
        for acronym in self.COMMON_ACRONYMS:
            # Word boundary matching to avoid partial matches
            pattern = r'\b' + re.escape(acronym) + r'\b'
            count += len(re.findall(pattern, text, re.IGNORECASE))
        return count

    def count_technical_terms(self, text: str) -> int:
        """Count occurrences of technical terms."""
        count = 0
        for term in self.TECHNICAL_TERMS:
            pattern = r'\b' + re.escape(term) + r'\b'
            count += len(re.findall(pattern, text, re.IGNORECASE))
        return count

    def analyze_chapter(self, filepath: Path) -> ReadabilityMetrics:
        """Analyze a single chapter for readability metrics."""
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract chapter ID from path
        chapter_id = '/'.join(filepath.parts[-2:]).replace('.md', '')

        # Extract readable text
        text = self.extract_text_from_markdown(content)

        # Count words
        words = text.split()
        word_count = len(words)

        # Count sentences (split on . ! ?)
        sentences = re.split(r'[.!?]+', text)
        sentences = [s.strip() for s in sentences if s.strip()]
        sentence_count = len(sentences)

        # Count syllables
        syllable_count = sum(self.count_syllables(word) for word in words)

        # Count paragraphs
        paragraphs = [p.strip() for p in text.split('\n\n') if p.strip()]
        paragraph_count = len(paragraphs)

        # Count jargon
        acronym_count = self.count_acronyms(text)
        technical_terms_count = self.count_technical_terms(text)

        # Calculate metrics
        avg_sentence_length = word_count / max(sentence_count, 1)
        avg_paragraph_length = word_count / max(paragraph_count, 1)
        jargon_density = (acronym_count + technical_terms_count) / max(word_count, 1)

        # Flesch-Kincaid Grade Level
        # FK = (0.39 * words/sentences) + (11.8 * syllables/words) - 15.59
        fk_grade = (
            (0.39 * avg_sentence_length) +
            (11.8 * (syllable_count / max(word_count, 1))) -
            15.59
        )
        fk_grade = max(0, fk_grade)  # No negative grades

        # Estimated reading time (average 200 words per minute)
        estimated_reading_time = word_count / 200

        # Generate warnings
        warnings = []
        if fk_grade > 12:
            warnings.append(f"HIGH GRADE LEVEL: FK={fk_grade:.1f} (target: ≤12)")
        if avg_sentence_length > 25:
            warnings.append(f"LONG SENTENCES: avg={avg_sentence_length:.1f} words (target: ≤25)")
        if avg_paragraph_length > 150:
            warnings.append(f"LONG PARAGRAPHS: avg={avg_paragraph_length:.1f} words (target: ≤150)")
        if jargon_density > 0.15:
            warnings.append(f"HIGH JARGON: {jargon_density*100:.1f}% (target: ≤15%)")
        if word_count < 2000:
            warnings.append(f"SHORT CHAPTER: {word_count} words (target: 2000+)")
        if word_count > 8000:
            warnings.append(f"LONG CHAPTER: {word_count} words (target: ≤8000)")

        return ReadabilityMetrics(
            filepath=str(filepath),
            chapter_id=chapter_id,
            word_count=word_count,
            sentence_count=sentence_count,
            syllable_count=syllable_count,
            paragraph_count=paragraph_count,
            acronym_count=acronym_count,
            technical_terms_count=technical_terms_count,
            avg_sentence_length=avg_sentence_length,
            avg_paragraph_length=avg_paragraph_length,
            flesch_kincaid_grade=fk_grade,
            estimated_reading_time_minutes=estimated_reading_time,
            jargon_density=jargon_density,
            warnings=warnings
        )

    def find_all_chapters(self) -> List[Path]:
        """Find all chapter markdown files."""
        chapters = []
        for module_dir in sorted(self.markdown_dir.iterdir()):
            if module_dir.is_dir() and module_dir.name.startswith('module'):
                for file in sorted(module_dir.glob('chapter*.md')):
                    chapters.append(file)
        return chapters

    def print_report(self, metrics: ReadabilityMetrics, verbose: bool = False):
        """Print a readable report for a single chapter."""
        print(f"\n{'='*80}")
        print(f"CHAPTER: {metrics.chapter_id}")
        print(f"{'='*80}")
        print(f"Word Count:              {metrics.word_count:>6} words")
        print(f"Sentences:               {metrics.sentence_count:>6} (avg {metrics.avg_sentence_length:.1f} words)")
        print(f"Paragraphs:              {metrics.paragraph_count:>6} (avg {metrics.avg_paragraph_length:.1f} words)")
        print(f"Flesch-Kincaid Grade:    {metrics.flesch_kincaid_grade:>6.1f} (target: ≤12)")
        print(f"Reading Time:            {metrics.estimated_reading_time_minutes:>6.1f} minutes")
        print(f"Acronyms:                {metrics.acronym_count:>6} instances")
        print(f"Technical Terms:         {metrics.technical_terms_count:>6} instances")
        print(f"Jargon Density:          {metrics.jargon_density*100:>6.1f}% (target: ≤15%)")

        if metrics.warnings:
            print(f"\n⚠️  WARNINGS:")
            for warning in metrics.warnings:
                print(f"    • {warning}")
        else:
            print(f"\n✅ NO ISSUES DETECTED")

    def generate_json_report(self, all_metrics: List[ReadabilityMetrics]) -> str:
        """Generate JSON report for CI/CD integration."""
        data = {
            'summary': {
                'total_chapters': len(all_metrics),
                'chapters_with_issues': len([m for m in all_metrics if m.warnings]),
                'avg_flesch_kincaid': sum(m.flesch_kincaid_grade for m in all_metrics) / len(all_metrics) if all_metrics else 0,
                'avg_reading_time_minutes': sum(m.estimated_reading_time_minutes for m in all_metrics) if all_metrics else 0,
                'total_word_count': sum(m.word_count for m in all_metrics),
            },
            'chapters': [
                {
                    'chapter_id': m.chapter_id,
                    'word_count': m.word_count,
                    'flesch_kincaid_grade': round(m.flesch_kincaid_grade, 2),
                    'reading_time_minutes': round(m.estimated_reading_time_minutes, 1),
                    'avg_sentence_length': round(m.avg_sentence_length, 1),
                    'avg_paragraph_length': round(m.avg_paragraph_length, 1),
                    'jargon_density': round(m.jargon_density, 3),
                    'warnings': m.warnings
                }
                for m in all_metrics
            ]
        }
        return json.dumps(data, indent=2)


def main():
    """Main entry point."""
    analyzer = ReadabilityAnalyzer()

    # Find chapters to analyze
    if len(sys.argv) > 1 and sys.argv[1] == '--all':
        chapters = analyzer.find_all_chapters()
    elif len(sys.argv) > 1:
        chapters = [Path(sys.argv[1])]
    else:
        chapters = analyzer.find_all_chapters()

    if not chapters:
        print("❌ No chapters found")
        sys.exit(1)

    print(f"\n📖 Analyzing {len(chapters)} chapters for readability...")

    # Analyze all chapters
    all_metrics = []
    for chapter_path in chapters:
        if chapter_path.exists():
            metrics = analyzer.analyze_chapter(chapter_path)
            all_metrics.append(metrics)
            analyzer.print_report(metrics)
        else:
            print(f"❌ File not found: {chapter_path}")

    # Print summary
    print(f"\n{'='*80}")
    print(f"SUMMARY ACROSS ALL CHAPTERS")
    print(f"{'='*80}")
    total_chapters = len(all_metrics)
    chapters_with_warnings = len([m for m in all_metrics if m.warnings])
    avg_fk = sum(m.flesch_kincaid_grade for m in all_metrics) / total_chapters if all_metrics else 0
    total_words = sum(m.word_count for m in all_metrics)
    total_reading_time = sum(m.estimated_reading_time_minutes for m in all_metrics)

    print(f"Total Chapters:          {total_chapters}")
    print(f"Chapters with Issues:    {chapters_with_warnings} ({chapters_with_warnings*100//total_chapters}%)")
    print(f"Average FK Grade Level:  {avg_fk:.1f}")
    print(f"Total Word Count:        {total_words:,} words")
    print(f"Total Reading Time:      {total_reading_time:.1f} minutes")

    # Generate JSON report
    json_report = analyzer.generate_json_report(all_metrics)
    with open('readability-report.json', 'w') as f:
        f.write(json_report)
    print(f"\n✅ JSON report saved to readability-report.json")

    # Exit with error if critical issues found
    critical_issues = [m for m in all_metrics if any('HIGH GRADE LEVEL' in w for w in m.warnings)]
    if critical_issues:
        print(f"\n❌ {len(critical_issues)} chapter(s) with HIGH GRADE LEVEL detected")
        sys.exit(1)

    print(f"\n✅ Readability check complete")


if __name__ == '__main__':
    main()
