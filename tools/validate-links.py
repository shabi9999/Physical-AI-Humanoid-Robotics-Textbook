#!/usr/bin/env python3
"""
Link Validator for Humanoid Robotics Textbook

Validates:
- Internal MDX cross-references ([text](/docs/module1/chapter1#anchor))
- External URLs (documentation links, references)
- Anchor existence (headings must be convertible to anchors)
- Relative path correctness

Usage:
    python3 validate-links.py
    python3 validate-links.py path/to/chapter.md
    python3 validate-links.py --all

Output:
    - Links by status (valid, broken, external)
    - Anchor resolution report
    - Summary with exit code
"""

import re
import os
import json
import sys
import requests
from pathlib import Path
from typing import Dict, List, Set, Tuple, Optional
from dataclasses import dataclass, field
from collections import defaultdict
from urllib.parse import urlparse


@dataclass
class LinkValidationResult:
    """Result for a single link validation."""
    filepath: str
    link_type: str  # 'internal', 'external', 'anchor'
    url: str
    display_text: str
    line_number: int
    status: str  # 'valid', 'broken', 'warning', 'external'
    message: str = ''


@dataclass
class ChapterValidation:
    """Validation results for a single chapter."""
    chapter_id: str
    filepath: str
    total_links: int
    valid_links: int
    broken_links: int
    external_links: int
    warnings: int
    link_results: List[LinkValidationResult] = field(default_factory=list)


class LinkValidator:
    """Validates internal and external links in markdown chapters."""

    # Pattern for markdown links [text](url)
    MARKDOWN_LINK_PATTERN = r'\[([^\]]+)\]\(([^)]+)\)'

    # Pattern for markdown headings
    HEADING_PATTERN = r'^#{1,6}\s+(.+)$'

    def __init__(self, base_dir: str = 'my-website/docs'):
        self.base_dir = Path(base_dir)
        self.chapter_headings: Dict[str, Set[str]] = {}  # chapter -> set of anchors
        self.external_urls_cache: Dict[str, bool] = {}  # url -> is_valid
        self._build_anchor_cache()

    def _slugify(self, text: str) -> str:
        """Convert heading text to URL anchor."""
        # Remove markdown formatting
        text = re.sub(r'[*_`]', '', text)
        # Remove special characters, keep alphanumeric and hyphens
        text = re.sub(r'[^a-z0-9\s-]', '', text.lower())
        # Replace whitespace with hyphens
        text = re.sub(r'\s+', '-', text)
        # Remove leading/trailing hyphens
        text = text.strip('-')
        return text

    def _build_anchor_cache(self):
        """Build cache of all heading anchors in all chapters."""
        for module_dir in sorted(self.base_dir.iterdir()):
            if not module_dir.is_dir():
                continue

            for chapter_file in sorted(module_dir.glob('*.md')):
                chapter_id = f"{module_dir.name}/{chapter_file.stem}"
                anchors: Set[str] = set()

                with open(chapter_file, 'r', encoding='utf-8') as f:
                    for line in f:
                        match = re.match(self.HEADING_PATTERN, line)
                        if match:
                            heading_text = match.group(1)
                            anchor = self._slugify(heading_text)
                            anchors.add(anchor)

                self.chapter_headings[chapter_id] = anchors

    def extract_links(self, content: str) -> List[Tuple[str, str, int]]:
        """Extract all markdown links from content."""
        links = []
        for line_num, line in enumerate(content.split('\n'), 1):
            for match in re.finditer(self.MARKDOWN_LINK_PATTERN, line):
                display_text = match.group(1)
                url = match.group(2)
                links.append((display_text, url, line_num))
        return links

    def validate_internal_link(self, url: str) -> Tuple[str, str]:
        """Validate internal link (/docs/module1/chapter1#anchor)."""
        # Parse URL
        if not url.startswith('/docs/'):
            return 'invalid', 'Internal links must start with /docs/'

        # Split by anchor
        parts = url.split('#')
        path_part = parts[0]
        anchor_part = parts[1] if len(parts) > 1 else None

        # Validate path format: /docs/module{N}/chapter{name}
        path_match = re.match(r'^/docs/(module\d+)/(chapter[\w-]+)$', path_part)
        if not path_match:
            return 'broken', f'Invalid path format: {path_part}'

        module = path_match.group(1)
        chapter = path_match.group(2)
        chapter_id = f"{module}/{chapter}"

        # Check if chapter exists
        chapter_file = self.base_dir / module / f"{chapter}.md"
        if not chapter_file.exists():
            return 'broken', f'Chapter file not found: {chapter_file}'

        # If anchor is specified, validate it exists
        if anchor_part:
            valid_anchors = self.chapter_headings.get(chapter_id, set())
            if anchor_part not in valid_anchors:
                return 'warning', f'Anchor #{anchor_part} not found in {chapter_id}'

        return 'valid', 'Link is valid'

    def validate_external_url(self, url: str) -> Tuple[str, str]:
        """Validate external URL (with caching)."""
        # Don't validate URLs that aren't http(s)
        if not url.startswith(('http://', 'https://')):
            return 'warning', 'Non-http URL (not validated)'

        # Check cache
        if url in self.external_urls_cache:
            is_valid = self.external_urls_cache[url]
            return ('valid' if is_valid else 'broken', 'From cache')

        # Try to fetch (with timeout)
        try:
            response = requests.head(url, timeout=5, allow_redirects=True)
            is_valid = response.status_code < 400
            self.external_urls_cache[url] = is_valid
            return ('valid' if is_valid else 'broken', f'HTTP {response.status_code}')
        except requests.exceptions.Timeout:
            return 'warning', 'Request timeout'
        except requests.exceptions.ConnectionError:
            return 'warning', 'Connection error'
        except Exception as e:
            return 'warning', str(e)

    def validate_link(self, url: str) -> Tuple[str, str]:
        """Determine link type and validate."""
        if url.startswith('/docs/'):
            return self.validate_internal_link(url)
        elif url.startswith(('http://', 'https://')):
            return self.validate_external_url(url)
        elif url.startswith('#'):
            # Anchor-only link (same page)
            return 'valid', 'Same-page anchor'
        else:
            return 'warning', f'Unknown URL type: {url}'

    def validate_chapter(self, filepath: Path) -> ChapterValidation:
        """Validate all links in a chapter."""
        chapter_id = f"{filepath.parent.name}/{filepath.stem}"

        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()

        links = self.extract_links(content)
        link_results = []

        valid_count = 0
        broken_count = 0
        external_count = 0
        warning_count = 0

        for display_text, url, line_num in links:
            # Determine link type
            if url.startswith(('http://', 'https://')):
                link_type = 'external'
                external_count += 1
            else:
                link_type = 'internal'

            # Validate
            status, message = self.validate_link(url)

            if status == 'valid':
                valid_count += 1
            elif status == 'broken':
                broken_count += 1
            elif status == 'warning':
                warning_count += 1

            # Skip reporting valid external links (too verbose)
            if not (link_type == 'external' and status == 'valid'):
                result = LinkValidationResult(
                    filepath=str(filepath),
                    link_type=link_type,
                    url=url,
                    display_text=display_text,
                    line_number=line_num,
                    status=status,
                    message=message
                )
                link_results.append(result)

        return ChapterValidation(
            chapter_id=chapter_id,
            filepath=str(filepath),
            total_links=len(links),
            valid_links=valid_count,
            broken_links=broken_count,
            external_links=external_count,
            warnings=warning_count,
            link_results=link_results
        )

    def find_all_chapters(self) -> List[Path]:
        """Find all chapter markdown files."""
        chapters = []
        for module_dir in sorted(self.base_dir.iterdir()):
            if module_dir.is_dir() and module_dir.name.startswith('module'):
                for file in sorted(module_dir.glob('chapter*.md')):
                    chapters.append(file)
        return chapters

    def print_report(self, validation: ChapterValidation):
        """Print validation report for a single chapter."""
        print(f"\n{'='*80}")
        print(f"CHAPTER: {validation.chapter_id}")
        print(f"{'='*80}")
        print(f"Total Links:         {validation.total_links}")
        print(f"Valid:               {validation.valid_links}")
        print(f"Broken:              {validation.broken_links}")
        print(f"Warnings:            {validation.warnings}")
        print(f"External:            {validation.external_links}")

        if validation.link_results:
            print(f"\nIssues Found:")
            for result in validation.link_results:
                status_icon = '❌' if result.status == 'broken' else '⚠️ '
                print(f"  {status_icon} Line {result.line_number}: [{result.display_text}]({result.url})")
                print(f"      Type: {result.link_type} | Status: {result.status}")
                print(f"      Message: {result.message}")
        else:
            print(f"\n✅ No issues found")

    def generate_json_report(self, all_validations: List[ChapterValidation]) -> str:
        """Generate JSON report for CI/CD integration."""
        total_chapters = len(all_validations)
        total_links = sum(v.total_links for v in all_validations)
        total_valid = sum(v.valid_links for v in all_validations)
        total_broken = sum(v.broken_links for v in all_validations)
        total_warnings = sum(v.warnings for v in all_validations)

        data = {
            'summary': {
                'total_chapters': total_chapters,
                'total_links': total_links,
                'valid_links': total_valid,
                'broken_links': total_broken,
                'warnings': total_warnings,
                'health': 'good' if total_broken == 0 else 'poor'
            },
            'chapters': [
                {
                    'chapter_id': v.chapter_id,
                    'total_links': v.total_links,
                    'valid': v.valid_links,
                    'broken': v.broken_links,
                    'warnings': v.warnings,
                    'external': v.external_links,
                    'issues': [
                        {
                            'line': r.line_number,
                            'type': r.link_type,
                            'url': r.url,
                            'text': r.display_text,
                            'status': r.status,
                            'message': r.message
                        }
                        for r in v.link_results
                    ]
                }
                for v in all_validations
            ]
        }
        return json.dumps(data, indent=2)


def main():
    """Main entry point."""
    validator = LinkValidator()

    # Find chapters to validate
    if len(sys.argv) > 1 and sys.argv[1] == '--all':
        chapters = validator.find_all_chapters()
    elif len(sys.argv) > 1:
        chapters = [Path(sys.argv[1])]
    else:
        chapters = validator.find_all_chapters()

    if not chapters:
        print("❌ No chapters found")
        sys.exit(1)

    print(f"\n🔗 Validating links in {len(chapters)} chapters...")

    # Validate all chapters
    all_validations = []
    for chapter_path in chapters:
        if chapter_path.exists():
            validation = validator.validate_chapter(chapter_path)
            all_validations.append(validation)
            validator.print_report(validation)
        else:
            print(f"❌ File not found: {chapter_path}")

    # Print summary
    print(f"\n{'='*80}")
    print(f"LINK VALIDATION SUMMARY")
    print(f"{'='*80}")
    total_chapters = len(all_validations)
    total_links = sum(v.total_links for v in all_validations)
    total_valid = sum(v.valid_links for v in all_validations)
    total_broken = sum(v.broken_links for v in all_validations)
    total_warnings = sum(v.warnings for v in all_validations)

    print(f"Total Chapters:      {total_chapters}")
    print(f"Total Links:         {total_links}")
    print(f"Valid:               {total_valid}")
    print(f"Broken:              {total_broken}")
    print(f"Warnings:            {total_warnings}")

    if total_broken > 0:
        print(f"\n❌ {total_broken} broken link(s) detected")

    # Generate JSON report
    json_report = validator.generate_json_report(all_validations)
    with open('link-validation-report.json', 'w') as f:
        f.write(json_report)
    print(f"\n✅ JSON report saved to link-validation-report.json")

    # Exit with error if broken links found
    if total_broken > 0:
        sys.exit(1)

    print(f"\n✅ Link validation complete")


if __name__ == '__main__':
    main()
