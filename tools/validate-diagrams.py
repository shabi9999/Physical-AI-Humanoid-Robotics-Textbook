#!/usr/bin/env python3
"""
Diagram Validator for Humanoid Robotics Textbook

Validates:
- Mermaid diagram syntax (flowcharts, graphs, diagrams)
- ASCII art structure and alignment
- Diagram presence and minimum count per chapter
- Proper code fence formatting

Usage:
    python3 validate-diagrams.py
    python3 validate-diagrams.py path/to/chapter.md
    python3 validate-diagrams.py --all

Output:
    - Diagram count and type per chapter
    - Syntax validation results
    - Summary with exit code
"""

import re
import json
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Set
from dataclasses import dataclass, field
from collections import defaultdict


@dataclass
class DiagramValidationResult:
    """Result for a single diagram validation."""
    diagram_id: str
    diagram_type: str  # 'mermaid', 'ascii'
    line_number: int
    status: str  # 'valid', 'error'
    message: str = ''
    diagram_content: str = ''


@dataclass
class ChapterDiagramValidation:
    """Validation results for diagrams in a chapter."""
    chapter_id: str
    filepath: str
    diagram_count: int
    mermaid_count: int
    ascii_count: int
    valid_diagrams: int
    invalid_diagrams: int
    warnings: List[str] = field(default_factory=list)
    diagram_results: List[DiagramValidationResult] = field(default_factory=list)


class DiagramValidator:
    """Validates diagrams in markdown chapters."""

    # Pattern for mermaid code blocks
    MERMAID_PATTERN = r'```mermaid\n([\s\S]*?)\n```'

    # Pattern for ASCII code blocks
    ASCII_PATTERN = r'```\n([\s\S]*?)\n```'

    # Mermaid keywords by type
    MERMAID_KEYWORDS = {
        'graph': ['graph TD', 'graph LR', 'graph RL', 'graph BT'],
        'flowchart': ['flowchart TD', 'flowchart LR', 'flowchart RL', 'flowchart BT'],
        'sequenceDiagram': ['sequenceDiagram'],
        'classDiagram': ['classDiagram'],
        'stateDiagram': ['stateDiagram'],
        'gitGraph': ['gitGraph'],
        'pie': ['pie title'],
    }

    def __init__(self, base_dir: str = 'my-website/docs'):
        self.base_dir = Path(base_dir)

    def validate_mermaid_syntax(self, content: str) -> Tuple[bool, str]:
        """Validate Mermaid diagram syntax."""
        lines = content.strip().split('\n')
        if not lines:
            return False, 'Empty mermaid block'

        # Check first line for valid mermaid keyword
        first_line = lines[0].strip()
        is_valid_start = False

        for keywords in self.MERMAID_KEYWORDS.values():
            for keyword in keywords:
                if first_line.startswith(keyword):
                    is_valid_start = True
                    break
            if is_valid_start:
                break

        if not is_valid_start:
            return False, f'Invalid mermaid start: {first_line}'

        # Basic bracket/parenthesis matching
        open_brackets = content.count('{') + content.count('[') + content.count('(')
        close_brackets = content.count('}') + content.count(']') + content.count(')')

        if open_brackets != close_brackets:
            return False, f'Bracket mismatch: {open_brackets} open, {close_brackets} close'

        # Check for required elements in graph/flowchart
        if 'graph' in first_line or 'flowchart' in first_line:
            if '-->' not in content and '---|' not in content and '-.-' not in content:
                return False, 'No connections found (missing -->, ---|, or -.-.)'

        return True, 'Valid mermaid syntax'

    def validate_ascii_structure(self, content: str) -> Tuple[bool, str]:
        """Validate ASCII art structure."""
        lines = content.strip().split('\n')
        if not lines:
            return False, 'Empty ASCII block'

        # ASCII should have recognizable box structure or tree structure
        has_box_chars = any(c in content for c in ['┌', '├', '└', '─', '│', '┐', '┘'])
        has_arrows = '-->' in content or '<--' in content or '→' in content or '←' in content
        has_pipes = '|' in content
        has_dashes = '-' in content

        if has_box_chars:
            return True, 'Valid ASCII box art'
        elif has_arrows or (has_pipes and has_dashes):
            return True, 'Valid ASCII tree/flow structure'
        else:
            return False, 'No recognizable ASCII structure (missing boxes, arrows, or tree symbols)'

    def validate_diagram_presence(self, chapter_id: str, diagram_count: int) -> Tuple[bool, str]:
        """Check if diagram count meets minimum requirements."""
        # Target: 3-5 diagrams per chapter for visualization
        if diagram_count == 0:
            return False, 'No diagrams found (target: 3-5 per chapter)'
        elif diagram_count < 2:
            return False, f'Only {diagram_count} diagram(s) found (target: 3-5 per chapter)'
        elif diagram_count < 3:
            return False, f'Only {diagram_count} diagram(s) found (target: 3-5 per chapter)'
        elif diagram_count > 8:
            return False, f'Too many diagrams: {diagram_count} (target: 3-5 per chapter)'
        else:
            return True, f'{diagram_count} diagrams found (target: 3-5 per chapter)'

    def extract_mermaid_diagrams(self, content: str) -> List[Tuple[int, str]]:
        """Extract mermaid diagrams with line numbers."""
        diagrams = []
        for match in re.finditer(self.MERMAID_PATTERN, content):
            # Count lines before this match to get approximate line number
            lines_before = content[:match.start()].count('\n')
            diagram_content = match.group(1)
            diagrams.append((lines_before + 1, diagram_content))
        return diagrams

    def find_ascii_diagrams(self, content: str) -> List[Tuple[int, str]]:
        """Find ASCII art blocks (heuristic: code blocks with special chars)."""
        diagrams = []
        # Look for code blocks that aren't mermaid
        lines = content.split('\n')

        i = 0
        while i < len(lines):
            if lines[i].strip() == '```' and (i == 0 or 'mermaid' not in lines[i-1]):
                # Found start of code block
                start = i
                i += 1
                block = []
                while i < len(lines) and lines[i].strip() != '```':
                    block.append(lines[i])
                    i += 1

                if i < len(lines) and lines[i].strip() == '```':
                    # Found end of block
                    block_content = '\n'.join(block)
                    # Check if it looks like ASCII art (has box chars or structure)
                    if any(c in block_content for c in ['┌', '├', '└', '─', '│', '→', '←', '|', '--']):
                        diagrams.append((start + 1, block_content))
                i += 1
            else:
                i += 1

        return diagrams

    def validate_chapter(self, filepath: Path) -> ChapterDiagramValidation:
        """Validate all diagrams in a chapter."""
        chapter_id = f"{filepath.parent.name}/{filepath.stem}"

        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract diagrams
        mermaid_diagrams = self.extract_mermaid_diagrams(content)
        ascii_diagrams = self.find_ascii_diagrams(content)

        total_diagrams = len(mermaid_diagrams) + len(ascii_diagrams)
        diagram_results = []
        valid_count = 0
        invalid_count = 0
        warnings = []

        # Validate mermaid diagrams
        for idx, (line_num, diagram_content) in enumerate(mermaid_diagrams, 1):
            is_valid, message = self.validate_mermaid_syntax(diagram_content)
            status = 'valid' if is_valid else 'error'

            if is_valid:
                valid_count += 1
            else:
                invalid_count += 1

            result = DiagramValidationResult(
                diagram_id=f"mermaid-{idx}",
                diagram_type='mermaid',
                line_number=line_num,
                status=status,
                message=message,
                diagram_content=diagram_content[:100] + ('...' if len(diagram_content) > 100 else '')
            )
            diagram_results.append(result)

        # Validate ASCII diagrams
        for idx, (line_num, diagram_content) in enumerate(ascii_diagrams, 1):
            is_valid, message = self.validate_ascii_structure(diagram_content)
            status = 'valid' if is_valid else 'error'

            if is_valid:
                valid_count += 1
            else:
                invalid_count += 1

            result = DiagramValidationResult(
                diagram_id=f"ascii-{idx}",
                diagram_type='ascii',
                line_number=line_num,
                status=status,
                message=message,
                diagram_content=diagram_content[:100] + ('...' if len(diagram_content) > 100 else '')
            )
            diagram_results.append(result)

        # Check diagram count
        diagram_count_valid, diagram_count_msg = self.validate_diagram_presence(
            chapter_id, total_diagrams
        )
        if not diagram_count_valid:
            warnings.append(diagram_count_msg)

        return ChapterDiagramValidation(
            chapter_id=chapter_id,
            filepath=str(filepath),
            diagram_count=total_diagrams,
            mermaid_count=len(mermaid_diagrams),
            ascii_count=len(ascii_diagrams),
            valid_diagrams=valid_count,
            invalid_diagrams=invalid_count,
            warnings=warnings,
            diagram_results=diagram_results
        )

    def find_all_chapters(self) -> List[Path]:
        """Find all chapter markdown files."""
        chapters = []
        for module_dir in sorted(self.base_dir.iterdir()):
            if module_dir.is_dir() and module_dir.name.startswith('module'):
                for file in sorted(module_dir.glob('chapter*.md')):
                    chapters.append(file)
        return chapters

    def print_report(self, validation: ChapterDiagramValidation):
        """Print validation report for a single chapter."""
        print(f"\n{'='*80}")
        print(f"CHAPTER: {validation.chapter_id}")
        print(f"{'='*80}")
        print(f"Total Diagrams:      {validation.diagram_count}")
        print(f"  Mermaid:           {validation.mermaid_count}")
        print(f"  ASCII:             {validation.ascii_count}")
        print(f"Valid:               {validation.valid_diagrams}")
        print(f"Invalid:             {validation.invalid_diagrams}")

        if validation.warnings:
            print(f"\n⚠️  WARNINGS:")
            for warning in validation.warnings:
                print(f"    • {warning}")

        if validation.diagram_results:
            print(f"\nDiagram Details:")
            for result in validation.diagram_results:
                status_icon = '❌' if result.status == 'error' else '✅'
                print(f"  {status_icon} {result.diagram_id} (line {result.line_number})")
                print(f"      Type: {result.diagram_type}")
                print(f"      Message: {result.message}")

        if not validation.warnings and validation.invalid_diagrams == 0:
            print(f"\n✅ All diagrams valid")

    def generate_json_report(self, all_validations: List[ChapterDiagramValidation]) -> str:
        """Generate JSON report for CI/CD integration."""
        total_chapters = len(all_validations)
        total_diagrams = sum(v.diagram_count for v in all_validations)
        total_valid = sum(v.valid_diagrams for v in all_validations)
        total_invalid = sum(v.invalid_diagrams for v in all_validations)
        total_warnings = sum(len(v.warnings) for v in all_validations)

        data = {
            'summary': {
                'total_chapters': total_chapters,
                'total_diagrams': total_diagrams,
                'valid_diagrams': total_valid,
                'invalid_diagrams': total_invalid,
                'warnings': total_warnings,
                'health': 'good' if total_invalid == 0 and total_warnings == 0 else 'needs_attention'
            },
            'chapters': [
                {
                    'chapter_id': v.chapter_id,
                    'diagram_count': v.diagram_count,
                    'mermaid': v.mermaid_count,
                    'ascii': v.ascii_count,
                    'valid': v.valid_diagrams,
                    'invalid': v.invalid_diagrams,
                    'warnings': v.warnings,
                    'diagrams': [
                        {
                            'id': r.diagram_id,
                            'type': r.diagram_type,
                            'line': r.line_number,
                            'status': r.status,
                            'message': r.message
                        }
                        for r in v.diagram_results
                    ]
                }
                for v in all_validations
            ]
        }
        return json.dumps(data, indent=2)


def main():
    """Main entry point."""
    validator = DiagramValidator()

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

    print(f"\n📊 Validating diagrams in {len(chapters)} chapters...")

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
    print(f"DIAGRAM VALIDATION SUMMARY")
    print(f"{'='*80}")
    total_chapters = len(all_validations)
    total_diagrams = sum(v.diagram_count for v in all_validations)
    total_mermaid = sum(v.mermaid_count for v in all_validations)
    total_ascii = sum(v.ascii_count for v in all_validations)
    total_valid = sum(v.valid_diagrams for v in all_validations)
    total_invalid = sum(v.invalid_diagrams for v in all_validations)

    print(f"Total Chapters:      {total_chapters}")
    print(f"Total Diagrams:      {total_diagrams}")
    print(f"  Mermaid:           {total_mermaid}")
    print(f"  ASCII:             {total_ascii}")
    print(f"Valid:               {total_valid}")
    print(f"Invalid:             {total_invalid}")

    # Generate JSON report
    json_report = validator.generate_json_report(all_validations)
    with open('diagram-validation-report.json', 'w') as f:
        f.write(json_report)
    print(f"\n✅ JSON report saved to diagram-validation-report.json")

    # Exit with error if invalid diagrams found
    if total_invalid > 0:
        print(f"\n❌ {total_invalid} invalid diagram(s) detected")
        sys.exit(1)

    print(f"\n✅ Diagram validation complete")


if __name__ == '__main__':
    main()
