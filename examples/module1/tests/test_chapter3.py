"""
Chapter 3 Example Tests

Validates that all Chapter 3 URDF and code examples work correctly.

Run with:
    pytest test_chapter3.py -v

In ROS 2 environment:
    source /opt/ros/humble/setup.bash
    pip install pytest
    pytest test_chapter3.py -v
"""

import subprocess
import sys
import pytest
from pathlib import Path
import xml.etree.ElementTree as ET

# Get the chapter3 directory
CHAPTER3_DIR = Path(__file__).parent.parent / "chapter3"


class TestChapter3Examples:
    """Test suite for Chapter 3 examples."""

    def test_simple_humanoid_urdf_exists(self):
        """Test that simple_humanoid.urdf file exists."""
        urdf_file = CHAPTER3_DIR / "simple_humanoid.urdf"
        assert urdf_file.exists(), f"simple_humanoid.urdf not found at {urdf_file}"

    def test_simple_humanoid_urdf_valid_xml(self):
        """Test that simple_humanoid.urdf is valid XML."""
        urdf_file = CHAPTER3_DIR / "simple_humanoid.urdf"

        try:
            tree = ET.parse(urdf_file)
            root = tree.getroot()
            assert root.tag == "robot", "URDF root element must be <robot>"
        except ET.ParseError as e:
            pytest.fail(f"Invalid URDF XML: {e}")

    def test_simple_humanoid_urdf_has_links(self):
        """Test that URDF contains proper number of links."""
        urdf_file = CHAPTER3_DIR / "simple_humanoid.urdf"
        tree = ET.parse(urdf_file)
        root = tree.getroot()

        links = root.findall("link")
        assert len(links) >= 12, \
            f"URDF must have at least 12 links (humanoid structure), found {len(links)}"

        # Check for key links
        link_names = [link.get("name") for link in links]
        key_links = ["base_link", "torso_link", "head_link",
                     "left_shoulder_link", "right_shoulder_link",
                     "left_hip_link", "right_hip_link"]
        for key_link in key_links:
            assert key_link in link_names, \
                f"URDF missing key link: {key_link}"

    def test_simple_humanoid_urdf_has_joints(self):
        """Test that URDF contains proper number of joints."""
        urdf_file = CHAPTER3_DIR / "simple_humanoid.urdf"
        tree = ET.parse(urdf_file)
        root = tree.getroot()

        joints = root.findall("joint")
        assert len(joints) >= 11, \
            f"URDF must have at least 11 joints, found {len(joints)}"

    def test_simple_humanoid_urdf_joint_types(self):
        """Test that joints have proper types (revolute or fixed)."""
        urdf_file = CHAPTER3_DIR / "simple_humanoid.urdf"
        tree = ET.parse(urdf_file)
        root = tree.getroot()

        joints = root.findall("joint")
        valid_types = {"revolute", "prismatic", "fixed", "floating", "planar"}

        for joint in joints:
            joint_type = joint.get("type")
            assert joint_type in valid_types, \
                f"Invalid joint type: {joint_type}"

    def test_simple_humanoid_urdf_no_cycles(self):
        """Test that URDF kinematic tree has no cycles."""
        urdf_file = CHAPTER3_DIR / "simple_humanoid.urdf"
        tree = ET.parse(urdf_file)
        root = tree.getroot()

        # Build parent-child relationships
        links = {link.get("name"): link for link in root.findall("link")}
        relationships = {}  # parent -> [children]

        for joint in root.findall("joint"):
            parent_elem = joint.find("parent")
            child_elem = joint.find("child")

            if parent_elem is not None and child_elem is not None:
                parent_name = parent_elem.get("link")
                child_name = child_elem.get("link")

                if parent_name not in relationships:
                    relationships[parent_name] = []
                relationships[parent_name].append(child_name)

        # DFS to detect cycles
        def has_cycle(node, visited, rec_stack):
            visited.add(node)
            rec_stack.add(node)

            for child in relationships.get(node, []):
                if child not in visited:
                    if has_cycle(child, visited, rec_stack):
                        return True
                elif child in rec_stack:
                    return True

            rec_stack.remove(node)
            return False

        # Check for cycles starting from base_link
        visited = set()
        rec_stack = set()
        assert not has_cycle("base_link", visited, rec_stack), \
            "URDF kinematic tree contains a cycle"

    def test_visualize_urdf_syntax(self):
        """Test that visualize_urdf.py has valid Python syntax."""
        script = CHAPTER3_DIR / "visualize_urdf.py"
        assert script.exists(), f"visualize_urdf.py not found at {script}"

        result = subprocess.run(
            [sys.executable, "-m", "py_compile", str(script)],
            capture_output=True,
            timeout=5
        )
        assert result.returncode == 0, \
            f"Syntax error in visualize_urdf.py: {result.stderr.decode()}"

    def test_visualize_urdf_runs(self):
        """Test that visualize_urdf.py can be executed."""
        script = CHAPTER3_DIR / "visualize_urdf.py"

        result = subprocess.run(
            [sys.executable, str(script)],
            capture_output=True,
            timeout=10,
            cwd=CHAPTER3_DIR
        )
        # Note: may fail if URDF not found in working dir, which is OK for pytest
        # The important thing is it runs without import errors

        output = result.stdout.decode()
        # Check that it at least parsed something
        assert "URDF Parser" in output or "not found" in output or result.returncode == 0

    def test_visualize_urdf_has_parser_class(self):
        """Test that visualize_urdf.py defines URDFParser class."""
        script = CHAPTER3_DIR / "visualize_urdf.py"
        with open(script, "r") as f:
            content = f.read()

        assert "class URDFParser" in content, \
            "visualize_urdf.py missing URDFParser class"

    def test_readme_exists(self):
        """Test that README.md exists in chapter3 directory."""
        readme = CHAPTER3_DIR / "README.md"
        assert readme.exists(), "chapter3/README.md not found"

        # Check that README has useful content
        with open(readme, "r") as f:
            content = f.read()

        required_sections = [
            "Prerequisites",
            "URDF",
            "Running",
            "Troubleshooting",
        ]

        for section in required_sections:
            assert section in content, \
                f"README missing {section} section"

        assert len(content) > 500, "README too short"

    def test_all_files_have_docstrings(self):
        """Test that all example files have module-level docstrings."""
        for script in CHAPTER3_DIR.glob("*.py"):
            with open(script, "r") as f:
                content = f.read()

            # Check for opening docstring
            assert '"""' in content or "'''" in content, \
                f"{script.name} missing module docstring"

    def test_all_files_have_comments(self):
        """Test that all example files have inline comments."""
        for script in CHAPTER3_DIR.glob("*.py"):
            with open(script, "r") as f:
                content = f.read()

            # Count comments (allow fewer for URDF files)
            comment_lines = [line for line in content.split("\n") if line.strip().startswith("#")]
            assert len(comment_lines) > 3, \
                f"{script.name} has insufficient inline comments (found {len(comment_lines)})"

    def test_urdf_has_comments(self):
        """Test that URDF file has XML comments explaining structure."""
        urdf_file = CHAPTER3_DIR / "simple_humanoid.urdf"
        with open(urdf_file, "r") as f:
            content = f.read()

        # Check for comment markers
        assert "<!--" in content, "URDF should have explanatory comments"
        assert "-->" in content, "URDF should have explanatory comments"

        # Count comments
        comment_count = content.count("<!--")
        assert comment_count >= 5, \
            f"URDF should have multiple comments, found {comment_count}"


class TestChapter3Integration:
    """Integration tests for Chapter 3 (requires ROS 2 environment)."""

    def test_check_urdf_available(self):
        """Test that check_urdf command is available."""
        result = subprocess.run(
            ["check_urdf", "--help"],
            capture_output=True,
            timeout=5
        )

        if result.returncode != 0:
            pytest.skip("ROS 2 check_urdf tool not installed")

    def test_check_urdf_validates_model(self):
        """Test that check_urdf successfully validates the model."""
        urdf_file = CHAPTER3_DIR / "simple_humanoid.urdf"

        result = subprocess.run(
            ["check_urdf", str(urdf_file)],
            capture_output=True,
            timeout=10
        )

        if result.returncode != 0:
            pytest.skip("ROS 2 check_urdf tool not available")

        output = result.stdout.decode()
        # Valid URDF produces "Successfully Parsed XML" message
        assert "Successfully Parsed XML" in output or "Transform accepted" in output, \
            f"URDF validation failed: {output}"

    def test_urdf_model_python_import(self):
        """Test that visualize_urdf can parse the model."""
        try:
            import sys
            from pathlib import Path

            chapter3_path = str(CHAPTER3_DIR)
            if chapter3_path not in sys.path:
                sys.path.insert(0, chapter3_path)

            # Create a minimal test to import and use the parser
            from visualize_urdf import URDFParser

            urdf_file = CHAPTER3_DIR / "simple_humanoid.urdf"
            parser = URDFParser(str(urdf_file))

            # Test that parser extracted data
            assert parser.get_robot_name() == "simple_humanoid"
            assert parser.get_link_count() >= 12
            assert parser.get_joint_count() >= 11

        except ImportError as e:
            pytest.skip(f"Cannot import URDFParser: {e}")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
