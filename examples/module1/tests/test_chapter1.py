"""
Chapter 1 Example Tests

Validates that all Chapter 1 code examples can be imported and executed.

Run with:
    pytest test_chapter1.py -v

In ROS 2 environment:
    source /opt/ros/humble/setup.bash
    pip install pytest
    pytest test_chapter1.py -v
"""

import subprocess
import time
import sys
import pytest
from pathlib import Path

# Get the chapter1 directory
CHAPTER1_DIR = Path(__file__).parent.parent / "chapter1"


class TestChapter1Examples:
    """Test suite for Chapter 1 examples."""

    def test_hello_ros2_syntax(self):
        """Test that hello_ros2.py has valid Python syntax."""
        script = CHAPTER1_DIR / "hello_ros2.py"
        assert script.exists(), f"hello_ros2.py not found at {script}"

        # Check Python syntax
        result = subprocess.run(
            [sys.executable, "-m", "py_compile", str(script)],
            capture_output=True,
            timeout=5
        )
        assert result.returncode == 0, f"Syntax error in hello_ros2.py: {result.stderr.decode()}"

    def test_publisher_syntax(self):
        """Test that publisher.py has valid Python syntax."""
        script = CHAPTER1_DIR / "publisher.py"
        assert script.exists(), f"publisher.py not found at {script}"

        result = subprocess.run(
            [sys.executable, "-m", "py_compile", str(script)],
            capture_output=True,
            timeout=5
        )
        assert result.returncode == 0, f"Syntax error in publisher.py: {result.stderr.decode()}"

    def test_subscriber_syntax(self):
        """Test that subscriber.py has valid Python syntax."""
        script = CHAPTER1_DIR / "subscriber.py"
        assert script.exists(), f"subscriber.py not found at {script}"

        result = subprocess.run(
            [sys.executable, "-m", "py_compile", str(script)],
            capture_output=True,
            timeout=5
        )
        assert result.returncode == 0, f"Syntax error in subscriber.py: {result.stderr.decode()}"

    def test_service_server_syntax(self):
        """Test that service_server.py has valid Python syntax."""
        script = CHAPTER1_DIR / "service_server.py"
        assert script.exists(), f"service_server.py not found at {script}"

        result = subprocess.run(
            [sys.executable, "-m", "py_compile", str(script)],
            capture_output=True,
            timeout=5
        )
        assert result.returncode == 0, f"Syntax error in service_server.py: {result.stderr.decode()}"

    def test_service_client_syntax(self):
        """Test that service_client.py has valid Python syntax."""
        script = CHAPTER1_DIR / "service_client.py"
        assert script.exists(), f"service_client.py not found at {script}"

        result = subprocess.run(
            [sys.executable, "-m", "py_compile", str(script)],
            capture_output=True,
            timeout=5
        )
        assert result.returncode == 0, f"Syntax error in service_client.py: {result.stderr.decode()}"

    def test_all_files_have_docstrings(self):
        """Test that all example files have module-level docstrings."""
        for script in CHAPTER1_DIR.glob("*.py"):
            with open(script, "r") as f:
                content = f.read()

            # Check for opening docstring
            assert '"""' in content or "'''" in content, \
                f"{script.name} missing module docstring"

    def test_all_files_have_comments(self):
        """Test that all example files have inline comments explaining the code."""
        for script in CHAPTER1_DIR.glob("*.py"):
            with open(script, "r") as f:
                content = f.read()

            # Count comments
            comment_lines = [line for line in content.split("\n") if line.strip().startswith("#")]
            assert len(comment_lines) > 5, \
                f"{script.name} has insufficient inline comments (found {len(comment_lines)})"

    def test_all_files_have_main_function(self):
        """Test that all example files have a main() function."""
        for script in CHAPTER1_DIR.glob("*.py"):
            if script.name == "hello_ros2.py":
                # hello_ros2 might not need main, skip
                continue

            with open(script, "r") as f:
                content = f.read()

            assert "def main" in content, \
                f"{script.name} missing main() function"

    def test_readme_exists(self):
        """Test that README.md exists in chapter1 directory."""
        readme = CHAPTER1_DIR / "README.md"
        assert readme.exists(), "chapter1/README.md not found"

        # Check that README has useful content
        with open(readme, "r") as f:
            content = f.read()

        assert "Prerequisites" in content, "README missing Prerequisites section"
        assert "python3" in content.lower(), "README missing Python instructions"
        assert len(content) > 500, "README too short"


class TestChapter1Integration:
    """Integration tests for Chapter 1 (requires ROS 2 environment)."""

    def test_rclpy_importable(self):
        """Test that rclpy is installed and importable."""
        try:
            import rclpy
            assert rclpy is not None
        except ImportError:
            pytest.skip("ROS 2 not installed (rclpy not found)")

    def test_std_msgs_importable(self):
        """Test that std_msgs is installed and importable."""
        try:
            from std_msgs.msg import String
            assert String is not None
        except ImportError:
            pytest.skip("ROS 2 std_msgs not installed")

    def test_example_interfaces_importable(self):
        """Test that example_interfaces is installed and importable."""
        try:
            from example_interfaces.srv import AddTwoInts
            assert AddTwoInts is not None
        except ImportError:
            pytest.skip("ROS 2 example_interfaces not installed")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
