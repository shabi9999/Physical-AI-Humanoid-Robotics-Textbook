"""
Chapter 2 Example Tests

Validates that all Chapter 2 code examples can be imported and executed.

Run with:
    pytest test_chapter2.py -v

In ROS 2 environment:
    source /opt/ros/humble/setup.bash
    pip install pytest
    pytest test_chapter2.py -v
"""

import subprocess
import sys
import pytest
from pathlib import Path

# Get the chapter2 directory
CHAPTER2_DIR = Path(__file__).parent.parent / "chapter2"


class TestChapter2Examples:
    """Test suite for Chapter 2 examples."""

    def test_simple_agent_syntax(self):
        """Test that simple_agent.py has valid Python syntax."""
        script = CHAPTER2_DIR / "simple_agent.py"
        assert script.exists(), f"simple_agent.py not found at {script}"

        # Check Python syntax
        result = subprocess.run(
            [sys.executable, "-m", "py_compile", str(script)],
            capture_output=True,
            timeout=5
        )
        assert result.returncode == 0, f"Syntax error in simple_agent.py: {result.stderr.decode()}"

    def test_simple_agent_runs(self):
        """Test that simple_agent.py can be executed."""
        script = CHAPTER2_DIR / "simple_agent.py"
        assert script.exists(), f"simple_agent.py not found at {script}"

        # Run the script
        result = subprocess.run(
            [sys.executable, str(script)],
            capture_output=True,
            timeout=10,
            cwd=CHAPTER2_DIR
        )
        assert result.returncode == 0, f"Runtime error in simple_agent.py: {result.stderr.decode()}"
        assert b"Agent ran successfully" in result.stdout, "Expected success message not found"

    def test_sensor_bridge_syntax(self):
        """Test that sensor_bridge.py has valid Python syntax."""
        script = CHAPTER2_DIR / "sensor_bridge.py"
        assert script.exists(), f"sensor_bridge.py not found at {script}"

        result = subprocess.run(
            [sys.executable, "-m", "py_compile", str(script)],
            capture_output=True,
            timeout=5
        )
        assert result.returncode == 0, f"Syntax error in sensor_bridge.py: {result.stderr.decode()}"

    def test_control_publisher_syntax(self):
        """Test that control_publisher.py has valid Python syntax."""
        script = CHAPTER2_DIR / "control_publisher.py"
        assert script.exists(), f"control_publisher.py not found at {script}"

        result = subprocess.run(
            [sys.executable, "-m", "py_compile", str(script)],
            capture_output=True,
            timeout=5
        )
        assert result.returncode == 0, f"Syntax error in control_publisher.py: {result.stderr.decode()}"

    def test_all_files_have_docstrings(self):
        """Test that all example files have module-level docstrings."""
        for script in CHAPTER2_DIR.glob("*.py"):
            with open(script, "r") as f:
                content = f.read()

            # Check for opening docstring
            assert '"""' in content or "'''" in content, \
                f"{script.name} missing module docstring"

    def test_all_files_have_comments(self):
        """Test that all example files have inline comments explaining the code."""
        for script in CHAPTER2_DIR.glob("*.py"):
            with open(script, "r") as f:
                content = f.read()

            # Count comments
            comment_lines = [line for line in content.split("\n") if line.strip().startswith("#")]
            assert len(comment_lines) > 5, \
                f"{script.name} has insufficient inline comments (found {len(comment_lines)})"

    def test_simple_agent_has_class(self):
        """Test that simple_agent.py defines SimpleAgent class."""
        script = CHAPTER2_DIR / "simple_agent.py"
        with open(script, "r") as f:
            content = f.read()

        assert "class SimpleAgent" in content, \
            "simple_agent.py missing SimpleAgent class definition"

    def test_simple_agent_has_required_methods(self):
        """Test that SimpleAgent has required methods."""
        script = CHAPTER2_DIR / "simple_agent.py"
        with open(script, "r") as f:
            content = f.read()

        required_methods = ["__init__", "process_sensor_data", "make_decision"]
        for method in required_methods:
            assert f"def {method}" in content, \
                f"SimpleAgent missing required method: {method}"

    def test_readme_exists(self):
        """Test that README.md exists in chapter2 directory."""
        readme = CHAPTER2_DIR / "README.md"
        assert readme.exists(), "chapter2/README.md not found"

        # Check that README has useful content
        with open(readme, "r") as f:
            content = f.read()

        assert "Prerequisites" in content, "README missing Prerequisites section"
        assert "python3" in content.lower(), "README missing Python instructions"
        assert len(content) > 500, "README too short"


class TestChapter2Integration:
    """Integration tests for Chapter 2 (requires ROS 2 environment)."""

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
            from std_msgs.msg import Float32, String
            assert Float32 is not None
            assert String is not None
        except ImportError:
            pytest.skip("ROS 2 std_msgs not installed")

    def test_simple_agent_importable(self):
        """Test that SimpleAgent can be imported from simple_agent.py."""
        try:
            import sys
            from pathlib import Path

            # Add chapter2 to path so we can import simple_agent
            chapter2_path = str(CHAPTER2_DIR)
            if chapter2_path not in sys.path:
                sys.path.insert(0, chapter2_path)

            from simple_agent import SimpleAgent

            # Test instantiation
            agent = SimpleAgent()
            assert agent is not None

            # Test basic method
            decision = agent.make_decision()
            assert isinstance(decision, str)
            assert decision in ["move_forward", "turn_left", "turn_right", "stop"], \
                f"Unexpected decision value: {decision}"
        except ImportError as e:
            pytest.skip(f"Cannot import SimpleAgent: {e}")

    def test_simple_agent_process_sensor_data(self):
        """Test that SimpleAgent.process_sensor_data works correctly."""
        try:
            import sys
            from pathlib import Path

            chapter2_path = str(CHAPTER2_DIR)
            if chapter2_path not in sys.path:
                sys.path.insert(0, chapter2_path)

            from simple_agent import SimpleAgent

            agent = SimpleAgent()

            # Test processing sensor data
            decision = agent.process_sensor_data(5.0)
            assert isinstance(decision, str)
            assert len(decision) > 0

            # Test multiple readings
            for reading in [2.0, 5.0, 8.0, 10.0]:
                decision = agent.process_sensor_data(reading)
                assert isinstance(decision, str)
        except ImportError as e:
            pytest.skip(f"Cannot import SimpleAgent: {e}")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
