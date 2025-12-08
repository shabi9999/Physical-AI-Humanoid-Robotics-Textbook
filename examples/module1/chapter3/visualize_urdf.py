#!/usr/bin/env python3
"""
URDF Visualization Helper Script

This script demonstrates how to work with URDF files in Python.
It provides utilities for:
1. Parsing URDF XML structure
2. Extracting robot information (links, joints)
3. Validating URDF syntax
4. Printing the kinematic tree structure

Key concepts:
1. URDF as XML document structure
2. Links (rigid bodies) and joints (connections)
3. Kinematic tree hierarchy
4. Joint types and properties

This script does NOT require ROS 2 to be running.
It uses Python's built-in XML parsing.

Run with:
    python3 visualize_urdf.py

Expected output:
    Robot name: simple_humanoid
    Total links: 13
    Total joints: 12

    Kinematic tree:
    base_link
    └── torso_link
        ├── head_link
        ├── left_shoulder_link
        │   └── left_elbow_link
        │       └── left_wrist_link
        ├── right_shoulder_link
        ...
"""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Dict, Set


class URDFParser:
    """
    Parse and analyze URDF files.

    This parser reads a URDF XML file and extracts information about
    the robot structure: links, joints, and their relationships.
    """

    def __init__(self, urdf_file: str):
        """
        Initialize the URDF parser with a file.

        Args:
            urdf_file (str): Path to the URDF file to parse
        """
        self.urdf_file = Path(urdf_file)
        self.tree = None
        self.root = None
        self.links: Dict[str, dict] = {}
        self.joints: Dict[str, dict] = {}
        self.robot_name = None

        # Parse the URDF file
        self._parse_urdf()

    def _parse_urdf(self):
        """
        Parse the URDF XML file.

        Reads the file and extracts robot name, links, and joints.
        """
        if not self.urdf_file.exists():
            raise FileNotFoundError(f"URDF file not found: {self.urdf_file}")

        try:
            # Parse XML
            self.tree = ET.parse(self.urdf_file)
            self.root = self.tree.getroot()

            # Extract robot name
            self.robot_name = self.root.get("name")

            # Extract all links
            for link_elem in self.root.findall("link"):
                link_name = link_elem.get("name")
                self.links[link_name] = {
                    "name": link_name,
                    "has_inertial": link_elem.find("inertial") is not None,
                    "has_visual": link_elem.find("visual") is not None,
                    "has_collision": link_elem.find("collision") is not None,
                }

            # Extract all joints
            for joint_elem in self.root.findall("joint"):
                joint_name = joint_elem.get("name")
                joint_type = joint_elem.get("type")
                parent_elem = joint_elem.find("parent")
                child_elem = joint_elem.find("child")

                parent_name = parent_elem.get("link") if parent_elem is not None else None
                child_name = child_elem.get("link") if child_elem is not None else None

                self.joints[joint_name] = {
                    "name": joint_name,
                    "type": joint_type,
                    "parent": parent_name,
                    "child": child_name,
                }

        except ET.ParseError as e:
            raise ValueError(f"Invalid URDF XML syntax: {e}")

    def get_robot_name(self) -> str:
        """Return the robot name from URDF."""
        return self.robot_name

    def get_links(self) -> List[str]:
        """Return list of all link names."""
        return list(self.links.keys())

    def get_joints(self) -> List[str]:
        """Return list of all joint names."""
        return list(self.joints.keys())

    def get_link_count(self) -> int:
        """Return number of links."""
        return len(self.links)

    def get_joint_count(self) -> int:
        """Return number of joints."""
        return len(self.joints)

    def build_kinematic_tree(self, root_link: str = "base_link") -> Dict:
        """
        Build the kinematic tree starting from a root link.

        Args:
            root_link (str): Starting link for the tree

        Returns:
            dict: Hierarchical structure of links and joints
        """
        tree = {
            "link": root_link,
            "children": [],
        }

        # Find all joints where this link is the parent
        for joint_name, joint_info in self.joints.items():
            if joint_info["parent"] == root_link:
                child_link = joint_info["child"]
                child_tree = self.build_kinematic_tree(child_link)
                child_tree["joint"] = joint_name
                child_tree["joint_type"] = joint_info["type"]
                tree["children"].append(child_tree)

        return tree

    def print_kinematic_tree(self, tree: Dict = None, indent: str = "", is_last: bool = True):
        """
        Print the kinematic tree in a readable format.

        Args:
            tree (dict): Tree node to print (defaults to root)
            indent (str): Current indentation level
            is_last (bool): Whether this is the last child
        """
        if tree is None:
            tree = self.build_kinematic_tree()

        # Print current link
        if indent == "":
            print(tree["link"])
        else:
            connector = "+-- " if is_last else "+-- "
            print(f"{indent}{connector}{tree['link']}", end="")
            if "joint" in tree:
                print(f" [{tree['joint_type']}]", end="")
            print()

        # Print children
        children = tree.get("children", [])
        for i, child in enumerate(children):
            is_last_child = (i == len(children) - 1)
            next_indent = indent + ("    " if is_last_child else "|   ")
            self.print_kinematic_tree(child, next_indent, is_last_child)

    def print_summary(self):
        """Print a summary of the URDF structure."""
        print("=" * 60)
        print(f"URDF Parser Summary")
        print("=" * 60)
        print(f"Robot name: {self.robot_name}")
        print(f"File: {self.urdf_file}")
        print()

        print(f"Structure:")
        print(f"  Links: {self.get_link_count()}")
        print(f"  Joints: {self.get_joint_count()}")
        print()

        print("Link Properties:")
        links_with_inertia = sum(
            1 for link in self.links.values() if link["has_inertial"]
        )
        links_with_visual = sum(
            1 for link in self.links.values() if link["has_visual"]
        )
        links_with_collision = sum(
            1 for link in self.links.values() if link["has_collision"]
        )

        print(f"  With inertial properties: {links_with_inertia}")
        print(f"  With visual geometry: {links_with_visual}")
        print(f"  With collision geometry: {links_with_collision}")
        print()

        print("Joint Types:")
        joint_types: Dict[str, int] = {}
        for joint_info in self.joints.values():
            jtype = joint_info["type"]
            joint_types[jtype] = joint_types.get(jtype, 0) + 1

        for jtype, count in sorted(joint_types.items()):
            print(f"  {jtype}: {count}")
        print()

        print("Kinematic Tree:")
        print()
        self.print_kinematic_tree()
        print()

        print("=" * 60)

    def validate(self) -> bool:
        """
        Validate the URDF structure.

        Checks for common errors like missing parent links, cycles, etc.

        Returns:
            bool: True if valid, raises exception otherwise
        """
        # Check that all joint parent/child links exist
        for joint_name, joint_info in self.joints.items():
            parent = joint_info["parent"]
            child = joint_info["child"]

            if parent not in self.links:
                raise ValueError(f"Joint {joint_name}: parent link '{parent}' not found")

            if child not in self.links:
                raise ValueError(f"Joint {joint_name}: child link '{child}' not found")

            # Can't have a joint from a link to itself
            if parent == child:
                raise ValueError(f"Joint {joint_name}: parent and child are the same link")

        # Check for cycles in the kinematic tree
        # A valid robot has a tree structure (no cycles)
        visited: Set[str] = set()
        cycles = self._detect_cycles("base_link", visited, set())

        if cycles:
            raise ValueError(f"Kinematic tree has cycles: {cycles}")

        print("[OK] URDF validation passed!")
        return True

    def _detect_cycles(
        self, link: str, visited: Set[str], rec_stack: Set[str]
    ) -> List[str]:
        """
        Detect cycles in the kinematic tree using DFS.

        Args:
            link (str): Current link
            visited (Set[str]): All visited links
            rec_stack (Set[str]): Current recursion stack

        Returns:
            List[str]: List of cycles found (empty if none)
        """
        visited.add(link)
        rec_stack.add(link)

        # Find all children of this link
        for joint_info in self.joints.values():
            if joint_info["parent"] == link:
                child = joint_info["child"]

                if child not in visited:
                    cycles = self._detect_cycles(child, visited, rec_stack)
                    if cycles:
                        return cycles

                elif child in rec_stack:
                    return [link, child]

        rec_stack.remove(link)
        return []


def main():
    """
    Main function: Parse and display URDF information.
    """
    # Find the simple_humanoid.urdf file
    script_dir = Path(__file__).parent
    urdf_file = script_dir / "simple_humanoid.urdf"

    if not urdf_file.exists():
        print(f"Error: URDF file not found at {urdf_file}")
        print("\nMake sure you run this script from the chapter3 directory:")
        print("  cd examples/module1/chapter3")
        print("  python3 visualize_urdf.py")
        return

    try:
        # Create parser
        parser = URDFParser(str(urdf_file))

        # Print summary and kinematic tree
        parser.print_summary()

        # Validate the URDF
        parser.validate()

        print("\n[SUCCESS] URDF parsing and visualization complete!")
        print("\nNext steps:")
        print("  - Use 'check_urdf' tool to validate in ROS 2 environment:")
        print("    check_urdf simple_humanoid.urdf")
        print("  - Visualize in RViz with robot_state_publisher")
        print("  - Modify joint limits and re-validate")

    except Exception as e:
        print(f"Error parsing URDF: {e}")


if __name__ == "__main__":
    main()
