#!/usr/bin/env python3
"""
Get Repository Info

This script searches for a repository in the workspace .repos files and returns
specific information (default: version/branch).

Usage:
    python3 get_repo_info.py <repo_name>

Output:
    The version string (e.g. "jazzy", "main", "feature/foo")
    Returns "unknown" if not found.
"""

import os
import sys
import argparse

# Add lib directory to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(SCRIPT_DIR, "lib"))

from workspace import find_repo_version

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Get repository info from .repos files")
    parser.add_argument("repo_name", help="Name of the repository to look up")
    args = parser.parse_args()

    version = find_repo_version(args.repo_name)
    print(version)
