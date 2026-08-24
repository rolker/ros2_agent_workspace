#!/usr/bin/env python3
"""
List Overlay Repositories

This script scans the configs/ directory for .repos files and outputs a JSON
list of all repositories defined across all layers (excluding underlay.repos).

Usage:
    python3 list_overlay_repos.py

Output:
    JSON array containing:
    - Repository name
    - URL
    - Version/branch
    - Source .repos file
"""

import os
import sys
import json
import argparse

# Add lib directory to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(SCRIPT_DIR, "lib"))

from workspace import WorkspaceConfigError, get_overlay_repos  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description="List repositories defined in .repos files")
    parser.add_argument(
        "--include-underlay", action="store_true", help="Include repositories from underlay.repos"
    )
    parser.add_argument("--format", choices=["json", "names"], default="json", help="Output format")

    args = parser.parse_args()

    try:
        repos = get_overlay_repos(include_underlay=args.include_underlay)
    except WorkspaceConfigError as exc:
        # Printing `[]` for an unparseable manifest hands every consumer an
        # empty list that is indistinguishable from "no repos configured" (#609).
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)

    if args.format == "names":
        for repo in repos:
            print(repo["name"])
    else:
        print(json.dumps(repos, indent=2))


if __name__ == "__main__":
    main()
