#!/usr/bin/env python3
"""
Get Repository Info

This script searches for a repository in the workspace .repos files and returns
specific information (default: version/branch).

Usage:
    python3 get_repo_info.py <repo_name>

Output:
    The version string (e.g. "jazzy", "main", "feature/foo") on stdout.
    "unknown" when every manifest parsed and none declares the repo.

Exit status:
    0  the version was printed — either a real version or a well-founded
       "unknown" (every manifest was read; none declares this repo).
    1  the answer is not knowable: at least one .repos file could not be
       parsed and none of the readable ones declares the repo. Printing
       "unknown" here would be a guess, and callers branch a worktree off
       this answer; before #609 it was an unhandled traceback, which is the
       same non-answer with a worse message.
    2  argparse usage error.
"""

import os
import sys
import argparse

# Add lib directory to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(SCRIPT_DIR, "lib"))

from workspace import WorkspaceConfigError, find_repo_version  # noqa: E402


def main(argv=None):
    """Print the repo's configured version, or refuse to guess one.

    A function rather than an `if __name__` body so the refusal path is
    asserted directly in test_workspace_lib.py — it is the behaviour the
    docstring's exit-status contract promises, and it had no handler at all
    before #609.
    """
    parser = argparse.ArgumentParser(description="Get repository info from .repos files")
    parser.add_argument("repo_name", help="Name of the repository to look up")
    args = parser.parse_args(argv)

    try:
        version = find_repo_version(args.repo_name)
    except WorkspaceConfigError as exc:
        # Not "unknown": the answer may have been in the manifest we could not
        # read, and callers branch a worktree off this answer (#609).
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
    print(version)


if __name__ == "__main__":
    main()
