#!/usr/bin/env python3
"""
Pre-commit hook to warn about untracked files in source directories.
This prevents accidental commits of temporary artifacts left by agents.
"""

import os
import subprocess
import sys
from pathlib import Path

# Common temp file patterns that should not be in source directories
SUSPICIOUS_PATTERNS = {
    ".tmp", ".temp", ".bak", ".backup",
    "_temp", "_backup",
    ".analysis", ".report", ".debug",
    ".log", ".out", ".err",
    ".swp", ".swo",  # Editor artifacts
}

WORKSPACES_DIR = "workspaces"


def get_untracked_files():
    """Get list of untracked files in the repository."""
    try:
        result = subprocess.run(
            ["git", "ls-files", "--others", "--exclude-standard"],
            capture_output=True,
            text=True,
            check=True,
        )
        return result.stdout.strip().split("\n") if result.stdout.strip() else []
    except subprocess.CalledProcessError:
        return []


def is_in_source_dir(filepath):
    """Check if file is in a source directory."""
    return f"{WORKSPACES_DIR}/" in filepath and "/src/" in filepath


def is_suspicious(filename):
    """Check if filename matches suspicious patterns."""
    name = Path(filename).name
    for pattern in SUSPICIOUS_PATTERNS:
        if pattern in name or name.endswith(pattern):
            return True
    return False


def main():
    """Check for suspicious untracked files in source directories."""
    untracked = get_untracked_files()
    suspicious = []

    for filepath in untracked:
        if filepath and is_in_source_dir(filepath) and is_suspicious(filepath):
            suspicious.append(filepath)

    if suspicious:
        print("\n⚠️  WARNING: Suspicious untracked files found in source directories!")
        print(
            "   These may be temporary artifacts that should not be committed:\n"
        )
        for filepath in suspicious:
            print(f"   - {filepath}")
        print("\n   Options:")
        print("   1. Delete these files if they're temporary")
        print("   2. Move them to .agent/scratchpad/ if they need to persist")
        print("   3. Add to .gitignore if they should be ignored\n")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
