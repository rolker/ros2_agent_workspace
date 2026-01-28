#!/usr/bin/env python3
"""
Pre-commit hook to check if default branch has new commits.
This helps developers stay aware of upstream changes and decide when to merge/rebase.
"""

import subprocess
import sys
from pathlib import Path


def main():
    """Check for updates in default branch and provide recommendations."""
    # Get repository root
    try:
        repo_root = (
            subprocess.check_output(
                ["git", "rev-parse", "--show-toplevel"], text=True
            )
            .strip()
        )
    except subprocess.CalledProcessError:
        # Not in a git repository, skip
        return 0

    # Path to the check script
    check_script = Path(repo_root) / ".agent" / "scripts" / "check_branch_updates.sh"
    
    if not check_script.exists():
        # Script not found, skip silently
        return 0

    # Run the check script (non-strict mode - informational only)
    try:
        result = subprocess.run(
            [str(check_script)],
            capture_output=False,
            text=True,
        )
        
        # Exit code 0: up-to-date or on default branch
        # Exit code 1: updates available (informational)
        # Exit code 2: error
        
        # We don't block commits in pre-commit hook, just inform
        # Users can run with --strict if they want to enforce
        return 0
        
    except subprocess.CalledProcessError as e:
        # If check script fails, don't block the commit
        print(f"⚠️  Warning: Branch update check failed: {e}")
        return 0
    except FileNotFoundError:
        # Script not executable or not found
        return 0


if __name__ == "__main__":
    sys.exit(main())
