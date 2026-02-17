#!/usr/bin/env python3
"""
Pre-commit hook to verify the current branch's issue number and display the issue title.

Helps agents catch issue-number mismatches before committing to the wrong issue.
Informational only — always returns 0.
"""

import os
import re
import subprocess
import sys


def get_branch_name():
    """Get the current git branch name."""
    try:
        return subprocess.check_output(
            ["git", "branch", "--show-current"], text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return ""


def extract_issue_number(branch_name):
    """Extract issue number from branch name like feature/issue-42 or feature/ISSUE-42-desc."""
    match = re.match(r"^feature/[iI][sS][sS][uU][eE]-(\d+)", branch_name)
    if match:
        return match.group(1)
    return None


def fetch_issue_title(issue_num):
    """Fetch issue title and state via gh CLI. Returns (title, state) or (None, None)."""
    try:
        result = subprocess.check_output(
            [
                "gh",
                "issue",
                "view",
                issue_num,
                "--json",
                "title,state",
                "--jq",
                '.title + "||" + .state',
            ],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=10,
        ).strip()
        if "||" in result:
            title, state = result.rsplit("||", 1)
            return title, state
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, FileNotFoundError):
        pass
    return None, None


def main():
    branch = get_branch_name()
    if not branch:
        return 0

    issue_num = extract_issue_number(branch)
    if not issue_num:
        # Not a feature/issue-N branch — nothing to verify
        return 0

    # Try to fetch issue title from GitHub
    title, state = fetch_issue_title(issue_num)

    if title is not None:
        print(f"\n🔍 Issue #{issue_num}: {title}")
        print("   >>> Verify this matches your task <<<")
        if state and state.upper() == "CLOSED":
            print(f"   ⚠️  Warning: Issue #{issue_num} is CLOSED")
        print()
    else:
        # Fall back to WORKTREE_ISSUE_TITLE env var (set by worktree_enter.sh)
        env_title = os.environ.get("WORKTREE_ISSUE_TITLE", "")
        if env_title:
            print(f"\n🔍 Issue #{issue_num}: {env_title} (cached)")
            print("   >>> Verify this matches your task <<<")
            print()
        else:
            print(f"\n🔍 Issue #{issue_num}: (could not fetch title)")
            print(f"   Run: gh issue view {issue_num} --json title --jq '.title'")
            print()

    # Informational only — never block the commit
    return 0


if __name__ == "__main__":
    sys.exit(main())
