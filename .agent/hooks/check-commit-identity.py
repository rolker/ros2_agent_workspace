#!/usr/bin/env python3
"""
Pre-commit hook to block commits with unconfigured or unrecognized git identity.

Agents frequently commit with wrong identity when they forget to run
set_git_identity_env.sh. This hook catches that before it becomes a problem.
"""

import fnmatch
import os
import subprocess
import sys

# Accepted email patterns
# - roland+*@ccom.unh.edu: any agent identity (copilot, gemini, claude, etc.)
# - roland@ccom.unh.edu: human (work)
# - roland@rolker.net: human (personal)
ACCEPTED_PATTERNS = [
    "roland+*@ccom.unh.edu",
    "roland@ccom.unh.edu",
    "roland@rolker.net",
]


def get_commit_email():
    """Get the email that will be used for the next commit.

    GIT_AUTHOR_EMAIL env var takes precedence over git config user.email.
    """
    email = os.environ.get("GIT_AUTHOR_EMAIL", "").strip()
    if email:
        return email

    try:
        result = subprocess.run(
            ["git", "config", "user.email"],
            capture_output=True,
            text=True,
            check=True,
        )
        return result.stdout.strip()
    except subprocess.CalledProcessError:
        return ""


def is_accepted(email):
    """Check if email matches any accepted pattern."""
    if not email:
        return False
    for pattern in ACCEPTED_PATTERNS:
        if fnmatch.fnmatch(email, pattern):
            return True
    return False


def main():
    """Verify commit identity is configured correctly."""
    email = get_commit_email()

    if is_accepted(email):
        return 0

    print()
    print("ERROR: Commit identity is not configured or not recognized!")
    print()
    if email:
        print(f"  Current email: {email}")
    else:
        print("  Current email: (not set)")
    print()
    print(f"  Accepted patterns: {', '.join(ACCEPTED_PATTERNS)}")
    print()
    print("  To fix, run one of:")
    print('    source .agent/scripts/set_git_identity_env.sh --detect')
    print('    source .agent/scripts/set_git_identity_env.sh --agent <framework>')
    print()
    return 1


if __name__ == "__main__":
    sys.exit(main())
