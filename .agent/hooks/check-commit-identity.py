#!/usr/bin/env python3
"""Pre-commit hook to block commits with wrong git identity.

Branch-and-env aware:
- On an agent-convention branch AND `$AGENT_NAME` set in the environment,
  enforce strict mode: require an agent-pattern email
  (`roland+*@ccom.unh.edu`) and reject the human patterns.
- Otherwise stay permissive (the pre-#468 behavior): accept any pattern
  in PERMISSIVE_ACCEPTED_PATTERNS so the human can edit interactively
  and field-mode hotfixes still work.

Patterns and the agent-branch regex live in `identity_patterns.py`.

Caveat (per #468 / plan): strict mode is gated on `$AGENT_NAME`, so an
agent committing in a subshell that lost the env var falls into
permissive mode — exactly the failure this hook tries to defeat. The CI
check (Mechanism C, `check_pr_authors.py`) is the load-bearing defense
against that case; this hook helps when the agent *does* have
`$AGENT_NAME` set.
"""

import os
import subprocess
import sys
from pathlib import Path

# Make identity_patterns importable when invoked as a pre-commit script.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from identity_patterns import (  # noqa: E402
    ACCEPTED_AGENT_PATTERNS,
    PERMISSIVE_ACCEPTED_PATTERNS,
    is_agent_branch,
    matches_any,
)


def get_commit_email():
    """Return the email that will be used for the next commit.

    GIT_AUTHOR_EMAIL takes precedence over git config user.email
    (matching git's own resolution order).
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


def get_current_branch():
    """Return the current branch name, or "" on detached HEAD / error."""
    try:
        result = subprocess.run(
            ["git", "symbolic-ref", "--short", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        )
        return result.stdout.strip()
    except subprocess.CalledProcessError:
        return ""


def strict_mode_active():
    """Return True if the strict gate fires: agent branch AND $AGENT_NAME set."""
    if not os.environ.get("AGENT_NAME", "").strip():
        return False
    return is_agent_branch(get_current_branch())


def main():
    email = get_commit_email()
    strict = strict_mode_active()
    accepted = ACCEPTED_AGENT_PATTERNS if strict else PERMISSIVE_ACCEPTED_PATTERNS

    if matches_any(email, accepted):
        return 0

    print()
    if strict:
        print("ERROR: Agent branch + $AGENT_NAME set — strict commit-identity mode.")
        print("       Commit must be authored under an agent email pattern.")
    else:
        print("ERROR: Commit identity is not configured or not recognized!")
    print()
    print(f"  Current email: {email or '(not set)'}")
    print(f"  AGENT_NAME:    {os.environ.get('AGENT_NAME') or '(not set)'}")
    print(f"  Branch:        {get_current_branch() or '(detached / unknown)'}")
    print(f"  Mode:          {'strict (agent-only)' if strict else 'permissive'}")
    print()
    print(f"  Accepted patterns: {', '.join(accepted)}")
    print()
    print("  To fix:")
    if strict:
        print('    git -c user.name="$AGENT_NAME" -c user.email="$AGENT_EMAIL" commit -m "…"')
        print("  Or:")
    print("    source .agent/scripts/set_git_identity_env.sh --detect")
    print("    source .agent/scripts/set_git_identity_env.sh --agent <framework>")
    print()
    return 1


if __name__ == "__main__":
    sys.exit(main())
