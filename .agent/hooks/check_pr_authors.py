#!/usr/bin/env python3
"""CI check: reject PRs where an agent-convention branch has commits
authored under a human email (Mechanism C from issue #468).

The pre-commit hook `check-commit-identity.py` enforces the same rule
locally, but it depends on `$AGENT_NAME` being set in the commit
shell — which the very bug #468 describes (env vars not surviving
subshells) can defeat. This script is the load-bearing defense: it
runs in CI, has no env-var dependency, and operates only on the commit
authorship that GitHub records.

Usage:
    check_pr_authors.py <pr-number>

Exits 0 if:
- The PR head branch is not an agent-convention branch (nothing to
  check), or
- All commits have authors matching `ACCEPTED_AGENT_PATTERNS` (or
  github-bot author types that are explicitly allowlisted).

Exits 1 (and prints offending commits) otherwise.

Requires the `gh` CLI authenticated against the repository.
"""

import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from identity_patterns import (  # noqa: E402
    HUMAN_PATTERNS,
    is_agent_branch,
    matches_any,
)


def gh_json(args: list) -> object:
    """Run `gh` and parse the JSON output. Exits 2 on gh failure."""
    try:
        result = subprocess.run(
            ["gh", *args],
            capture_output=True,
            text=True,
            check=True,
        )
    except FileNotFoundError:
        print("ERROR: `gh` CLI not found on PATH.", file=sys.stderr)
        sys.exit(2)
    except subprocess.CalledProcessError as exc:
        print(f"ERROR: gh failed: {' '.join(['gh', *args])}", file=sys.stderr)
        print(exc.stderr, file=sys.stderr)
        sys.exit(2)
    return json.loads(result.stdout)


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <pr-number>", file=sys.stderr)
        return 2

    pr_number = sys.argv[1]

    # Pull PR metadata + commit list in one gh call (fewer round trips).
    pr_info = gh_json(
        [
            "pr",
            "view",
            pr_number,
            "--json",
            "headRefName,isCrossRepository,commits",
        ]
    )
    head_branch = pr_info.get("headRefName", "")
    is_fork = bool(pr_info.get("isCrossRepository"))
    commits = pr_info.get("commits", [])

    if not is_agent_branch(head_branch):
        print(
            f"PR #{pr_number} head branch '{head_branch}' is not an "
            "agent-convention branch — nothing to check."
        )
        return 0

    if is_fork:
        print(
            f"INFO: PR #{pr_number} is a cross-repository PR (fork head). "
            "Validating commit authorship anyway."
        )
    if not commits:
        print(f"WARNING: PR #{pr_number} has no commits to validate.")
        return 0

    offending = []
    for c in commits:
        sha = c.get("oid", "")[:7]
        # gh returns a list of authors per commit (GitHub coauthor model).
        # Reject if any author's email is in HUMAN_PATTERNS.
        for author in c.get("authors") or []:
            email = (author.get("email") or "").strip()
            name = (author.get("name") or "").strip()
            if matches_any(email, HUMAN_PATTERNS):
                offending.append((sha, name, email, c.get("messageHeadline", "")))

    if not offending:
        print(
            f"✅ PR #{pr_number}: {len(commits)} commit(s) on agent branch "
            f"'{head_branch}'; no human-authored commits."
        )
        return 0

    print(
        f"❌ PR #{pr_number}: {len(offending)} of {len(commits)} commit(s) "
        f"on agent branch '{head_branch}' are authored under a human email.",
        file=sys.stderr,
    )
    print(file=sys.stderr)
    print("Offending commits:", file=sys.stderr)
    for sha, name, email, headline in offending:
        print(f"  {sha} <{email}> {name} — {headline}", file=sys.stderr)
    print(file=sys.stderr)
    print(
        "Agent-convention branches (feature/issue-N, feature/ISSUE-N-desc,",
        file=sys.stderr,
    )
    print("skill/<name>-<timestamp>) must carry agent-authored commits only.", file=sys.stderr)
    print('See AGENTS.md "Agent Commit Identity" for the canonical pattern.', file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
