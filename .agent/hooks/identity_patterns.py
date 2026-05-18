"""Shared identity patterns for agent-vs-human commit-author checks.

Single source of truth for:
- Accepted agent email patterns (any +suffix on roland@ccom.unh.edu)
- Human email patterns (Roland's bare addresses)
- Agent-branch regexes (feature/issue-N and skill/<name>-<timestamp>)

Consumers:
- .agent/hooks/check-commit-identity.py — pre-commit hook (Mechanism A)
- .agent/hooks/check_pr_authors.py     — CI script (Mechanism C)
- .agent/hooks/verify-issue-branch.py  — pre-commit hook (existing, uses
  the same feature/issue regex to extract issue numbers)

Keeping these patterns and regexes here prevents drift between layers.
"""

import fnmatch
import re

# Emails accepted as agent identities — any +suffix before @ccom.unh.edu.
# Examples: roland+claude-code@ccom.unh.edu, roland+copilot-cli@ccom.unh.edu.
ACCEPTED_AGENT_PATTERNS = [
    "roland+*@ccom.unh.edu",
]

# Roland's human email addresses. Accepted in human-mode contexts
# (AGENT_NAME unset, non-agent branch, detached HEAD, field-mode hotfix)
# and rejected in strict-mode contexts (agent branch + AGENT_NAME set).
HUMAN_PATTERNS = [
    "roland@ccom.unh.edu",
    "roland@rolker.net",
]

# Permissive accepted set — what check-commit-identity.py accepts when
# the strict-mode gate doesn't fire. Matches the pre-#468 behavior.
PERMISSIVE_ACCEPTED_PATTERNS = ACCEPTED_AGENT_PATTERNS + HUMAN_PATTERNS

# Agent branch convention regex (mirrors verify-issue-branch.py's
# historical regex). Case-insensitive on the literal "ISSUE" portion
# to accept both `feature/issue-N` and `feature/ISSUE-N-desc`.
FEATURE_ISSUE_BRANCH_RE = re.compile(r"^feature/[iI][sS][sS][uU][eE]-(\d+)")

# Skill-worktree branch convention from AGENTS.md:
# skill/{name}-{YYYYMMDD-HHMMSS-NNNNNNNNN}
SKILL_BRANCH_RE = re.compile(r"^skill/[^/]+-\d{8}-\d{6}-\d{9}$")


def matches_any(email: str, patterns: list) -> bool:
    """Return True if email matches any fnmatch pattern in patterns."""
    if not email:
        return False
    return any(fnmatch.fnmatch(email, p) for p in patterns)


def is_agent_branch(branch_name: str) -> bool:
    """Return True if branch matches the agent-branch convention.

    Recognizes:
    - feature/issue-N or feature/ISSUE-N-description (issue branches)
    - skill/<name>-YYYYMMDD-HHMMSS-NNNNNNNNN (skill-worktree branches)

    An empty / missing branch name (detached HEAD) returns False so the
    caller falls through to permissive mode rather than failing closed.
    """
    if not branch_name:
        return False
    return bool(FEATURE_ISSUE_BRANCH_RE.match(branch_name)) or bool(
        SKILL_BRANCH_RE.match(branch_name)
    )
