"""Issue data: git-bug first, gh CLI fallback."""

import json
import subprocess
import time


# Simple TTL cache: {key: (timestamp, value)}
_cache = {}
_CACHE_TTL = 60  # seconds


def _cached(key, ttl=_CACHE_TTL):
    """Return cached value if fresh, else None."""
    if key in _cache:
        ts, val = _cache[key]
        if time.time() - ts < ttl:
            return val
    return None


def _set_cache(key, value):
    _cache[key] = (time.time(), value)


def get_issue(issue_number, repo_dir=None):
    """Fetch issue summary. Tries git-bug first, falls back to gh CLI.

    Returns dict with: title, body, labels, state, url (or None on failure).
    """
    cache_key = f"issue:{issue_number}:{repo_dir}"
    cached = _cached(cache_key)
    if cached is not None:
        return cached

    # Try git-bug first
    result = _try_gitbug(issue_number, repo_dir)
    if result is None:
        result = _try_gh(issue_number, repo_dir)

    if result is not None:
        _set_cache(cache_key, result)
    return result


def _try_gitbug(issue_number, repo_dir):
    """Try to get issue from git-bug (local-first, offline-capable).

    Uses git bug select + show + deselect — the correct pattern for looking up
    a bug by issue number. 'git bug show <N>' is not valid; select sets the
    current bug context first.
    """
    try:
        # Select the bug by issue number
        select = subprocess.run(
            ["git", "bug", "select", str(issue_number)],
            capture_output=True,
            text=True,
            timeout=5,
            cwd=repo_dir,
        )
        if select.returncode != 0:
            return None

        # Show the selected bug
        show = subprocess.run(
            ["git", "bug", "show"],
            capture_output=True,
            text=True,
            timeout=5,
            cwd=repo_dir,
        )

        # Always deselect to avoid leaving side effects
        subprocess.run(
            ["git", "bug", "deselect"],
            capture_output=True,
            timeout=5,
            cwd=repo_dir,
        )

        if show.returncode == 0 and show.stdout.strip():
            lines = show.stdout.strip().split("\n")
            # First line format: "<id-prefix> <title>" — strip the leading ID token
            first_line = lines[0] if lines else ""
            parts = first_line.split(" ", 1)
            title = parts[1] if len(parts) > 1 else first_line
            body = "\n".join(lines[1:]) if len(lines) > 1 else ""
            return {
                "title": title,
                "body": body,
                "labels": [],
                "state": "open",
                "url": None,
                "source": "git-bug",
            }
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass
    return None


def _try_gh(issue_number, repo_dir):
    """Get issue from GitHub CLI."""
    try:
        result = subprocess.run(
            ["gh", "issue", "view", str(issue_number), "--json", "title,body,labels,state,url"],
            capture_output=True,
            text=True,
            timeout=10,
            cwd=repo_dir,
        )
        if result.returncode == 0:
            data = json.loads(result.stdout)
            data["source"] = "github"
            # Normalize labels to list of strings
            if "labels" in data and data["labels"]:
                data["labels"] = [
                    lbl.get("name", str(lbl)) if isinstance(lbl, dict) else str(lbl)
                    for lbl in data["labels"]
                ]
            return data
    except (subprocess.TimeoutExpired, FileNotFoundError, json.JSONDecodeError):
        pass
    return None
