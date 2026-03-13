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
    """Try to get issue from git-bug."""
    try:
        result = subprocess.run(
            ["git", "bug", "show", str(issue_number)],
            capture_output=True,
            text=True,
            timeout=5,
            cwd=repo_dir,
        )
        if result.returncode == 0 and result.stdout.strip():
            # git-bug show outputs plain text; parse title from first line
            lines = result.stdout.strip().split("\n")
            title = lines[0] if lines else f"Issue #{issue_number}"
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
