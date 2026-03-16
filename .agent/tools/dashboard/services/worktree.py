"""Parse worktree_list.sh --json output and correlate with tmux panes."""

import json
import os
import subprocess
import threading
import time

import services.tmux as tmux

# Session cache — populated by the SSE poller, read by terminal/context routes
_session_cache = []
_cache_lock = threading.Lock()
_cache_time = 0
_CACHE_TTL = 5  # seconds


def list_worktrees(workspace_root):
    """Run worktree_list.sh --json and return parsed data."""
    script = os.path.join(workspace_root, ".agent", "scripts", "worktree_list.sh")
    if not os.path.exists(script):
        return {"worktrees": [], "summary": {"total": 0, "layer": 0, "workspace": 0, "dirty": 0}}

    try:
        result = subprocess.run(
            [script, "--json"],
            capture_output=True,
            text=True,
            timeout=30,
            cwd=workspace_root,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return {"worktrees": [], "summary": {"total": 0, "layer": 0, "workspace": 0, "dirty": 0}}

    if result.returncode != 0:
        return {"worktrees": [], "summary": {"total": 0, "layer": 0, "workspace": 0, "dirty": 0}}

    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError:
        return {"worktrees": [], "summary": {"total": 0, "layer": 0, "workspace": 0, "dirty": 0}}


def discover_sessions(workspace_root):
    """Match worktrees to tmux panes by path overlap.

    Returns a list of session dicts, each with worktree info + optional pane_id.
    """
    wt_data = list_worktrees(workspace_root)
    panes = tmux.list_panes()

    sessions = []
    for wt in wt_data.get("worktrees", []):
        # Skip the main workspace entry
        if wt.get("type") == "main":
            continue

        wt_path = wt.get("path", "")

        # Find a tmux pane whose cwd is inside this worktree.
        # Use path-boundary check to avoid /issue-12 matching /issue-123.
        matching_pane = None
        wt_real = os.path.realpath(wt_path)
        for pane in panes:
            pane_real = os.path.realpath(pane["cwd"])
            if pane_real == wt_real or pane_real.startswith(wt_real + os.sep):
                matching_pane = pane
                break

        session_id = _make_session_id(wt)
        status = "inactive"
        if matching_pane:
            status = tmux.detect_status(matching_pane["pane_id"])

        sessions.append(
            {
                "id": session_id,
                "issue": wt.get("issue"),
                "skill": wt.get("skill"),
                "type": wt.get("type"),
                "path": wt_path,
                "branch": wt.get("branch"),
                "worktree_status": wt.get("status", "unknown"),
                "files_changed": wt.get("files_changed", 0),
                "repo": wt.get("repo"),
                "layer": wt.get("layer"),
                "pane_id": matching_pane["pane_id"] if matching_pane else None,
                "agent_status": status,
            }
        )

    _update_cache(sessions)
    return sessions


def get_sessions(workspace_root):
    """Return cached sessions, refreshing if stale.

    Used by terminal/context routes to avoid shelling out on every poll.
    """
    global _session_cache, _cache_time
    with _cache_lock:
        if time.time() - _cache_time < _CACHE_TTL and _session_cache:
            return list(_session_cache)
    # Cache miss — do a fresh discovery
    sessions = discover_sessions(workspace_root)
    return sessions


def _update_cache(sessions):
    """Update the session cache (called by discover_sessions)."""
    global _session_cache, _cache_time
    with _cache_lock:
        _session_cache = list(sessions)
        _cache_time = time.time()


def _make_session_id(wt):
    """Generate a stable session ID from worktree data."""
    if wt.get("issue"):
        return f"issue-{wt['issue']}"
    if wt.get("skill"):
        return f"skill-{wt['skill']}"
    # Fallback: use basename of path
    return os.path.basename(wt.get("path", "unknown"))
