"""GET /api/context/:id — issue summary, test/build results, changed files."""

import os


def handle_get(server, session_id):
    """Return context data for a session."""
    from services import worktree, issues, workspace

    sessions = worktree.get_sessions(server.workspace_root)
    session = _find_session(sessions, session_id)

    if session is None:
        server.send_error_json(404, f"Session '{session_id}' not found")
        return

    ctx = {
        "session": {
            "id": session["id"],
            "issue": session["issue"],
            "skill": session["skill"],
            "type": session["type"],
            "branch": session["branch"],
            "worktree_status": session["worktree_status"],
            "files_changed": session["files_changed"],
            "agent_status": session["agent_status"],
            "path": session["path"],
        },
        "issue": None,
        "test_summary": None,
        "build_report": None,
        "changed_files": [],
    }

    # Issue data — for layer worktrees, resolve the project repo directory
    # so `gh issue view` queries the correct GitHub repo.
    if session["issue"] is not None:
        issue_cwd = _resolve_issue_repo(session, server.workspace_root)
        ctx["issue"] = issues.get_issue(session["issue"], issue_cwd)

    # Test summary
    ctx["test_summary"] = workspace.get_test_summary(session["path"])

    # Build report (raw markdown)
    ctx["build_report"] = workspace.get_build_report(session["path"])

    # Changed files
    ctx["changed_files"] = workspace.get_changed_files(session["path"])

    server.send_json(ctx)


def _resolve_issue_repo(session, workspace_root):
    """Find the git repo directory to use for issue lookups.

    For layer worktrees, the issue belongs to the project repo (e.g.,
    unh_marine_simulation), not the workspace repo. The project repo
    is at <worktree>/<layer>_ws/src/<repo>/.
    """
    if session["type"] == "layer" and session.get("repo") and session.get("layer"):
        candidate = os.path.join(session["path"], f"{session['layer']}_ws", "src", session["repo"])
        if os.path.isdir(candidate):
            return candidate
    return workspace_root


def _find_session(sessions, session_id):
    for s in sessions:
        if s["id"] == session_id:
            return s
    return None
