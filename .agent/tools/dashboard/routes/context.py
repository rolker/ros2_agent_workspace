"""GET /api/context/:id — issue summary, test/build results, changed files."""


def handle_get(server, session_id):
    """Return context data for a session."""
    from services import worktree, issues, workspace

    sessions = worktree.discover_sessions(server.workspace_root)
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

    # Issue data
    if session["issue"] is not None:
        ctx["issue"] = issues.get_issue(session["issue"], server.workspace_root)

    # Test summary
    ctx["test_summary"] = workspace.get_test_summary(session["path"])

    # Build report (raw markdown)
    ctx["build_report"] = workspace.get_build_report(session["path"])

    # Changed files
    ctx["changed_files"] = workspace.get_changed_files(session["path"])

    server.send_json(ctx)


def _find_session(sessions, session_id):
    for s in sessions:
        if s["id"] == session_id:
            return s
    return None
