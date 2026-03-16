"""GET /api/plan/:id — work plan for a session."""


def handle_get(server, session_id):
    """Return the work plan markdown for a session."""
    from services import worktree, workspace

    sessions = worktree.get_sessions(server.workspace_root)
    session = _find_session(sessions, session_id)

    if session is None:
        server.send_error_json(404, f"Session '{session_id}' not found")
        return

    # App sessions have no worktree path — no plan to look up
    if session.get("type") == "app" or session["path"] is None:
        server.send_json({"plan": None})
        return

    plan = workspace.get_work_plan(session["issue"], session["path"], server.workspace_root)

    if plan is None:
        server.send_json({"plan": None})
        return

    server.send_json(
        {
            "plan": {
                "source": plan["source"],
                "content": plan["content"],
            }
        }
    )


def _find_session(sessions, session_id):
    for s in sessions:
        if s["id"] == session_id:
            return s
    return None
