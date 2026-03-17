"""GET /api/terminal/:id — capture terminal output.
POST /api/terminal/:id/send — send text to agent."""

import json


def handle_get(server, session_id):
    """Return captured terminal output for a session."""
    from services import worktree, tmux

    sessions = worktree.get_sessions(server.workspace_root)
    session = worktree.find_session(sessions, session_id)

    if session is None:
        server.send_error_json(404, f"Session '{session_id}' not found")
        return

    if session["pane_id"] is None:
        server.send_json({"output": "", "status": "inactive"})
        return

    output = tmux.capture_pane(session["pane_id"])
    server.send_json(
        {
            "output": output,
            "status": session["agent_status"],
        }
    )


def handle_send(server, session_id):
    """Send text to the agent's tmux pane."""
    from services import worktree, tmux

    body = server.read_body()
    try:
        data = json.loads(body)
    except json.JSONDecodeError:
        server.send_error_json(400, "Invalid JSON")
        return

    text = data.get("text", "")
    if not text:
        server.send_error_json(400, "Missing 'text' field")
        return

    sessions = worktree.get_sessions(server.workspace_root)
    session = worktree.find_session(sessions, session_id)

    if session is None:
        server.send_error_json(404, f"Session '{session_id}' not found")
        return

    if session.get("type") == "app":
        server.send_error_json(400, "Cannot send input to app sessions (read-only)")
        return

    if session["pane_id"] is None:
        server.send_error_json(400, "Session has no active tmux pane")
        return

    ok = tmux.send_keys(session["pane_id"], text)
    server.send_json({"ok": ok})
