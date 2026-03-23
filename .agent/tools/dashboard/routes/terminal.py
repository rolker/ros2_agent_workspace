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
    # Re-detect status live so the badge matches the captured output rather
    # than the potentially 15s-stale cached value.
    status = tmux.detect_status(session["pane_id"])
    server.send_json({"output": output, "status": status})


def handle_send(server, session_id):
    """Send text to the agent's tmux pane."""
    from services import worktree, tmux

    # CSRF guard: reject cross-origin POST requests from browsers.
    # Non-simple requests (application/json) are already blocked by CORS preflight,
    # but a simple-request form POST (enctype=text/plain) can still reach this endpoint.
    # Note: non-browser clients (curl, scripts) do not send Origin and are not blocked
    # here; when bound to 127.0.0.1 (the default) they are assumed to be the local user.
    # When bound to 0.0.0.0, the UI may be accessed via a LAN IP; same-origin requests
    # carry Origin: http://<lan-ip>:<port> which we allow by matching against Host.
    origin = server.headers.get("Origin", "")
    host = server.headers.get("Host", "")
    if origin:
        allowed = (
            origin.startswith("http://127.0.0.1")
            or origin.startswith("http://localhost")
            or (host and origin == f"http://{host}")
        )
        if not allowed:
            server.send_error_json(403, "Forbidden: cross-origin requests not allowed")
            return
    else:
        # No Origin header — non-browser clients (curl, scripts) or crafted requests.
        # Allow when bound to loopback; reject when Host indicates a LAN interface.
        if host and not host.startswith("127.0.0.1") and not host.startswith("localhost"):
            server.send_error_json(403, "Forbidden: missing Origin on non-loopback interface")
            return

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
    if not ok:
        server.send_error_json(500, "Failed to send keys to tmux pane")
        return
    server.send_json({"ok": True})
