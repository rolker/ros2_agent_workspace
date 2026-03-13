"""GET /api/sessions — list all discovered sessions."""


def handle_get(server):
    """Return all sessions as JSON."""
    from services import worktree

    sessions = worktree.discover_sessions(server.workspace_root)
    server.send_json(sessions)
