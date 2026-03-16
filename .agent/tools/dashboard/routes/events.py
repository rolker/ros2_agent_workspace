"""GET /api/events — Server-Sent Events stream for real-time updates."""

import json
import sys
import threading
import time


# Ring buffer for events (shared across all SSE connections)
_events = []
_events_lock = threading.Lock()
_MAX_EVENTS = 500
_next_id = 0


def push_event(event_type, data):
    """Push an event to all SSE listeners."""
    global _next_id
    with _events_lock:
        _events.append(
            {
                "id": _next_id,
                "type": event_type,
                "data": json.dumps(data),
            }
        )
        _next_id += 1
        # Trim old events
        while len(_events) > _MAX_EVENTS:
            _events.pop(0)


def handle_sse(server):
    """Stream SSE events to the client.

    The connection stays open; events are pushed as they arrive.
    """
    server.send_response(200)
    server.send_header("Content-Type", "text/event-stream")
    server.send_header("Cache-Control", "no-cache")
    server.send_header("Connection", "keep-alive")
    server.end_headers()

    # Parse Last-Event-ID from client (for reconnection)
    last_id = -1
    last_event_id = server.headers.get("Last-Event-ID")
    if last_event_id is not None:
        try:
            last_id = int(last_event_id)
        except ValueError:
            pass

    try:
        while True:
            new_events = []
            with _events_lock:
                new_events = [e for e in _events if e["id"] > last_id]

            for event in new_events:
                msg = f"id: {event['id']}\nevent: {event['type']}\ndata: {event['data']}\n\n"
                server.wfile.write(msg.encode())
                server.wfile.flush()
                last_id = event["id"]

            time.sleep(1)
    except (BrokenPipeError, ConnectionResetError):
        # Client disconnected
        pass


def start_poller(workspace_root, interval=10):
    """Background thread that polls session status and pushes SSE events."""
    from services import worktree

    previous_statuses = {}
    first_poll = True

    def poll_loop():
        nonlocal previous_statuses, first_poll
        while True:
            try:
                sessions = worktree.discover_sessions(workspace_root)
                current = {}
                for session in sessions:
                    sid = session["id"]
                    current[sid] = session["agent_status"]

                    if first_poll:
                        # Suppress session_added events on the initial poll
                        # to avoid a burst of redundant refreshSessions() calls
                        continue

                    # Detect status changes
                    prev = previous_statuses.get(sid)
                    if prev is not None and prev != session["agent_status"]:
                        push_event(
                            "status_change",
                            {
                                "session": sid,
                                "status": session["agent_status"],
                                "previous": prev,
                            },
                        )
                    elif prev is None:
                        # New session appeared
                        push_event(
                            "session_added",
                            {
                                "session": sid,
                                "issue": session["issue"],
                                "status": session["agent_status"],
                            },
                        )

                # Detect removed sessions
                for sid in previous_statuses:
                    if sid not in current:
                        push_event("session_removed", {"session": sid})

                previous_statuses = current
                first_poll = False
            except Exception as exc:
                print(f"[sse-poller] error: {exc}", file=sys.stderr)

            time.sleep(interval)

    thread = threading.Thread(target=poll_loop, daemon=True, name="sse-poller")
    thread.start()
    return thread
