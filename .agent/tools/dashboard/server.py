#!/usr/bin/env python3
"""Agent Dashboard — local web server for monitoring concurrent agent sessions.

Usage:
    python server.py [--port PORT] [--workspace PATH]

Zero dependencies: Python stdlib only.
"""

import argparse
import json
import mimetypes
import os
import re
import sys
import urllib.parse
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

# Ensure the dashboard package directory is on the path
_dashboard_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _dashboard_dir)


class DashboardHandler(BaseHTTPRequestHandler):
    """HTTP request handler for the dashboard API and static files."""

    # Set by the server instance
    workspace_root = None
    static_dir = None

    # Route patterns: (method, regex) -> handler
    # Populated in _setup_routes()
    _routes = []

    def do_GET(self):
        self._dispatch("GET")

    def do_POST(self):
        self._dispatch("POST")

    def do_OPTIONS(self):
        """Handle preflight (same-origin only, no CORS needed)."""
        self.send_response(204)
        self.end_headers()

    def _dispatch(self, method):
        """Route the request to the appropriate handler."""
        path = self.path.split("?")[0]  # strip query string

        for route_method, pattern, handler in self._routes:
            if method != route_method:
                continue
            match = pattern.match(path)
            if match:
                handler(self, **match.groupdict())
                return

        # Static files
        if method == "GET":
            self._serve_static(path)
            return

        self.send_error_json(404, f"Not found: {method} {path}")

    def _serve_static(self, path):
        """Serve static files from the static/ directory."""
        if path == "/":
            path = "/index.html"

        # Security: decode percent-encoding before normpath so that
        # /%2e%2e/.. sequences are caught by the startswith("..") check.
        safe_path = os.path.normpath(urllib.parse.unquote(path).lstrip("/"))
        if safe_path.startswith(".."):
            self.send_error_json(403, "Forbidden")
            return

        filepath = os.path.realpath(os.path.join(self.static_dir, safe_path))
        if not filepath.startswith(
            os.path.realpath(self.static_dir) + os.sep
        ) and filepath != os.path.realpath(self.static_dir):
            self.send_error_json(403, "Forbidden")
            return

        if not os.path.isfile(filepath):
            self.send_error_json(404, f"Not found: {path}")
            return

        content_type, _ = mimetypes.guess_type(filepath)
        if content_type is None:
            content_type = "application/octet-stream"

        with open(filepath, "rb") as f:
            content = f.read()

        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", len(content))
        self.end_headers()
        self.wfile.write(content)

    def send_json(self, data):
        """Send a JSON response."""
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", len(body))
        self.end_headers()
        self.wfile.write(body)

    def send_error_json(self, code, message):
        """Send a JSON error response."""
        body = json.dumps({"error": message}).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", len(body))
        self.end_headers()
        self.wfile.write(body)

    def read_body(self):
        """Read the request body."""
        length = int(self.headers.get("Content-Length", 0))
        return self.rfile.read(length).decode()

    def log_message(self, format, *args):
        """Suppress default access logging (too noisy with polling)."""
        pass

    def handle_error(self, request, client_address):
        """Suppress BrokenPipeError — common when browser closes connection early."""
        import traceback

        exc = sys.exc_info()[1]
        if isinstance(exc, BrokenPipeError):
            return
        traceback.print_exc()


def _setup_routes():
    """Register API route handlers."""
    from routes import sessions, terminal, context, plan, events

    return [
        ("GET", re.compile(r"^/api/sessions$"), lambda s, **kw: sessions.handle_get(s)),
        (
            "GET",
            re.compile(r"^/api/terminal/(?P<session_id>[^/]+)$"),
            lambda s, **kw: terminal.handle_get(s, kw["session_id"]),
        ),
        (
            "POST",
            re.compile(r"^/api/terminal/(?P<session_id>[^/]+)/send$"),
            lambda s, **kw: terminal.handle_send(s, kw["session_id"]),
        ),
        (
            "GET",
            re.compile(r"^/api/context/(?P<session_id>[^/]+)$"),
            lambda s, **kw: context.handle_get(s, kw["session_id"]),
        ),
        (
            "GET",
            re.compile(r"^/api/plan/(?P<session_id>[^/]+)$"),
            lambda s, **kw: plan.handle_get(s, kw["session_id"]),
        ),
        ("GET", re.compile(r"^/api/events$"), lambda s, **kw: events.handle_sse(s)),
    ]


def main():
    parser = argparse.ArgumentParser(description="Agent Dashboard Server")
    parser.add_argument("--port", type=int, default=3000, help="Port to listen on (default: 3000)")
    parser.add_argument(
        "--bind",
        type=str,
        default="127.0.0.1",
        help="Address to bind to (default: 127.0.0.1, use 0.0.0.0 for LAN access)",
    )
    parser.add_argument(
        "--workspace",
        type=str,
        default=None,
        help="Workspace root directory (default: auto-detect)",
    )
    args = parser.parse_args()

    # Auto-detect workspace root
    workspace_root = args.workspace
    if workspace_root is None:
        # Walk up from this script to find the workspace root
        candidate = os.path.dirname(os.path.abspath(__file__))
        for _ in range(10):
            if os.path.exists(os.path.join(candidate, "AGENTS.md")):
                workspace_root = candidate
                break
            candidate = os.path.dirname(candidate)

    if workspace_root is None:
        print("ERROR: Could not detect workspace root. Use --workspace.", file=sys.stderr)
        sys.exit(1)

    workspace_root = os.path.abspath(workspace_root)
    static_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")

    # Configure handler
    DashboardHandler.workspace_root = workspace_root
    DashboardHandler.static_dir = static_dir
    DashboardHandler._routes = _setup_routes()

    # Start SSE poller
    from routes.events import start_poller

    start_poller(workspace_root)

    # Start server
    server = ThreadingHTTPServer((args.bind, args.port), DashboardHandler)
    print(f"Agent Dashboard running at http://{args.bind}:{args.port}")
    print(f"Workspace: {workspace_root}")
    print("Press Ctrl+C to stop.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.")
        server.shutdown()


if __name__ == "__main__":
    main()
