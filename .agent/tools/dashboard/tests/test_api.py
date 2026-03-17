#!/usr/bin/env python3
"""Integration tests for the dashboard HTTP API.

Uses an ephemeral port (port=0) so tests never conflict with a running
dashboard instance on port 3000 or any other fixed port. The server is
started once per test class and shut down in tearDownClass.

Run with:
    python -m unittest discover .agent/tools/dashboard/tests
    python -m pytest .agent/tools/dashboard/tests/
"""
import json
import os
import sys
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

# Add the dashboard package directory to sys.path before importing server
_dashboard_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _dashboard_dir not in sys.path:
    sys.path.insert(0, _dashboard_dir)

from server import DashboardHandler, _setup_routes  # noqa: E402


def _find_workspace_root():
    """Walk up from this file to find the workspace root (contains AGENTS.md)."""
    candidate = os.path.dirname(os.path.abspath(__file__))
    for _ in range(15):
        if os.path.exists(os.path.join(candidate, "AGENTS.md")):
            return candidate
        candidate = os.path.dirname(candidate)
    return None


class TestDashboardAPI(unittest.TestCase):
    """Integration tests against a live server on an ephemeral port."""

    server = None
    port = None
    _thread = None

    @classmethod
    def setUpClass(cls):
        workspace_root = _find_workspace_root()
        if workspace_root is None:
            raise RuntimeError("Cannot find workspace root — AGENTS.md not found")

        static_dir = os.path.join(_dashboard_dir, "static")

        # Subclass so test-specific class attributes don't pollute DashboardHandler
        class _TestHandler(DashboardHandler):
            pass

        _TestHandler.workspace_root = workspace_root
        _TestHandler.static_dir = static_dir
        _TestHandler._routes = _setup_routes()

        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), _TestHandler)
        cls.port = cls.server.server_address[1]
        cls._thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls._thread.start()

    @classmethod
    def tearDownClass(cls):
        if cls.server:
            cls.server.shutdown()
            cls.server.server_close()

    # --- helpers ---

    def _url(self, path):
        return f"http://127.0.0.1:{self.port}{path}"

    def _get_json(self, path):
        """GET a JSON endpoint. Returns (status_code, parsed_body)."""
        with urllib.request.urlopen(self._url(path)) as resp:
            return resp.status, json.loads(resp.read())

    def _get_raw(self, path):
        """GET an endpoint. Returns (status_code, headers_dict, body_bytes)."""
        with urllib.request.urlopen(self._url(path)) as resp:
            return resp.status, dict(resp.headers), resp.read()

    def _get_status(self, path):
        """GET an endpoint; return only the HTTP status code (handles 4xx/5xx)."""
        try:
            with urllib.request.urlopen(self._url(path)) as resp:
                return resp.status
        except urllib.error.HTTPError as exc:
            return exc.code

    def _first_session(self):
        """Return the first session from /api/sessions, or skip the test."""
        _, sessions = self._get_json("/api/sessions")
        if not sessions:
            self.skipTest("No sessions in workspace")
        return sessions[0]

    # --- /api/sessions ---

    def test_sessions_returns_200_list(self):
        status, data = self._get_json("/api/sessions")
        self.assertEqual(status, 200)
        self.assertIsInstance(data, list)

    def test_sessions_required_keys(self):
        _, sessions = self._get_json("/api/sessions")
        if not sessions:
            self.skipTest("No sessions in workspace")
        required = {
            "id",
            "issue",
            "type",
            "branch",
            "worktree_status",
            "files_changed",
            "agent_status",
            "pane_id",
        }
        for key in required:
            self.assertIn(key, sessions[0], f"Missing key '{key}' in session")

    def test_sessions_excludes_main_worktree(self):
        _, sessions = self._get_json("/api/sessions")
        types = {s["type"] for s in sessions}
        self.assertNotIn("main", types, "Main worktree must not appear in sessions list")

    def test_sessions_ids_are_unique(self):
        _, sessions = self._get_json("/api/sessions")
        ids = [s["id"] for s in sessions]
        self.assertEqual(len(ids), len(set(ids)), "Session IDs must be unique")

    def test_sessions_issue_ids_start_with_issue(self):
        _, sessions = self._get_json("/api/sessions")
        for s in sessions:
            if s["type"] in ("workspace", "layer"):
                self.assertRegex(
                    s["id"],
                    r"^issue-",
                    f"Session ID '{s['id']}' (type={s['type']}) should start with 'issue-'",
                )

    def test_sessions_files_changed_is_non_negative(self):
        _, sessions = self._get_json("/api/sessions")
        for s in sessions:
            self.assertGreaterEqual(
                s["files_changed"],
                0,
                f"Session '{s['id']}' has negative files_changed",
            )

    # --- /api/terminal/:id ---

    def test_terminal_unknown_session_returns_404(self):
        code = self._get_status("/api/terminal/no-such-session-xyz-99999")
        self.assertEqual(code, 404)

    def test_terminal_inactive_session_returns_empty_output(self):
        """Sessions with no tmux pane return empty output and 'inactive' status."""
        _, sessions = self._get_json("/api/sessions")
        inactive = next((s for s in sessions if s["pane_id"] is None), None)
        if inactive is None:
            self.skipTest("All sessions have tmux panes")
        _, data = self._get_json(f"/api/terminal/{inactive['id']}")
        self.assertEqual(data.get("output"), "")
        self.assertEqual(data.get("status"), "inactive")

    def test_terminal_response_has_output_and_status_keys(self):
        session = self._first_session()
        _, data = self._get_json(f"/api/terminal/{session['id']}")
        self.assertIn("output", data)
        self.assertIn("status", data)

    # --- /api/plan/:id ---

    def test_plan_unknown_session_returns_404(self):
        code = self._get_status("/api/plan/no-such-session-xyz-99999")
        self.assertEqual(code, 404)

    def test_plan_known_session_has_plan_key(self):
        session = self._first_session()
        _, data = self._get_json(f"/api/plan/{session['id']}")
        self.assertIn("plan", data)

    def test_plan_non_null_has_source_and_content(self):
        _, sessions = self._get_json("/api/sessions")
        for s in sessions:
            _, data = self._get_json(f"/api/plan/{s['id']}")
            if data.get("plan") is not None:
                self.assertIn("source", data["plan"])
                self.assertIn("content", data["plan"])
                return
        # No plan found in any session — that's fine, just check schema on None
        self.skipTest("No sessions have a non-null plan")

    # --- /api/context/:id ---

    def test_context_unknown_session_returns_404(self):
        code = self._get_status("/api/context/no-such-session-xyz-99999")
        self.assertEqual(code, 404)

    def test_context_known_session_has_required_keys(self):
        session = self._first_session()
        _, data = self._get_json(f"/api/context/{session['id']}")
        for key in ("session", "issue", "test_summary", "build_report", "changed_files"):
            self.assertIn(key, data, f"Missing key '{key}' in context response")

    def test_context_session_id_matches_request(self):
        session = self._first_session()
        _, data = self._get_json(f"/api/context/{session['id']}")
        self.assertEqual(data["session"]["id"], session["id"])

    def test_context_changed_files_is_list(self):
        session = self._first_session()
        _, data = self._get_json(f"/api/context/{session['id']}")
        self.assertIsInstance(data["changed_files"], list)

    # --- static files ---

    def test_static_index_returns_html(self):
        status, headers, _ = self._get_raw("/")
        self.assertEqual(status, 200)
        self.assertIn("text/html", headers.get("Content-Type", ""))

    def test_static_style_css(self):
        status, headers, _ = self._get_raw("/style.css")
        self.assertEqual(status, 200)
        self.assertIn("text/css", headers.get("Content-Type", ""))

    def test_static_app_js(self):
        status, headers, _ = self._get_raw("/app.js")
        self.assertEqual(status, 200)
        self.assertIn("javascript", headers.get("Content-Type", ""))

    def test_static_missing_file_returns_404(self):
        code = self._get_status("/nonexistent-file-xyz.txt")
        self.assertEqual(code, 404)

    def test_path_traversal_is_blocked(self):
        # Simple traversal — normpath detects the .. prefix → 403
        code = self._get_status("/../../../etc/passwd")
        self.assertIn(code, (403, 404), "Directory traversal should be blocked")


if __name__ == "__main__":
    unittest.main()
