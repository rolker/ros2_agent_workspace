#!/usr/bin/env python3
"""Browser tests for the Agent Dashboard using Playwright.

Tests JavaScript rendering, DOM interaction, SSE connection, and the D1
regression (href double-escaping in inlineFormat()).

Skipped automatically if playwright is not installed:
    pip install playwright && playwright install chromium

Or via workspace venv:
    .venv/bin/pip install playwright && .venv/bin/playwright install chromium

Run with:
    python -m unittest discover .agent/tools/dashboard/tests
    make test-dashboard
"""
import json
import os
import sys
import threading
import unittest
from http.server import ThreadingHTTPServer

try:
    from playwright.sync_api import sync_playwright

    PLAYWRIGHT_AVAILABLE = True
except ImportError:
    PLAYWRIGHT_AVAILABLE = False

# Add the dashboard package directory to sys.path
_dashboard_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _dashboard_dir not in sys.path:
    sys.path.insert(0, _dashboard_dir)

from server import DashboardHandler, _setup_routes  # noqa: E402

# Synthetic session returned by the patched get_sessions() so tab tests
# are independent of the host environment (no live worktrees or tmux needed).
_FAKE_SESSIONS = [
    {
        "id": "issue-workspace-999",
        "issue": 999,
        "skill": None,
        "type": "workspace",
        "path": None,
        "branch": "feature/issue-999",
        "worktree_status": "clean",
        "files_changed": 0,
        "repo": None,
        "layer": None,
        "pane_id": None,
        "agent_status": "inactive",
    }
]


def _find_workspace_root():
    candidate = os.path.dirname(os.path.abspath(__file__))
    for _ in range(15):
        if os.path.exists(os.path.join(candidate, "AGENTS.md")):
            return candidate
        candidate = os.path.dirname(candidate)
    return None


@unittest.skipUnless(
    PLAYWRIGHT_AVAILABLE,
    "playwright not installed — run: pip install playwright && playwright install chromium",
)
class TestDashboardBrowser(unittest.TestCase):
    """End-to-end browser tests against a live server on an ephemeral port."""

    server = None
    port = None
    _thread = None
    _pw = None
    browser = None

    @classmethod
    def setUpClass(cls):
        workspace_root = _find_workspace_root()
        if workspace_root is None:
            raise RuntimeError("Cannot find workspace root — AGENTS.md not found")

        static_dir = os.path.join(_dashboard_dir, "static")

        class _TestHandler(DashboardHandler):
            pass

        _TestHandler.workspace_root = workspace_root
        _TestHandler.static_dir = static_dir
        _TestHandler._routes = _setup_routes()

        # Patch worktree discovery so tab tests work on any machine without
        # requiring live worktrees or tmux (CI-safe).
        # addClassCleanup runs even when setUpClass raises (e.g. SkipTest from
        # missing Chromium), guaranteeing the patch is always reverted.
        import services.worktree as _wt

        orig = _wt.get_sessions
        cls.addClassCleanup(setattr, _wt, "get_sessions", orig)
        _wt.get_sessions = lambda root: list(_FAKE_SESSIONS)

        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), _TestHandler)
        cls.port = cls.server.server_address[1]
        cls._thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls._thread.start()

        cls._pw = sync_playwright().start()
        try:
            cls.browser = cls._pw.chromium.launch(headless=True)
        except Exception as exc:
            cls._pw.stop()
            cls._pw = None
            raise unittest.SkipTest(
                f"Chromium binaries not available — run: playwright install chromium ({exc})"
            )

    @classmethod
    def tearDownClass(cls):
        if cls.browser:
            cls.browser.close()
        if cls._pw:
            cls._pw.stop()
        if cls.server:
            cls.server.shutdown()
            cls.server.server_close()

    def setUp(self):
        self.page = self.browser.new_page()
        self.page.goto(f"http://127.0.0.1:{self.port}/")
        # Wait for DOM ready, then for the initial /api/sessions fetch to render
        # something into the tab bar (tabs or the "no sessions" placeholder).
        # Cannot use networkidle — the SSE connection is always open.
        self.page.wait_for_load_state("domcontentloaded", timeout=5000)
        self.page.wait_for_function(
            "document.getElementById('tab-bar').children.length > 0",
            timeout=8000,
        )

    def tearDown(self):
        self.page.close()

    # --- Page structure ---

    def test_page_title(self):
        self.assertIn("Agent Dashboard", self.page.title())

    def test_all_panels_present(self):
        for panel_id in ("#terminal-panel", "#plan-panel", "#context-panel"):
            self.assertTrue(
                self.page.locator(panel_id).count() > 0,
                f"Panel '{panel_id}' not found in DOM",
            )

    def test_send_bar_present(self):
        self.assertTrue(self.page.locator("#agent-input").is_visible())
        self.assertTrue(self.page.locator("#send-btn").is_visible())

    # --- Session tabs ---

    def test_tab_bar_renders_session_tabs(self):
        """At least one session tab is rendered from the API response."""
        self.page.wait_for_selector(".tab", timeout=5000)
        count = self.page.locator(".tab").count()
        self.assertGreater(count, 0, "Expected at least one session tab")

    def test_first_tab_auto_selected_on_load(self):
        """init() auto-selects the first session tab."""
        self.page.wait_for_selector(".tab.active", timeout=5000)
        active_count = self.page.locator(".tab.active").count()
        self.assertEqual(active_count, 1, "Exactly one tab should be active")

    def test_tab_labels_show_issue_number(self):
        """Workspace session tabs show '#N (ws)' label format."""
        self.page.wait_for_selector(".tab", timeout=5000)
        # Find a tab whose text matches the workspace label format
        tab_texts = self.page.locator(".tab").all_text_contents()
        workspace_tabs = [t for t in tab_texts if "(ws)" in t]
        self.assertGreater(
            len(workspace_tabs),
            0,
            f"Expected at least one '#N (ws)' tab. Got: {tab_texts[:5]}",
        )

    def test_tab_labels_contain_issue_number(self):
        """Tab labels contain a '#N' issue number."""
        self.page.wait_for_selector(".tab", timeout=5000)
        tab_texts = self.page.locator(".tab").all_text_contents()
        issue_tabs = [t for t in tab_texts if "#" in t]
        self.assertGreater(
            len(issue_tabs),
            0,
            f"Expected tabs with '#N' issue numbers. Got: {tab_texts[:5]}",
        )

    def test_only_agent_tabs_in_top_bar(self):
        """App sessions (type=app) do not appear as top-level tabs."""
        self.page.wait_for_selector(".tab", timeout=5000)
        # App sessions have labels without '#' and without '(ws)'/'(core)' etc.
        # The real check is that no tab has the 'app' data attribute,
        # but we can verify via the API that app session IDs don't appear as tabs.
        import urllib.request

        with urllib.request.urlopen(f"http://127.0.0.1:{self.port}/api/sessions") as resp:
            sessions = json.loads(resp.read())

        app_ids = {s["id"] for s in sessions if s["type"] == "app"}
        if not app_ids:
            self.skipTest("No app sessions to verify")

        tab_session_ids = self.page.evaluate(
            "() => [...document.querySelectorAll('.tab')].map(t => t.dataset.session)"
        )
        for app_id in app_ids:
            self.assertNotIn(
                app_id,
                tab_session_ids,
                f"App session '{app_id}' should not appear as top-level tab",
            )

    # --- Tab switching and panel loading ---

    def test_clicking_tab_makes_it_active(self):
        """Clicking a non-active tab switches the active state."""
        self.page.wait_for_selector(".tab", timeout=5000)
        tabs = self.page.locator(".tab").all()
        if len(tabs) < 2:
            self.skipTest("Need at least 2 tabs to test switching")
        # Click the second tab
        tabs[1].click()
        self.page.wait_for_timeout(200)
        active_id = self.page.evaluate(
            "() => document.querySelector('.tab.active')?.dataset.session"
        )
        second_id = tabs[1].get_attribute("data-session")
        self.assertEqual(active_id, second_id)

    def test_selecting_tab_populates_context_panel(self):
        """After selecting a tab, the context panel shows session data (not placeholder)."""
        self.page.wait_for_selector(".tab", timeout=5000)
        self.page.locator(".tab").first.click()
        self.page.wait_for_function(
            "!document.getElementById('context-content').innerText.includes('Select a session')",
            timeout=8000,
        )
        content = self.page.locator("#context-content").inner_text()
        self.assertNotEqual(content.strip(), "")

    def test_selecting_tab_populates_plan_panel(self):
        """After selecting a tab, the plan panel updates from 'Select a session'."""
        self.page.wait_for_selector(".tab", timeout=5000)
        self.page.locator(".tab").first.click()
        self.page.wait_for_function(
            "!document.getElementById('plan-content').innerText.includes('Select a session')",
            timeout=8000,
        )

    # --- SSE / connection status ---

    def test_connection_status_shows_connected(self):
        """Connection status indicator is 'connected' when server is reachable."""
        self.page.wait_for_selector("#connection-status.connected", timeout=5000)

    def test_sse_does_not_immediately_error(self):
        """SSE connection stays open — indicator remains connected after 1s."""
        self.page.wait_for_selector("#connection-status.connected", timeout=5000)
        self.page.wait_for_timeout(1200)
        css_class = self.page.locator("#connection-status").get_attribute("class") or ""
        self.assertIn("connected", css_class, "SSE stream appears to have errored out")

    # --- D1 regression: href double-escaping ---

    def test_d1_plan_links_not_double_escaped(self):
        """D1 regression: & in plan link URLs must not be double-escaped to &amp;amp;

        Before the fix, inlineFormat() called escapeHtml() on the full text first
        (converting & -> &amp;), then additionally did href.replace(/&/g, '&amp;'),
        producing &amp;amp; in the href attribute. This broke URLs with query params.
        """
        # Inject a plan containing a link with & in the query string
        test_plan = json.dumps(
            {
                "plan": {
                    "source": "test",
                    "content": (
                        "## Test Plan\n\n"
                        "See [results](https://example.com/search?a=1&b=2) for details.\n"
                        "Also [docs](https://example.com/page?x=foo&y=bar).\n"
                    ),
                }
            }
        )

        def intercept_plan(route):
            route.fulfill(status=200, content_type="application/json", body=test_plan)

        self.page.route("**/api/plan/**", intercept_plan)

        # Navigate fresh so the interceptor is active from the start
        self.page.goto(f"http://127.0.0.1:{self.port}/")
        self.page.wait_for_selector(".tab", timeout=5000)
        self.page.locator(".tab").first.click()

        # Wait for the link to appear in the plan panel
        self.page.wait_for_selector("#plan-content a", timeout=8000)

        hrefs = self.page.evaluate(
            "() => [...document.querySelectorAll('#plan-content a')]"
            ".map(a => a.getAttribute('href'))"
        )

        self.assertGreater(len(hrefs), 0, "No links rendered in plan content")
        for href in hrefs:
            self.assertIsNotNone(href, "Link href should not be null")
            self.assertNotIn(
                "&amp;",
                href,
                f"D1 regression: href contains HTML-escaped & (double-escaping): {href}",
            )
            self.assertIn(
                "&",
                href,
                f"href should contain a literal & for query params but got: {href}",
            )

    def test_plan_links_have_correct_url(self):
        """Plan links rendered from markdown preserve the full URL."""
        test_plan = json.dumps(
            {
                "plan": {
                    "source": "test",
                    "content": "See [GitHub](https://github.com/rolker/ros2_agent_workspace).",
                }
            }
        )

        self.page.route(
            "**/api/plan/**",
            lambda r: r.fulfill(status=200, content_type="application/json", body=test_plan),
        )

        self.page.goto(f"http://127.0.0.1:{self.port}/")
        self.page.wait_for_selector(".tab", timeout=5000)
        self.page.locator(".tab").first.click()
        self.page.wait_for_selector("#plan-content a", timeout=8000)

        href = self.page.locator("#plan-content a").first.get_attribute("href")
        self.assertEqual(href, "https://github.com/rolker/ros2_agent_workspace")

    def test_plan_links_open_in_new_tab(self):
        """Plan links have target=_blank and rel=noopener noreferrer."""
        test_plan = json.dumps(
            {
                "plan": {
                    "source": "test",
                    "content": "See [link](https://example.com/page).",
                }
            }
        )

        self.page.route(
            "**/api/plan/**",
            lambda r: r.fulfill(status=200, content_type="application/json", body=test_plan),
        )

        self.page.goto(f"http://127.0.0.1:{self.port}/")
        self.page.wait_for_selector(".tab", timeout=5000)
        self.page.locator(".tab").first.click()
        self.page.wait_for_selector("#plan-content a", timeout=8000)

        link = self.page.locator("#plan-content a").first
        self.assertEqual(link.get_attribute("target"), "_blank")
        rel = link.get_attribute("rel") or ""
        self.assertIn("noopener", rel)
        self.assertIn("noreferrer", rel)

    # --- Markdown rendering ---

    def test_plan_renders_markdown_bold(self):
        """**bold** in plan content renders as <strong>."""
        test_plan = json.dumps({"plan": {"source": "test", "content": "This is **important**."}})
        self.page.route(
            "**/api/plan/**",
            lambda r: r.fulfill(status=200, content_type="application/json", body=test_plan),
        )

        self.page.goto(f"http://127.0.0.1:{self.port}/")
        self.page.wait_for_selector(".tab", timeout=5000)
        self.page.locator(".tab").first.click()
        self.page.wait_for_function(
            "document.querySelector('#plan-content strong') !== null",
            timeout=8000,
        )
        text = self.page.locator("#plan-content strong").first.inner_text()
        self.assertEqual(text, "important")

    def test_plan_renders_headings(self):
        """## Heading in plan content renders as h2."""
        test_plan = json.dumps(
            {"plan": {"source": "test", "content": "## My Heading\n\nSome text."}}
        )
        self.page.route(
            "**/api/plan/**",
            lambda r: r.fulfill(status=200, content_type="application/json", body=test_plan),
        )

        self.page.goto(f"http://127.0.0.1:{self.port}/")
        self.page.wait_for_selector(".tab", timeout=5000)
        self.page.locator(".tab").first.click()
        self.page.wait_for_function(
            "document.querySelector('#plan-content h2') !== null",
            timeout=8000,
        )
        text = self.page.locator("#plan-content h2").first.inner_text()
        self.assertIn("My Heading", text)

    def test_plan_null_shows_fallback(self):
        """When plan API returns null, the plan panel shows a fallback message."""
        self.page.route(
            "**/api/plan/**",
            lambda r: r.fulfill(status=200, content_type="application/json", body='{"plan": null}'),
        )

        self.page.goto(f"http://127.0.0.1:{self.port}/")
        self.page.wait_for_selector(".tab", timeout=5000)
        self.page.locator(".tab").first.click()
        self.page.wait_for_function(
            "!document.getElementById('plan-content').innerText.includes('Select a session')",
            timeout=8000,
        )
        # Should show some fallback — either issue body or "no plan" message
        content = self.page.locator("#plan-content").inner_text()
        self.assertNotEqual(content.strip(), "")


if __name__ == "__main__":
    unittest.main()
