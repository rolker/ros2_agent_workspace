#!/usr/bin/env python3
"""Unit tests for dashboard service logic.

Tests pure Python functions that don't require a running server, subprocess
calls, or network access. Fast and safe to run in any environment.

Run with:
    python -m unittest discover .agent/tools/dashboard/tests
"""
import os
import re
import sys
import unittest

# Add the dashboard package directory to sys.path
_dashboard_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _dashboard_dir not in sys.path:
    sys.path.insert(0, _dashboard_dir)

from services.worktree import _make_session_id  # noqa: E402


class TestMakeSessionId(unittest.TestCase):
    """Tests for _make_session_id() format and collision resistance."""

    def _wt(self, **kwargs):
        """Return a minimal worktree dict with sensible defaults."""
        base = {
            "type": "workspace",
            "issue": None,
            "skill": None,
            "repo": "workspace",
            "layer": None,
            "path": "/tmp/test-worktree",
        }
        base.update(kwargs)
        return base

    # --- format ---

    def test_workspace_issue_id(self):
        wt = self._wt(type="workspace", issue=42, repo="workspace")
        self.assertEqual(_make_session_id(wt), "issue-workspace-workspace-42")

    def test_layer_issue_id(self):
        wt = self._wt(type="layer", issue=42, repo="unh_marine_autonomy", layer="core")
        self.assertEqual(_make_session_id(wt), "issue-layer-unh_marine_autonomy-42")

    def test_skill_id_uses_path_basename(self):
        """Skill session ID uses path basename for disambiguation."""
        wt = self._wt(
            issue=None, skill="research", repo=None, path="/tmp/skill-research-20260317-090000"
        )
        self.assertEqual(_make_session_id(wt), "skill-skill-research-20260317-090000")

    def test_fallback_uses_path_basename(self):
        """No issue and no skill: falls back to the basename of path."""
        wt = self._wt(issue=None, skill=None, repo=None, path="/some/dir/my-session")
        self.assertEqual(_make_session_id(wt), "my-session")

    def test_id_starts_with_issue_for_issue_worktrees(self):
        for wt_type in ("workspace", "layer"):
            with self.subTest(type=wt_type):
                wt = self._wt(type=wt_type, issue=1, repo="some_repo")
                self.assertTrue(
                    _make_session_id(wt).startswith("issue-"),
                    f"ID should start with 'issue-' for {wt_type} worktrees",
                )

    def test_id_contains_no_spaces(self):
        """Session IDs must be URL-safe (no spaces)."""
        cases = [
            self._wt(type="workspace", issue=1, repo="workspace"),
            self._wt(type="layer", issue=2, repo="unh_marine_autonomy", layer="core"),
            self._wt(issue=None, skill="research", repo=None),
        ]
        for wt in cases:
            sid = _make_session_id(wt)
            self.assertNotIn(" ", sid, f"Session ID '{sid}' contains spaces")

    def test_id_is_string(self):
        for wt in [
            self._wt(type="workspace", issue=1, repo="workspace"),
            self._wt(issue=None, skill="research"),
            self._wt(issue=None, skill=None, repo=None, path="/tmp/x"),
        ]:
            self.assertIsInstance(_make_session_id(wt), str)

    # --- collision resistance ---

    def test_workspace_and_layer_same_issue_are_different(self):
        """Workspace and layer worktrees for the same issue must not collide."""
        ws = self._wt(type="workspace", issue=42, repo="workspace")
        layer = self._wt(type="layer", issue=42, repo="unh_marine_autonomy", layer="core")
        self.assertNotEqual(_make_session_id(ws), _make_session_id(layer))

    def test_different_issues_same_repo_are_different(self):
        wt1 = self._wt(type="workspace", issue=10, repo="workspace")
        wt2 = self._wt(type="workspace", issue=11, repo="workspace")
        self.assertNotEqual(_make_session_id(wt1), _make_session_id(wt2))

    def test_same_issue_different_repos_are_different(self):
        """Layer worktrees in different repos for the same issue must not collide."""
        wt1 = self._wt(type="layer", issue=5, repo="repo_a", layer="core")
        wt2 = self._wt(type="layer", issue=5, repo="repo_b", layer="core")
        self.assertNotEqual(_make_session_id(wt1), _make_session_id(wt2))

    def test_skill_worktrees_different_paths_are_unique(self):
        """Two skill worktrees with the same name but different paths have unique IDs."""
        wt1 = self._wt(issue=None, skill="research", path="/tmp/skill-research-20260301-120000")
        wt2 = self._wt(issue=None, skill="research", path="/tmp/skill-research-20260317-090000")
        self.assertNotEqual(_make_session_id(wt1), _make_session_id(wt2))

    def test_many_unique_workspace_sessions(self):
        """Bulk check: 20 workspace worktrees for different issues all get unique IDs."""
        ids = [
            _make_session_id(self._wt(type="workspace", issue=i, repo="workspace"))
            for i in range(1, 21)
        ]
        self.assertEqual(len(ids), len(set(ids)), "Duplicate IDs found in bulk workspace sessions")

    def test_many_unique_layer_sessions(self):
        """Bulk check: same issue across 10 different repos all get unique IDs."""
        ids = [
            _make_session_id(self._wt(type="layer", issue=42, repo=f"repo_{i}", layer="core"))
            for i in range(10)
        ]
        self.assertEqual(len(ids), len(set(ids)), "Duplicate IDs found across repos")


class TestAppSessionPattern(unittest.TestCase):
    """Tests for the app session name regex used by discover_app_sessions()."""

    _PATTERN = re.compile(r"^issue-(\d+)-(.+)$")

    def test_matches_simple_label(self):
        m = self._PATTERN.match("issue-42-myapp")
        self.assertIsNotNone(m)
        self.assertEqual(m.group(1), "42")
        self.assertEqual(m.group(2), "myapp")

    def test_matches_hyphenated_label(self):
        m = self._PATTERN.match("issue-1-my-app-with-dashes")
        self.assertIsNotNone(m)
        self.assertEqual(m.group(2), "my-app-with-dashes")

    def test_rejects_non_numeric_issue(self):
        self.assertIsNone(self._PATTERN.match("issue-notanumber-label"))

    def test_rejects_feature_branch_format(self):
        self.assertIsNone(self._PATTERN.match("feature/issue-42"))

    def test_rejects_plain_tmux_session_name(self):
        self.assertIsNone(self._PATTERN.match("izzyboat"))

    def test_rejects_issue_without_label(self):
        # "issue-42" with no trailing label should not match
        self.assertIsNone(self._PATTERN.match("issue-42"))


if __name__ == "__main__":
    unittest.main()
