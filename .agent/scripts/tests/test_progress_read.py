"""Tests for progress_read.py.

Unit tests for parse_progress() plus a subprocess smoke test of the CLI.
Run with: python3 -m unittest test_progress_read  (from this directory)
"""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

# Import the module under test (mirrors test_build_report_generator.py).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from progress_read import parse_progress, _matches_type  # noqa: E402

SCRIPT = str(Path(__file__).resolve().parent.parent / "progress_read.py")

# A multi-entry fixture exercising every correlation-key form and the
# field-ordering / offset variants. The PR entry deliberately places a
# skill-specific **Verdict** field *before* the canonical **PR** field
# (ADR-0013: consumers locate by name, not line offset). It also mixes a
# `**Branch**` correlation entry and a `Z`-offset timestamp.
GOLDEN = """\
---
issue: 485
---

# Issue #485 — title

## Issue Review
**Status**: complete
**When**: 2026-05-25 14:19 -04:00
**By**: Claude Code Agent (model)

**Issue**: #485

### Actions
- [ ] (must-fix) do the thing
- [x] (suggestion, Copilot R2) already done

## Plan Authored
**Status**: complete
**When**: 2026-05-25 15:02 -04:00
**By**: Author (model)

**Plan**: `.agent/work-plans/issue-485/plan.md` at `239e9e2`

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-21 00:35 Z
**By**: Author (model)
**Verdict**: changes-requested
**Branch**: feature/issue-470 at `2ef0d71`

### Findings
- [x] (must-fix) cascade-pattern instance

## Integrated Review
**Status**: complete
**When**: 2026-05-18 23:10 +00:00
**By**: Author (model)

**PR**: #471 at `f5817e3`
**Sources**: 4

### Findings
- [x] (cross-confirmed) coauthor handling
"""

# A separate fixture with an actual `## External Review` heading — issue-468
# (the golden-output reference) has none, so predecessor recognition needs
# its own fixture.
PREDECESSOR = """\
---
issue: 460
---

# Issue #460 — title

## External Review
**Status**: complete
**When**: 2026-05-10 09:00 -04:00
**By**: Author (model)

**PR**: #99 at `abc1234`

### Actions
- [ ] (suggestion, Copilot) tidy a docstring
"""


def _by_type(result):
    return {e["type"]: e for e in result["entries"]}


class TestParseProgress(unittest.TestCase):
    def setUp(self):
        self.result = parse_progress(GOLDEN, path="golden.md")
        self.entries = _by_type(self.result)

    def test_frontmatter_issue(self):
        self.assertEqual(self.result["issue"], 485)

    def test_all_four_entries_parsed(self):
        self.assertEqual(len(self.result["entries"]), 4)
        self.assertEqual(
            [e["type"] for e in self.result["entries"]],
            ["Issue Review", "Plan Authored", "Local Review (Pre-Push)", "Integrated Review"],
        )

    def test_issue_correlation(self):
        self.assertEqual(
            self.entries["Issue Review"]["correlation"],
            {"kind": "issue", "issue": 485},
        )

    def test_plan_correlation(self):
        self.assertEqual(
            self.entries["Plan Authored"]["correlation"],
            {"kind": "plan", "path": ".agent/work-plans/issue-485/plan.md", "sha": "239e9e2"},
        )

    def test_branch_correlation_alternative(self):
        # **Branch** is the alternative to **PR** for review entries.
        self.assertEqual(
            self.entries["Local Review (Pre-Push)"]["correlation"],
            {"kind": "branch", "branch": "feature/issue-470", "sha": "2ef0d71"},
        )

    def test_pr_correlation_with_field_before_canonical(self):
        # **Verdict** precedes **PR**; must still resolve by field name.
        self.assertEqual(
            self.entries["Integrated Review"]["correlation"],
            {"kind": "pr", "pr": 471, "sha": "f5817e3"},
        )

    def test_offset_detection_numeric_and_z(self):
        self.assertTrue(self.entries["Issue Review"]["when_has_offset"])
        self.assertTrue(self.entries["Local Review (Pre-Push)"]["when_has_offset"])  # Z
        self.assertTrue(self.entries["Integrated Review"]["when_has_offset"])

    def test_findings_checked_and_source_hint(self):
        actions = self.entries["Issue Review"]["findings"]
        self.assertEqual(len(actions), 2)
        self.assertFalse(actions[0]["checked"])
        self.assertEqual(actions[0]["source_hint"], "must-fix")
        self.assertTrue(actions[1]["checked"])
        self.assertEqual(actions[1]["source_hint"], "suggestion, Copilot R2")
        self.assertEqual(actions[0]["section"], "Actions")

    def test_recognized_flag(self):
        for entry in self.result["entries"]:
            self.assertTrue(entry["recognized"], entry["type"])


class TestPredecessorRecognition(unittest.TestCase):
    def test_external_review_predecessor_of(self):
        result = parse_progress(PREDECESSOR, path="pred.md")
        entry = result["entries"][0]
        self.assertEqual(entry["type"], "External Review")
        self.assertEqual(entry["predecessor_of"], "Integrated Review")
        self.assertEqual(entry["correlation"], {"kind": "pr", "pr": 99, "sha": "abc1234"})

    def test_type_filter_matches_predecessor(self):
        result = parse_progress(PREDECESSOR, path="pred.md")
        entry = result["entries"][0]
        # Requesting Integrated Review must also match External Review history.
        self.assertTrue(_matches_type(entry, {"Integrated Review"}))
        self.assertFalse(_matches_type(entry, {"Plan Review"}))

    def test_suffixed_external_review_normalizes_to_canonical_base(self):
        # Legacy non-conformant heading: must still be recognized as External
        # Review (and thus an Integrated Review predecessor), not silently
        # dropped by type filters.
        text = (
            "## External Review (Round 5–6)\n"
            "**PR**: #5 at `abc1234`\n"
            "### Actions\n"
            "- [ ] (suggestion) thing\n"
        )
        entry = parse_progress(text)["entries"][0]
        self.assertEqual(entry["type"], "External Review (Round 5–6)")
        self.assertEqual(entry["base_type"], "External Review")
        self.assertTrue(entry["recognized"])
        self.assertEqual(entry["predecessor_of"], "Integrated Review")
        self.assertEqual(entry["correlation"], {"kind": "pr", "pr": 5, "sha": "abc1234"})
        self.assertTrue(_matches_type(entry, {"Integrated Review"}))
        self.assertTrue(_matches_type(entry, {"External Review"}))

    def test_local_review_prepush_parenthetical_not_stripped(self):
        # `Local Review (Pre-Push)` is canonical as-is; its parenthetical must
        # NOT be stripped to `Local Review`.
        text = "## Local Review (Pre-Push)\n**Branch**: feat at `abc1234`\n"
        entry = parse_progress(text)["entries"][0]
        self.assertEqual(entry["base_type"], "Local Review (Pre-Push)")
        self.assertTrue(entry["recognized"])


class TestMalformedAndEdgeCases(unittest.TestCase):
    def test_missing_correlation_field_does_not_crash(self):
        text = (
            "## Issue Review\n"
            "**Status**: complete\n"
            "**When**: 2026-05-25 14:19 -04:00\n"
            "**By**: x\n"
        )
        result = parse_progress(text)
        self.assertIsNone(result["entries"][0]["correlation"])

    def test_missing_offset_flagged(self):
        text = "## Issue Review\n**When**: 2026-05-25 14:19\n\n**Issue**: #1\n"
        result = parse_progress(text)
        self.assertFalse(result["entries"][0]["when_has_offset"])

    def test_non_canonical_heading_marked_unrecognized(self):
        text = "## Summary\n**When**: 2026-05-25 14:19 -04:00\n"
        result = parse_progress(text)
        self.assertEqual(result["entries"][0]["type"], "Summary")
        self.assertFalse(result["entries"][0]["recognized"])

    def test_empty_file(self):
        result = parse_progress("")
        self.assertEqual(result["entries"], [])
        self.assertIsNone(result["issue"])

    def test_h1_and_subsections_not_treated_as_entries(self):
        # Only `## ` headings are entries; `# ` and `### ` are not.
        result = parse_progress(GOLDEN)
        types = [e["type"] for e in result["entries"]]
        self.assertNotIn("Issue #485 — title", types)
        self.assertNotIn("Findings", types)
        self.assertNotIn("Actions", types)

    def test_fenced_code_block_not_parsed_as_entry(self):
        # Regression: a heading + checkboxes quoted inside a fenced block must
        # NOT become a phantom entry (it would inject ghost findings at a
        # real-looking correlation SHA into the integrator).
        text = (
            "## Local Review\n"
            "**PR**: #5 at `abc1234`\n"
            "### Findings\n"
            "- [ ] (must-fix) real finding\n"
            "\n"
            "```markdown\n"
            "## External Review\n"
            "**PR**: #999 at `deadbeef`\n"
            "- [x] (suggestion) fake finding inside fence\n"
            "```\n"
        )
        result = parse_progress(text)
        self.assertEqual([e["type"] for e in result["entries"]], ["Local Review"])
        # The fenced checkbox must not be counted as a finding.
        findings = result["entries"][0]["findings"]
        self.assertEqual(len(findings), 1)
        self.assertEqual(findings[0]["text"], "(must-fix) real finding")

    def test_crlf_line_endings(self):
        text = "## Issue Review\r\n**When**: 2026-05-25 14:19 -04:00\r\n\r\n**Issue**: #7\r\n"
        result = parse_progress(text)
        self.assertEqual(result["entries"][0]["correlation"], {"kind": "issue", "issue": 7})
        self.assertTrue(result["entries"][0]["when_has_offset"])

    def test_no_frontmatter_body_preserved(self):
        text = "# Issue #3 — title\n\n## Issue Review\n**Issue**: #3\n"
        result = parse_progress(text)
        self.assertIsNone(result["issue"])
        self.assertEqual([e["type"] for e in result["entries"]], ["Issue Review"])

    def test_leading_hr_not_treated_as_frontmatter(self):
        # A `---` horizontal rule (no key: line) must not swallow the body.
        text = "---\nintro prose\n---\n\n## Issue Review\n**Issue**: #4\n"
        result = parse_progress(text)
        self.assertIsNone(result["issue"])
        self.assertEqual([e["type"] for e in result["entries"]], ["Issue Review"])

    def test_checkbox_without_leading_paren_has_no_source_hint(self):
        text = "## Issue Review\n**Issue**: #1\n### Actions\n- [ ] plain action, no source\n"
        result = parse_progress(text)
        self.assertIsNone(result["entries"][0]["findings"][0]["source_hint"])

    def test_checkbox_before_subsection_ignored(self):
        # ADR-0013 puts findings/actions under ### subsections; a stray checkbox
        # in the header area (before any ###) is not a finding.
        text = (
            "## Issue Review\n"
            "**Issue**: #1\n"
            "- [ ] (stray) header-area checkbox\n"
            "### Actions\n"
            "- [ ] (real) under a subsection\n"
        )
        findings = parse_progress(text)["entries"][0]["findings"]
        self.assertEqual(len(findings), 1)
        self.assertEqual(findings[0]["section"], "Actions")
        self.assertEqual(findings[0]["source_hint"], "real")


class TestCli(unittest.TestCase):
    def _run(self, text, *args):
        with tempfile.NamedTemporaryFile(
            "w", suffix=".md", delete=False, encoding="utf-8"
        ) as handle:
            handle.write(text)
            name = handle.name
        try:
            proc = subprocess.run(
                [sys.executable, SCRIPT, name, *args],
                capture_output=True,
                text=True,
                check=True,
            )
            return json.loads(proc.stdout)
        finally:
            Path(name).unlink()

    def test_cli_emits_json(self):
        data = self._run(GOLDEN)
        self.assertEqual(len(data["entries"]), 4)

    def test_cli_type_filter(self):
        data = self._run(GOLDEN, "--type", "Plan Authored")
        self.assertEqual([e["type"] for e in data["entries"]], ["Plan Authored"])

    def test_cli_missing_file_exits_nonzero(self):
        proc = subprocess.run(
            [sys.executable, SCRIPT, "/no/such/file.md"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(proc.returncode, 1)


if __name__ == "__main__":
    unittest.main()
