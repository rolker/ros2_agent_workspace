"""Tests for build_report_generator.py.

Unit tests for parse_log_line() and integration tests that run the full script
via subprocess against synthetic events.log files.
"""

import subprocess
import sys
from pathlib import Path

# Import the module under test
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from build_report_generator import parse_log_line

SCRIPT = str(Path(__file__).resolve().parent.parent / "build_report_generator.py")


# ---------------------------------------------------------------------------
# Unit tests for parse_log_line
# ---------------------------------------------------------------------------


class TestParseLogLine:
    def test_job_queued_with_ordered_dict(self):
        """Regression test for issue #215 — OrderedDict in JobQueued data."""
        line = (
            "[0.123] (my_pkg) JobQueued: " + "OrderedDict([('identifier', 'my_pkg'), ('deps', [])])"
        )
        result = parse_log_line(line)
        assert result is not None
        pkg, event, _data_str = result
        assert pkg == "my_pkg"
        assert event == "JobQueued"

    def test_job_queued_plain_dict(self):
        line = "[0.001] (nav2_core) JobQueued: {'identifier': 'nav2_core'}"
        result = parse_log_line(line)
        assert result is not None
        pkg, event, data_str = result
        assert pkg == "nav2_core"
        assert event == "JobQueued"
        assert data_str == "{'identifier': 'nav2_core'}"

    def test_job_ended(self):
        line = "[1.500] (my_pkg) JobEnded: {'identifier': 'my_pkg', 'rc': 0}"
        result = parse_log_line(line)
        assert result is not None
        pkg, event, data_str = result
        assert pkg == "my_pkg"
        assert event == "JobEnded"
        assert "'rc': 0" in data_str

    def test_job_ended_failure(self):
        line = "[2.100] (bad_pkg) JobEnded: {'identifier': 'bad_pkg', 'rc': 2}"
        result = parse_log_line(line)
        assert result is not None
        pkg, event, data_str = result
        assert pkg == "bad_pkg"
        assert event == "JobEnded"
        assert "'rc': 2" in data_str

    def test_stderr_line(self):
        line = "[0.800] (my_pkg) StderrLine: {'data': b'warning: unused variable'}"
        result = parse_log_line(line)
        assert result is not None
        pkg, event, _data_str = result
        assert pkg == "my_pkg"
        assert event == "StderrLine"

    def test_timer_event_dash_package(self):
        line = "[0.500] (-) TimerEvent: {}"
        result = parse_log_line(line)
        assert result is not None
        pkg, event, _data_str = result
        assert pkg == "-"
        assert event == "TimerEvent"

    def test_malformed_no_timestamp_bracket(self):
        line = "no bracket here (pkg) JobQueued: {}"
        assert parse_log_line(line) is None

    def test_malformed_no_package_paren(self):
        line = "[0.001] missing_parens JobQueued: {}"
        assert parse_log_line(line) is None

    def test_malformed_no_event_colon(self):
        line = "[0.001] (pkg) NoColonHere"
        assert parse_log_line(line) is None

    def test_empty_string(self):
        assert parse_log_line("") is None

    def test_unparseable_data_still_parsed(self):
        """parse_log_line returns the raw data string without evaluating it."""
        line = "[0.001] (pkg) JobEnded: not_valid_python{{"
        result = parse_log_line(line)
        # The regex parser extracts the data string as-is; validation
        # happens later in main() via ast.literal_eval.
        assert result is not None
        pkg, event, data_str = result
        assert pkg == "pkg"
        assert event == "JobEnded"
        assert data_str == "not_valid_python{{"


# ---------------------------------------------------------------------------
# Integration test helpers
# ---------------------------------------------------------------------------


def _make_events_log(tmp_path, lines):
    """Create a log_dir with events.log containing the given lines."""
    log_dir = tmp_path / "log"
    log_dir.mkdir()
    (log_dir / "events.log").write_text("\n".join(lines) + "\n")
    return str(log_dir)


def _run_report(log_dir, layer_name="test_layer"):
    """Run build_report_generator.py and return (stdout, returncode)."""
    result = subprocess.run(
        [sys.executable, SCRIPT, "--log-dir", log_dir, "--layer-name", layer_name],
        capture_output=True,
        text=True,
    )
    return result.stdout.strip(), result.returncode


# ---------------------------------------------------------------------------
# Integration tests — full script via subprocess
# ---------------------------------------------------------------------------


class TestIntegrationHappyPath:
    def test_all_packages_succeed(self, tmp_path):
        lines = [
            "[0.001] (pkg_a) JobQueued: {'identifier': 'pkg_a'}",
            "[0.002] (pkg_b) JobQueued: {'identifier': 'pkg_b'}",
            "[0.003] (pkg_c) JobQueued: {'identifier': 'pkg_c'}",
            "[1.000] (pkg_a) JobEnded: {'identifier': 'pkg_a', 'rc': 0}",
            "[1.100] (pkg_b) JobEnded: {'identifier': 'pkg_b', 'rc': 0}",
            "[1.200] (pkg_c) JobEnded: {'identifier': 'pkg_c', 'rc': 0}",
        ]
        log_dir = _make_events_log(tmp_path, lines)
        stdout, rc = _run_report(log_dir)

        assert rc == 0
        assert "test_layer" in stdout
        assert "3 (OK: 3)" in stdout
        assert "\u2705 Success" in stdout  # ✅

    def test_single_package(self, tmp_path):
        lines = [
            "[0.001] (solo) JobQueued: {'identifier': 'solo'}",
            "[1.000] (solo) JobEnded: {'identifier': 'solo', 'rc': 0}",
        ]
        log_dir = _make_events_log(tmp_path, lines)
        stdout, _ = _run_report(log_dir)

        assert "1 (OK: 1)" in stdout
        assert "\u2705 Success" in stdout


class TestIntegrationFailures:
    def test_mixed_pass_fail(self, tmp_path):
        lines = [
            "[0.001] (good_a) JobQueued: {'identifier': 'good_a'}",
            "[0.002] (good_b) JobQueued: {'identifier': 'good_b'}",
            "[0.003] (bad_c) JobQueued: {'identifier': 'bad_c'}",
            "[1.000] (good_a) JobEnded: {'identifier': 'good_a', 'rc': 0}",
            "[1.100] (good_b) JobEnded: {'identifier': 'good_b', 'rc': 0}",
            "[1.200] (bad_c) JobEnded: {'identifier': 'bad_c', 'rc': 1}",
        ]
        log_dir = _make_events_log(tmp_path, lines)
        stdout, _ = _run_report(log_dir)

        assert "3 (OK: 2)" in stdout
        assert "\u274c Failed" in stdout  # ❌
        assert "bad_c" in stdout

    def test_all_fail(self, tmp_path):
        lines = [
            "[0.001] (a) JobQueued: {'identifier': 'a'}",
            "[0.002] (b) JobQueued: {'identifier': 'b'}",
            "[1.000] (a) JobEnded: {'identifier': 'a', 'rc': 1}",
            "[1.100] (b) JobEnded: {'identifier': 'b', 'rc': 2}",
        ]
        log_dir = _make_events_log(tmp_path, lines)
        stdout, _ = _run_report(log_dir)

        assert "2 (OK: 0)" in stdout
        assert "\u274c Failed" in stdout
        assert "a" in stdout
        assert "b" in stdout


class TestIntegrationStderr:
    def test_stderr_warnings_success(self, tmp_path):
        """Package succeeds but has stderr — shows as warning."""
        lines = [
            "[0.001] (warn_pkg) JobQueued: {'identifier': 'warn_pkg'}",
            "[0.500] (warn_pkg) StderrLine: {'data': b'some warning'}",
            "[1.000] (warn_pkg) JobEnded: {'identifier': 'warn_pkg', 'rc': 0}",
        ]
        log_dir = _make_events_log(tmp_path, lines)
        stdout, _ = _run_report(log_dir)

        assert "\u2705 Success" in stdout
        assert "warn_pkg" in stdout

    def test_stderr_and_failure(self, tmp_path):
        """Failed package with stderr + another package with only warnings."""
        lines = [
            "[0.001] (fail_pkg) JobQueued: {'identifier': 'fail_pkg'}",
            "[0.002] (warn_only) JobQueued: {'identifier': 'warn_only'}",
            "[0.500] (fail_pkg) StderrLine: {'data': b'error'}",
            "[0.600] (warn_only) StderrLine: {'data': b'warning'}",
            "[1.000] (fail_pkg) JobEnded: {'identifier': 'fail_pkg', 'rc': 1}",
            "[1.100] (warn_only) JobEnded: {'identifier': 'warn_only', 'rc': 0}",
        ]
        log_dir = _make_events_log(tmp_path, lines)
        stdout, _ = _run_report(log_dir)

        assert "\u274c Failed" in stdout
        assert "**Failed**: fail_pkg" in stdout
        assert "**Warnings**: warn_only" in stdout


class TestIntegrationEdgeCases:
    def test_missing_log_file(self, tmp_path):
        """No events.log at all."""
        log_dir = str(tmp_path / "nonexistent")
        stdout, _ = _run_report(log_dir)

        assert "\u26a0\ufe0f Log Not Found" in stdout
        assert "0" in stdout

    def test_empty_log_file(self, tmp_path):
        """events.log exists but is empty."""
        log_dir = _make_events_log(tmp_path, [""])
        stdout, _ = _run_report(log_dir)

        assert "\u26a0\ufe0f No Pkgs" in stdout

    def test_ordered_dict_regression(self, tmp_path):
        """OrderedDict in JobQueued data (regression test for #215)."""
        lines = [
            "[0.001] (pkg_a) JobQueued: OrderedDict([('identifier', 'pkg_a')])",
            "[0.002] (pkg_b) JobQueued: OrderedDict([('identifier', 'pkg_b')])",
            "[1.000] (pkg_a) JobEnded: {'identifier': 'pkg_a', 'rc': 0}",
            "[1.100] (pkg_b) JobEnded: {'identifier': 'pkg_b', 'rc': 0}",
        ]
        log_dir = _make_events_log(tmp_path, lines)
        stdout, rc = _run_report(log_dir)

        # OrderedDict lines are parsed by the regex but can't be
        # ast.literal_eval'd, so JobQueued still registers the package.
        # The script must not crash.
        assert rc == 0
        assert "2 (OK: 2)" in stdout
        assert "\u2705 Success" in stdout

    def test_unparseable_job_ended_data(self, tmp_path):
        """JobEnded with unparseable data -> defaults to rc=1 (failure)."""
        lines = [
            "[0.001] (pkg_x) JobQueued: {'identifier': 'pkg_x'}",
            "[1.000] (pkg_x) JobEnded: not_valid_python",
        ]
        log_dir = _make_events_log(tmp_path, lines)
        stdout, _ = _run_report(log_dir)

        # ast.literal_eval raises ValueError/SyntaxError -> caught,
        # defaults to rc=1 so the package is reported as failed.
        assert "1 (OK: 0)" in stdout
        assert "\u274c Failed" in stdout
        assert "pkg_x" in stdout

    def test_non_dict_job_ended_data(self, tmp_path):
        """JobEnded with a list instead of dict -> defaults to rc=1 (failure)."""
        lines = [
            "[0.001] (pkg_y) JobQueued: {'identifier': 'pkg_y'}",
            "[1.000] (pkg_y) JobEnded: [1, 2, 3]",
        ]
        log_dir = _make_events_log(tmp_path, lines)
        stdout, rc = _run_report(log_dir)

        # ast.literal_eval succeeds (valid list), but list has no .get()
        # -> AttributeError caught -> defaults to rc=1.
        assert rc == 0  # script itself exits cleanly
        assert "1 (OK: 0)" in stdout
        assert "\u274c Failed" in stdout
        assert "pkg_y" in stdout

    def test_timer_events_ignored(self, tmp_path):
        """TimerEvent lines with (-) package don't affect counts."""
        lines = [
            "[0.001] (my_pkg) JobQueued: {'identifier': 'my_pkg'}",
            "[0.500] (-) TimerEvent: {}",
            "[0.600] (-) TimerEvent: {}",
            "[1.000] (my_pkg) JobEnded: {'identifier': 'my_pkg', 'rc': 0}",
        ]
        log_dir = _make_events_log(tmp_path, lines)
        stdout, _ = _run_report(log_dir)

        assert "1 (OK: 1)" in stdout
        assert "\u2705 Success" in stdout

    def test_custom_layer_name(self, tmp_path):
        """Layer name appears in output."""
        lines = [
            "[0.001] (p) JobQueued: {'identifier': 'p'}",
            "[1.000] (p) JobEnded: {'identifier': 'p', 'rc': 0}",
        ]
        log_dir = _make_events_log(tmp_path, lines)
        stdout, _ = _run_report(log_dir, layer_name="simulation")

        assert "| simulation |" in stdout
