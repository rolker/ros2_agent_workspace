"""Tests for validate_workspace.py's outcome classification and exit status (#609).

With zero repos configured, `not (missing or extra or mismatched)` is vacuously
true, so the script printed "✅ Workspace validation PASSED!" and exited 0 — a
green all-clear over a comparison that compared nothing. That is the same
false-green class #609 removed from sync_repos.py, and it reaches an operator
through `make validate` and through dashboard.sh, which branches on this
script's exit code.

Seams: `get_overlay_repos` (the configuration) and `get_actual_repos` (the
filesystem walk) are stubbed, so no test reads this host's real `layers/` or
`configs/manifest`.
"""

import subprocess
import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SCRIPTS_DIR))
sys.path.insert(0, str(SCRIPTS_DIR / "lib"))

import pytest  # noqa: E402

import validate_workspace as vw  # noqa: E402
from validate_workspace import ValidationResult  # noqa: E402


def _repo(name, version="jazzy", source_file="core.repos"):
    return {
        "name": name,
        "url": f"git@example:{name}.git",
        "version": version,
        "source_file": source_file,
    }


def _stub(monkeypatch, tmp_path, configured, actual=None, optional_layers=()):
    """Stub the configuration and the on-disk repo scan.

    `configured` is a list of repo dicts as get_overlay_repos returns them;
    `actual` is the {name: info} map get_actual_repos returns, defaulting to
    "exactly what is configured, on the expected branch".

    A plain helper rather than a pytest fixture: a fixture's name and the test
    parameter receiving it are necessarily the same identifier, which pylint
    reports as redefined-outer-name (the convention test_sync_repos.py set).
    """
    if actual is None:
        actual = {
            r["name"]: {
                "path": str(tmp_path / r["name"]),
                "branch": r["version"],
                "context": "core_ws",
            }
            for r in configured
        }
    monkeypatch.setattr(vw, "get_workspace_root", lambda: str(tmp_path))
    monkeypatch.setattr(vw, "get_overlay_repos", lambda include_underlay=False: list(configured))
    monkeypatch.setattr(vw, "get_actual_repos", lambda root: dict(actual))
    monkeypatch.setattr(vw, "get_optional_layers", lambda root: set(optional_layers))
    # Version checking shells out to `git rev-parse`; with no real repos that
    # resolves to nothing, which is the "compare branch names" fallback. Pin it
    # so these tests are about classification, not about git.
    monkeypatch.setattr(vw, "get_git_commit", lambda path: None)


def _run_main(monkeypatch, argv=()):
    """Run main() with a stubbed argv; returns the SystemExit code."""
    monkeypatch.setattr(sys, "argv", ["validate_workspace.py", *argv])
    with pytest.raises(SystemExit) as exc:
        vw.main()
    return exc.value.code


# --------------------------------------------------------------------------
# The regression: an empty configuration is not a pass
# --------------------------------------------------------------------------


def test_no_configured_repos_is_unconfigured_not_passed(monkeypatch, tmp_path):
    _stub(monkeypatch, tmp_path, configured=[])
    result, missing = vw.validate_workspace()
    assert result is ValidationResult.UNCONFIGURED
    assert not missing


def test_no_configured_repos_never_prints_the_green_all_clear(monkeypatch, tmp_path, capsys):
    """The operator-visible half: `make validate` used to print PASSED here."""
    _stub(monkeypatch, tmp_path, configured=[])
    vw.validate_workspace()
    out = capsys.readouterr().out
    assert "PASSED" not in out
    assert "nothing to validate" in out


def test_no_configured_repos_names_the_remedy(monkeypatch, tmp_path, capsys):
    """`--fix` cannot help here — there is nothing to import from — so the
    message must point at setup, the way sync_repos.py's does."""
    _stub(monkeypatch, tmp_path, configured=[])
    vw.validate_workspace()
    out = capsys.readouterr().out
    assert "make setup-all" in out
    assert "worktree" in out


def test_no_configured_repos_exits_three(monkeypatch, tmp_path):
    """3, not 0. dashboard.sh reads this code."""
    _stub(monkeypatch, tmp_path, configured=[])
    assert _run_main(monkeypatch) == 3


def test_no_configured_repos_does_not_attempt_a_fix(monkeypatch, tmp_path):
    """--fix imports missing repos from .repos files; with no .repos files
    there is nothing to import from, so it must not run (and must not mask
    the 3 with fix_workspace's own exit 1)."""
    _stub(monkeypatch, tmp_path, configured=[])
    called = []
    monkeypatch.setattr(vw, "fix_workspace", lambda missing, verbose=False: called.append(missing))
    assert _run_main(monkeypatch, ["--fix"]) == 3
    assert not called


def test_only_drift_is_routed_into_fix(monkeypatch):
    """The gate is on the outcome, not on `missing` happening to be empty.

    Today UNCONFIGURED always returns an empty missing list, so a gate of
    "anything that is not a pass" would behave identically — which is exactly
    why the intent is worth pinning: `--fix` imports repos from .repos files,
    and UNCONFIGURED means there are no .repos files to import from. Driven by
    stubbing the outcome directly, since the real code cannot produce this
    pair.
    """
    monkeypatch.setattr(
        vw,
        "validate_workspace",
        lambda verbose=False: (ValidationResult.UNCONFIGURED, [{"name": "alpha"}]),
    )
    called = []
    monkeypatch.setattr(vw, "fix_workspace", lambda missing, verbose=False: called.append(missing))
    assert _run_main(monkeypatch, ["--fix"]) == 3
    assert not called


# --------------------------------------------------------------------------
# The other two outcomes still work — the carve-out did not eat them
# --------------------------------------------------------------------------


def test_matching_workspace_still_passes_and_exits_zero(monkeypatch, tmp_path):
    _stub(monkeypatch, tmp_path, configured=[_repo("alpha"), _repo("beta")])
    assert vw.validate_workspace()[0] is ValidationResult.PASSED
    _stub(monkeypatch, tmp_path, configured=[_repo("alpha"), _repo("beta")])
    assert _run_main(monkeypatch) == 0


def test_missing_repo_is_drift_and_exits_one(monkeypatch, tmp_path):
    _stub(monkeypatch, tmp_path, configured=[_repo("alpha"), _repo("beta")], actual={})
    result, missing = vw.validate_workspace()
    assert result is ValidationResult.DRIFTED
    assert sorted(item["name"] for item in missing) == ["alpha", "beta"]
    _stub(monkeypatch, tmp_path, configured=[_repo("alpha"), _repo("beta")], actual={})
    assert _run_main(monkeypatch) == 1


def test_drift_and_unconfigured_have_different_exit_codes(monkeypatch, tmp_path):
    """The whole point of the third state: dashboard.sh must be able to tell
    "your workspace drifted" from "your workspace was never set up"."""
    assert vw.EXIT_CODES[ValidationResult.DRIFTED] != vw.EXIT_CODES[ValidationResult.UNCONFIGURED]


def test_optional_layer_repo_missing_is_still_a_pass(monkeypatch, tmp_path):
    """Pre-existing behaviour that must survive: a repo from an optional layer
    is allowed to be absent (the carve-out sync_repos.py now shares)."""
    _stub(
        monkeypatch,
        tmp_path,
        configured=[_repo("alpha"), _repo("private", source_file="site.repos")],
        actual={
            "alpha": {"path": str(tmp_path / "alpha"), "branch": "jazzy", "context": "core_ws"}
        },
        optional_layers=["site"],
    )
    assert vw.validate_workspace()[0] is ValidationResult.PASSED


# --------------------------------------------------------------------------
# --fix takes the re-validation's answer
# --------------------------------------------------------------------------


def test_successful_fix_reports_the_revalidated_result(monkeypatch, tmp_path):
    """main() re-validated after a successful --fix and then discarded the
    answer, exiting 1 on the stale one — a fixed workspace still read red."""
    states = iter(
        [
            (ValidationResult.DRIFTED, [{"name": "alpha", "config": _repo("alpha")}]),
            (ValidationResult.PASSED, []),
        ]
    )
    monkeypatch.setattr(vw, "validate_workspace", lambda verbose=False: next(states))
    monkeypatch.setattr(vw, "fix_workspace", lambda missing, verbose=False: True)
    assert _run_main(monkeypatch, ["--fix"]) == 0


def test_fix_that_leaves_drift_still_exits_one(monkeypatch, tmp_path):
    """The other direction: the re-validation's answer is taken, not assumed."""
    states = iter(
        [
            (ValidationResult.DRIFTED, [{"name": "alpha", "config": _repo("alpha")}]),
            (ValidationResult.DRIFTED, [{"name": "alpha", "config": _repo("alpha")}]),
        ]
    )
    monkeypatch.setattr(vw, "validate_workspace", lambda verbose=False: next(states))
    monkeypatch.setattr(vw, "fix_workspace", lambda missing, verbose=False: True)
    assert _run_main(monkeypatch, ["--fix"]) == 1


# --------------------------------------------------------------------------
# Exit-code contract, end to end
# --------------------------------------------------------------------------


def test_no_outcome_reuses_argparses_exit_code():
    """2 is argparse's. If UNCONFIGURED were numbered 2, dashboard.sh could not
    tell "no repos configured" from "you typed the flag wrong"."""
    assert 2 not in vw.EXIT_CODES.values()


def test_usage_error_still_exits_two_not_three():
    """Why UNCONFIGURED is 3: argparse owns 2, and a caller branching on the
    exit code cannot tell "no repos configured" from "you typed it wrong"."""
    proc = subprocess.run(
        [sys.executable, str(SCRIPTS_DIR / "validate_workspace.py"), "--nonsense"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 2
