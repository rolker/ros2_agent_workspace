"""Tests for sync_repos.py's outcome classification and exit status (#609).

`make sync` used to print a success summary and exit 0 even when repos failed
to update, so an unattended sync reported green while leaving repos stale. The
fix classifies each repo as SYNCED / SKIPPED / FAILED and exits non-zero when
any real failure occurred — while keeping benign skips (dirty tree, detached
HEAD) at exit 0, because a red that fires on an intentionally dirty repo is a
signal everyone learns to ignore.

Two layers, deliberately separate:

  - `sync_repo()` classification, stubbed at the `run_network_cmd` /
    `run_git_cmd` seam. This is the layer that catches the original bug — the
    failure branches used to fall through to `return True`.
  - `main()` accumulation and exit status, stubbed at the `get_overlay_repos`
    and `SCRIPT_DIR` seams so nothing touches this host's real `layers/`.

All subprocess and network activity is stubbed; no test reaches the network.
"""

import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SCRIPTS_DIR))
sys.path.insert(0, str(SCRIPTS_DIR / "lib"))

import pytest  # noqa: E402

import sync_repos  # noqa: E402
from sync_repos import SyncOutcome  # noqa: E402


# --------------------------------------------------------------------------
# sync_repo() classification
# --------------------------------------------------------------------------


def make_repo(tmp_path):
    """A real directory, so `.exists()` is genuine rather than stubbed.

    A plain helper rather than a pytest fixture: a fixture's name and the test
    parameter that receives it are necessarily the same identifier, which
    pylint reports as redefined-outer-name. Taking tmp_path directly avoids
    that without suppressing the check.
    """
    d = tmp_path / "some_repo"
    d.mkdir()
    return d


def _stub_git(monkeypatch, *, dirty=False, branch="jazzy", branch_ok=True, status_ok=True):
    """Stub the read-only git probes sync_repo() makes before syncing."""

    def fake_run_git_cmd(repo_path, cmd_args, dry_run=False):
        if cmd_args[:1] == ["status"]:
            if not status_ok:
                return False, "fatal: index file corrupt"
            return True, "M file.txt" if dirty else ""
        if cmd_args[:1] == ["branch"]:
            return (True, branch) if branch_ok else (False, "not a git repository")
        return True, ""

    monkeypatch.setattr(sync_repos, "run_git_cmd", fake_run_git_cmd)


def _stub_network(monkeypatch, success, output=""):
    monkeypatch.setattr(
        sync_repos,
        "run_network_cmd",
        lambda repo_path, cmd_args, dry_run=False: (success, output),
    )


def test_successful_pull_is_synced(monkeypatch, tmp_path):
    _stub_git(monkeypatch)
    _stub_network(monkeypatch, True, "Already up to date.")
    assert sync_repos.sync_repo(make_repo(tmp_path), "r") is SyncOutcome.SYNCED


def test_failed_pull_is_failed(monkeypatch, tmp_path):
    """The regression that #609 exists for: this used to return True."""
    _stub_git(monkeypatch)
    _stub_network(monkeypatch, False, "Connection reset by peer")
    assert sync_repos.sync_repo(make_repo(tmp_path), "r") is SyncOutcome.FAILED


def test_failed_fetch_on_feature_branch_is_failed(monkeypatch, tmp_path):
    """The feature-branch arm had the same fall-through as the default arm."""
    _stub_git(monkeypatch, branch="feature/issue-1")
    _stub_network(monkeypatch, False, "Connection reset by peer")
    assert sync_repos.sync_repo(make_repo(tmp_path), "r") is SyncOutcome.FAILED


def test_dirty_tree_is_a_benign_skip(monkeypatch, tmp_path):
    """Must NOT be FAILED — an intentionally dirty repo turning the run red is
    the false-red this design exists to avoid."""
    _stub_git(monkeypatch, dirty=True)
    assert sync_repos.sync_repo(make_repo(tmp_path), "r") is SyncOutcome.SKIPPED


def test_detached_head_is_a_benign_skip(monkeypatch, tmp_path):
    """`git branch --show-current` succeeds and prints nothing."""
    _stub_git(monkeypatch, branch="")
    assert sync_repos.sync_repo(make_repo(tmp_path), "r") is SyncOutcome.SKIPPED


def test_unreadable_git_state_is_failed(monkeypatch, tmp_path):
    """The git command itself failing (not a repo / corrupt .git) is NOT the
    same as a detached HEAD. Collapsing the two leaves a broken repo silently
    stale under a green exit."""
    _stub_git(monkeypatch, branch_ok=False)
    assert sync_repos.sync_repo(make_repo(tmp_path), "r") is SyncOutcome.FAILED


def test_unreadable_working_tree_is_failed(monkeypatch, tmp_path):
    """`git status --porcelain` failing means the working tree could not be
    read — NOT that it is clean. Treating it as clean let a repo with a corrupt
    index and real uncommitted changes sync-and-report green (#609)."""
    _stub_git(monkeypatch, status_ok=False, dirty=True)
    _stub_network(monkeypatch, True, "Already up to date.")
    assert sync_repos.sync_repo(make_repo(tmp_path), "r") is SyncOutcome.FAILED


def test_is_dirty_distinguishes_unreadable_from_clean(monkeypatch, tmp_path):
    """The three states must stay distinct at the helper level too."""
    repo = make_repo(tmp_path)
    _stub_git(monkeypatch, dirty=False)
    assert sync_repos.is_dirty(repo) is False
    _stub_git(monkeypatch, dirty=True)
    assert sync_repos.is_dirty(repo) is True
    _stub_git(monkeypatch, status_ok=False)
    assert sync_repos.is_dirty(repo) is None


def test_run_git_cmd_reports_oserror_instead_of_raising(monkeypatch, tmp_path):
    """An unreadable repo dir raises OSError, not CalledProcessError. Letting it
    escape kills the run mid-way with no summary and strands every later repo."""

    def boom(*args, **kwargs):
        raise PermissionError(13, "Permission denied")

    monkeypatch.setattr(sync_repos.subprocess, "run", boom)
    success, output = sync_repos.run_git_cmd(make_repo(tmp_path), ["status", "--porcelain"])
    assert success is False
    assert "cannot run git" in output


def test_missing_path_is_failed(tmp_path):
    assert sync_repos.sync_repo(tmp_path / "nope", "r") is SyncOutcome.FAILED


def test_outcomes_are_all_truthy():
    """Guards the trap the enum introduces: `if sync_repo(...)` would treat
    FAILED as success. Any call site must compare against members."""
    assert all(bool(o) for o in SyncOutcome)


# --------------------------------------------------------------------------
# main() accumulation + exit status
# --------------------------------------------------------------------------


def make_workspace(tmp_path, monkeypatch):
    """Point main() at a throwaway tree instead of this host's real layers/.

    A helper rather than a fixture, for the same reason as make_repo above.
    """
    root = tmp_path / "ws"
    (root / "layers" / "main" / "core_ws" / "src").mkdir(parents=True)
    # main() derives root_dir as SCRIPT_DIR.parent.parent
    monkeypatch.setattr(sync_repos, "SCRIPT_DIR", root / ".agent" / "scripts")
    monkeypatch.setattr(sys, "argv", ["sync_repos.py"])
    monkeypatch.setattr(sync_repos, "make_throttler", lambda *a, **k: (lambda: None))
    return root


def repo_record(name):
    """A repo record shaped like get_overlay_repos() returns: main() maps
    source_file (`core.repos`) to the workspace dir (`core_ws`)."""
    return {"name": name, "source_file": "core.repos"}


def _run_main(monkeypatch, workspace, repos, outcomes):
    """Drive main() with a scripted outcome per repo name; returns gitbug calls."""
    for r in repos:
        (workspace / "layers" / "main" / "core_ws" / "src" / r["name"]).mkdir(
            parents=True, exist_ok=True
        )
    monkeypatch.setattr(
        sync_repos.list_overlay_repos,
        "get_overlay_repos",
        lambda include_underlay=False: repos,
    )
    monkeypatch.setattr(
        sync_repos,
        "sync_repo",
        lambda path, name, dry_run=False: outcomes.get(name, SyncOutcome.SYNCED),
    )
    gitbug_calls = []
    monkeypatch.setattr(
        sync_repos,
        "sync_gitbug",
        lambda path, dry_run=False: gitbug_calls.append(Path(path).name),
    )
    return gitbug_calls


def test_all_success_exits_zero(monkeypatch, tmp_path, capsys):
    _run_main(
        monkeypatch, make_workspace(tmp_path, monkeypatch), [repo_record("a"), repo_record("b")], {}
    )
    sync_repos.main()
    out = capsys.readouterr().out
    assert "0 failures" in out
    assert "❌" not in out


def test_a_real_failure_exits_nonzero(monkeypatch, tmp_path, capsys):
    """The whole point: a failed repo must not report green."""
    _run_main(
        monkeypatch,
        make_workspace(tmp_path, monkeypatch),
        [repo_record("a"), repo_record("b")],
        {"b": SyncOutcome.FAILED},
    )
    with pytest.raises(SystemExit) as exc:
        sync_repos.main()
    assert exc.value.code == 1
    out = capsys.readouterr().out
    assert "1 failure(s)" in out
    assert "- b:" in out  # names the repo, not just a count


def test_benign_skips_alone_exit_zero(monkeypatch, tmp_path, capsys):
    """A dirty or detached repo must not turn the run red."""
    _run_main(
        monkeypatch,
        make_workspace(tmp_path, monkeypatch),
        [repo_record("a"), repo_record("b")],
        {"a": SyncOutcome.SKIPPED, "b": SyncOutcome.SKIPPED},
    )
    sync_repos.main()
    assert "0 failures" in capsys.readouterr().out


def test_unresolvable_path_is_a_failure(monkeypatch, tmp_path, capsys):
    """A configured repo we cannot even locate is the 'left stale, unnoticed'
    case — it must fail the run, and say why."""
    _run_main(monkeypatch, make_workspace(tmp_path, monkeypatch), [repo_record("a")], {})
    monkeypatch.setattr(
        sync_repos.list_overlay_repos,
        "get_overlay_repos",
        lambda include_underlay=False: [repo_record("ghost")],
    )
    with pytest.raises(SystemExit) as exc:
        sync_repos.main()
    assert exc.value.code == 1
    assert "path not resolved" in capsys.readouterr().out


def test_gitbug_runs_only_for_synced_repos(monkeypatch, tmp_path):
    """git-bug must not push against a repo whose pull just failed."""
    calls = _run_main(
        monkeypatch,
        make_workspace(tmp_path, monkeypatch),
        [repo_record("ok"), repo_record("bad"), repo_record("skip")],
        {"bad": SyncOutcome.FAILED, "skip": SyncOutcome.SKIPPED},
    )
    with pytest.raises(SystemExit):
        sync_repos.main()
    assert "bad" not in calls
    assert "skip" not in calls
    assert "ok" in calls


def test_mixed_run_counts_every_category(monkeypatch, tmp_path, capsys):
    _run_main(
        monkeypatch,
        make_workspace(tmp_path, monkeypatch),
        [repo_record("a"), repo_record("b"), repo_record("c")],
        {"b": SyncOutcome.SKIPPED, "c": SyncOutcome.FAILED},
    )
    with pytest.raises(SystemExit):
        sync_repos.main()
    out = capsys.readouterr().out
    # 'a' plus the root repo both synced; 'b' skipped; 'c' failed.
    assert "2 synced, 1 skipped" in out
