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


def _stub_git(monkeypatch, *, dirty=False, branch="jazzy", fails=()):
    """Stub the read-only git probes sync_repo() makes before syncing.

    `fails` names the probes whose git command should fail rather than return a
    value — "status" (working tree), "branch", "status-sb" (ahead/behind). One
    parameter rather than one flag per probe, so adding a probe does not widen
    the signature (and so pylint's argument limit needs no suppression).
    """

    def fake_run_git_cmd(repo_path, cmd_args, dry_run=False):
        probes = {
            ("status", "--porcelain"): ("status", "M file.txt" if dirty else ""),
            ("status", "-sb"): ("status-sb", "## feature/x...origin/feature/x"),
            ("branch", "--show-current"): ("branch", branch),
        }
        probe = probes.get(tuple(cmd_args[:2]))
        if probe is None:
            return True, ""
        name, value = probe
        if name in fails:
            return False, f"fatal: cannot read {name}"
        return True, value

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
    assert sync_repos.sync_repo(make_repo(tmp_path), "r").outcome is SyncOutcome.SYNCED


def test_failed_pull_is_failed(monkeypatch, tmp_path):
    """The regression that #609 exists for: this used to return True."""
    _stub_git(monkeypatch)
    _stub_network(monkeypatch, False, "Connection reset by peer")
    assert sync_repos.sync_repo(make_repo(tmp_path), "r").outcome is SyncOutcome.FAILED


def test_failed_fetch_on_feature_branch_is_failed(monkeypatch, tmp_path):
    """The feature-branch arm had the same fall-through as the default arm."""
    _stub_git(monkeypatch, branch="feature/issue-1")
    _stub_network(monkeypatch, False, "Connection reset by peer")
    assert sync_repos.sync_repo(make_repo(tmp_path), "r").outcome is SyncOutcome.FAILED


def test_failure_reasons_name_the_actual_cause(monkeypatch, tmp_path):
    """Every FAILED path carries its own reason — the summary prints it, and
    "sync failed" for all causes alike tells the operator nothing."""
    repo = make_repo(tmp_path)
    _stub_git(monkeypatch)
    _stub_network(monkeypatch, False, "fatal: unable to access ...: 502")
    assert sync_repos.sync_repo(repo, "r").reason == "pull failed: fatal: unable to access ...: 502"
    _stub_git(monkeypatch, fails={"branch"})
    assert sync_repos.sync_repo(repo, "r").reason == "cannot read git state"
    _stub_git(monkeypatch, dirty=True)
    assert sync_repos.sync_repo(repo, "r").reason == "uncommitted changes"
    assert sync_repos.sync_repo(tmp_path / "nope", "r").reason == "path does not exist"


def test_offline_failure_is_named_as_unreachable(monkeypatch, tmp_path):
    """An off-network host fails every repo at once and is not a retried
    transient signature — say `remote unreachable`, not `pull failed`."""
    _stub_git(monkeypatch)
    _stub_network(
        monkeypatch,
        False,
        "ssh: Could not resolve hostname github.com: Name or service not known",
    )
    result = sync_repos.sync_repo(make_repo(tmp_path), "r")
    assert result.outcome is SyncOutcome.FAILED
    assert result.reason.startswith("remote unreachable (pull):")


def test_unreadable_ahead_behind_status_does_not_claim_a_clean_fetch(monkeypatch, tmp_path, capsys):
    """`git status -sb` failing after a successful fetch is not a sync failure —
    the fetch worked — but the output must not imply we checked (Plan Review
    finding 8, left unsettled by the first implementation pass)."""
    _stub_git(monkeypatch, branch="feature/issue-1", fails={"status-sb"})
    _stub_network(monkeypatch, True, "")
    assert sync_repos.sync_repo(make_repo(tmp_path), "r").outcome is SyncOutcome.SYNCED
    assert "could not read ahead/behind status" in capsys.readouterr().out


def test_dirty_tree_is_a_benign_skip(monkeypatch, tmp_path):
    """Must NOT be FAILED — an intentionally dirty repo turning the run red is
    the false-red this design exists to avoid."""
    _stub_git(monkeypatch, dirty=True)
    assert sync_repos.sync_repo(make_repo(tmp_path), "r").outcome is SyncOutcome.SKIPPED


def test_detached_head_is_a_benign_skip(monkeypatch, tmp_path):
    """`git branch --show-current` succeeds and prints nothing."""
    _stub_git(monkeypatch, branch="")
    assert sync_repos.sync_repo(make_repo(tmp_path), "r").outcome is SyncOutcome.SKIPPED


def test_unreadable_git_state_is_failed(monkeypatch, tmp_path):
    """The git command itself failing (not a repo / corrupt .git) is NOT the
    same as a detached HEAD. Collapsing the two leaves a broken repo silently
    stale under a green exit."""
    _stub_git(monkeypatch, fails={"branch"})
    assert sync_repos.sync_repo(make_repo(tmp_path), "r").outcome is SyncOutcome.FAILED


def test_unreadable_working_tree_is_failed(monkeypatch, tmp_path):
    """`git status --porcelain` failing means the working tree could not be
    read — NOT that it is clean. Treating it as clean let a repo with a corrupt
    index and real uncommitted changes sync-and-report green (#609)."""
    _stub_git(monkeypatch, dirty=True, fails={"status"})
    _stub_network(monkeypatch, True, "Already up to date.")
    assert sync_repos.sync_repo(make_repo(tmp_path), "r").outcome is SyncOutcome.FAILED


def test_is_dirty_distinguishes_unreadable_from_clean(monkeypatch, tmp_path):
    """The three states must stay distinct at the helper level too."""
    repo = make_repo(tmp_path)
    _stub_git(monkeypatch, dirty=False)
    assert sync_repos.is_dirty(repo) is False
    _stub_git(monkeypatch, dirty=True)
    assert sync_repos.is_dirty(repo) is True
    _stub_git(monkeypatch, fails={"status"})
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
    assert sync_repos.sync_repo(tmp_path / "nope", "r").outcome is SyncOutcome.FAILED


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


def failed_result(reason):
    return sync_repos.SyncResult(SyncOutcome.FAILED, reason)


def skipped_result(reason):
    return sync_repos.SyncResult(SyncOutcome.SKIPPED, reason)


def repo_record(name, layer="core"):
    """A repo record shaped like get_overlay_repos() returns: main() maps
    source_file (`core.repos`) to the workspace dir (`core_ws`)."""
    return {"name": name, "source_file": f"{layer}.repos"}


def declare_optional(workspace, *layers):
    """Write a real configs/manifest/optional_layers.txt under the tmp workspace."""
    manifest = workspace / "configs" / "manifest"
    manifest.mkdir(parents=True, exist_ok=True)
    (manifest / "optional_layers.txt").write_text("# comment\n\n" + "\n".join(layers) + "\n")


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
        lambda path, name, dry_run=False: outcomes.get(
            name, sync_repos.SyncResult(SyncOutcome.SYNCED)
        ),
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
    # Exact counts: asserting only "0 failures" let a mutation that drops the
    # synced/skipped tallies pass. The root repo plus a and b = 3 synced.
    assert "✅ Sync complete — 3 synced, 0 skipped, 0 failures." in out
    assert "❌" not in out


def test_a_real_failure_exits_nonzero(monkeypatch, tmp_path, capsys):
    """The whole point: a failed repo must not report green."""
    _run_main(
        monkeypatch,
        make_workspace(tmp_path, monkeypatch),
        [repo_record("a"), repo_record("b")],
        {"b": failed_result("pull failed: Connection reset by peer")},
    )
    with pytest.raises(SystemExit) as exc:
        sync_repos.main()
    assert exc.value.code == 1
    out = capsys.readouterr().out
    assert "1 failure(s)" in out
    # Names the repo AND the actual cause — "sync failed" for every cause alike
    # tells the operator nothing about what to check.
    assert "- b: pull failed: Connection reset by peer" in out


def test_benign_skips_alone_exit_zero(monkeypatch, tmp_path, capsys):
    """A dirty or detached repo must not turn the run red."""
    _run_main(
        monkeypatch,
        make_workspace(tmp_path, monkeypatch),
        [repo_record("a"), repo_record("b")],
        {"a": skipped_result("uncommitted changes"), "b": skipped_result("detached HEAD")},
    )
    sync_repos.main()
    # Root repo synced; a and b skipped.
    assert "✅ Sync complete — 1 synced, 2 skipped, 0 failures." in capsys.readouterr().out


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


def test_absent_optional_layer_is_a_skip_not_a_failure(monkeypatch, tmp_path, capsys):
    """`site` is listed in optional_layers.txt and setup_layers.sh deletes the
    layer and exits 0 when its private repos are unreachable. Failing the run
    for that makes `make sync` permanently red on a supported host — the false
    red this design calls worse than the false green it removes."""
    workspace = make_workspace(tmp_path, monkeypatch)
    declare_optional(workspace, "site")
    _run_main(monkeypatch, workspace, [repo_record("a")], {})
    monkeypatch.setattr(
        sync_repos.list_overlay_repos,
        "get_overlay_repos",
        lambda include_underlay=False: [repo_record("a"), repo_record("private", layer="site")],
    )
    sync_repos.main()  # no SystemExit: exit 0
    out = capsys.readouterr().out
    assert "0 failures" in out
    assert "2 synced, 1 skipped" in out


def test_repo_missing_from_a_present_optional_layer_is_still_a_skip(monkeypatch, tmp_path, capsys):
    """A present layer directory does not mean the layer imported completely.

    setup_layers.sh exits 0 leaving a *partially* imported optional layer
    whenever it did not create the directory that run (`LAYER_DIR_EXISTED=true`
    — "preserve a previously-cloned layer"), and leaves an empty `src/` with a
    "Setup complete" message on a host with no `vcs`. Gating the carve-out on
    the directory would put `make sync` permanently red on both of those
    supported states — and disagree with validate_workspace.py, which allows any
    repo in an optional layer to be missing.
    """
    workspace = make_workspace(tmp_path, monkeypatch)
    declare_optional(workspace, "site")
    # Layer directory present but empty: the no-`vcs` state, and the shape a
    # partial import leaves behind.
    (workspace / "layers" / "main" / "site_ws" / "src").mkdir(parents=True)
    _run_main(monkeypatch, workspace, [], {})
    monkeypatch.setattr(
        sync_repos.list_overlay_repos,
        "get_overlay_repos",
        lambda include_underlay=False: [repo_record("private", layer="site")],
    )
    sync_repos.main()  # no SystemExit: exit 0
    out = capsys.readouterr().out
    assert "path not resolved" not in out
    assert "layer 'site' is optional on this host" in out
    assert "1 synced, 1 skipped" in out
    assert "0 failures" in out


def test_absent_required_layer_fails_and_says_it_is_not_set_up(monkeypatch, tmp_path, capsys):
    """A required layer that was never imported still fails the run — but names
    setup as the fix rather than reading as a network problem."""
    workspace = make_workspace(tmp_path, monkeypatch)
    declare_optional(workspace, "site")
    _run_main(monkeypatch, workspace, [], {})
    monkeypatch.setattr(
        sync_repos.list_overlay_repos,
        "get_overlay_repos",
        lambda include_underlay=False: [repo_record("needed", layer="platforms")],
    )
    with pytest.raises(SystemExit) as exc:
        sync_repos.main()
    assert exc.value.code == 1
    out = capsys.readouterr().out
    assert "not set up" in out
    assert "setup_layers.sh platforms" in out


def test_empty_repo_list_does_not_report_a_quantified_all_clear(monkeypatch, tmp_path, capsys):
    """When no .repos file is found (un-bootstrapped clone, or a workspace
    worktree with no configs/manifest), every configured repo goes
    unenumerated. Printing "1 synced, 0 skipped, 0 failures" for the root repo
    alone claims more than the bare success line this fix replaced."""
    _run_main(monkeypatch, make_workspace(tmp_path, monkeypatch), [], {})
    with pytest.raises(SystemExit) as exc:
        sync_repos.main()
    assert exc.value.code == 1
    out = capsys.readouterr().out
    assert "No repositories could be enumerated" in out
    assert "0 failures" not in out


def test_gitbug_runs_only_for_synced_repos(monkeypatch, tmp_path):
    """git-bug must not push against a repo whose pull just failed."""
    calls = _run_main(
        monkeypatch,
        make_workspace(tmp_path, monkeypatch),
        [repo_record("ok"), repo_record("bad"), repo_record("skip")],
        {"bad": failed_result("pull failed: boom"), "skip": skipped_result("detached HEAD")},
    )
    with pytest.raises(SystemExit):
        sync_repos.main()
    assert "bad" not in calls
    assert "skip" not in calls
    assert "ok" in calls


def test_root_workspace_repo_failure_fails_the_run(monkeypatch, tmp_path, capsys):
    """The root repo is synced from its own call site, and it is the repo
    merge_pr.sh has just merged into. Without this, reverting that call site to
    the `if sync_repo(...)` truthiness trap — every SyncOutcome and SyncResult
    is truthy — left the whole suite green."""
    calls = _run_main(
        monkeypatch,
        make_workspace(tmp_path, monkeypatch),
        [repo_record("a")],
        {"ros2_agent_workspace": failed_result("pull failed: Connection reset by peer")},
    )
    with pytest.raises(SystemExit) as exc:
        sync_repos.main()
    assert exc.value.code == 1
    out = capsys.readouterr().out
    assert "- ros2_agent_workspace: pull failed: Connection reset by peer" in out
    assert "(1 synced, 0 skipped)" in out
    # git-bug must not push against the repo whose pull just failed; the tmp
    # workspace root is named "ws".
    assert "ws" not in calls


def test_root_workspace_repo_skip_is_not_a_failure(monkeypatch, tmp_path, capsys):
    """A dirty workspace root is a benign skip like any other repo — it must not
    be tallied as synced either."""
    _run_main(
        monkeypatch,
        make_workspace(tmp_path, monkeypatch),
        [repo_record("a")],
        {"ros2_agent_workspace": skipped_result("uncommitted changes")},
    )
    sync_repos.main()
    assert "✅ Sync complete — 1 synced, 1 skipped, 0 failures." in capsys.readouterr().out


def test_mixed_run_counts_every_category(monkeypatch, tmp_path, capsys):
    _run_main(
        monkeypatch,
        make_workspace(tmp_path, monkeypatch),
        [repo_record("a"), repo_record("b"), repo_record("c")],
        {"b": skipped_result("detached HEAD"), "c": failed_result("pull failed: boom")},
    )
    with pytest.raises(SystemExit):
        sync_repos.main()
    out = capsys.readouterr().out
    # 'a' plus the root repo both synced; 'b' skipped; 'c' failed.
    assert "2 synced, 1 skipped" in out
