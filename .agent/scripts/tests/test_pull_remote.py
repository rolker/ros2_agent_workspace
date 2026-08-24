"""Tests for pull_remote.py's git-probe classification (#609).

`is_dirty()` returned `success and bool(output)`, so a `git status` that
*failed* was reported as "clean" — and this is the script that then MERGES on
that answer. `get_current_branch()` returned None for both a genuine detached
HEAD and a failed git command, so an unreadable repo was reported as a benign
skip and the run still exited 0. Both are the same defect class #609 removed
from sync_repos.py: a failed probe read as a benign answer.

The classification is exercised through `process_repo()` — the entry point
`iter_repos` actually calls, whose (status, message) pair is what
`print_summary_and_exit` turns into an exit code — rather than through the
module-private precondition helper, so the tests pin the contract operators
see. Everything is stubbed at the `run_git` / `run_git_network` seam: no test
touches a real repo, the network, or this host's `layers/`.
"""

import sys
from pathlib import Path
from types import SimpleNamespace

SCRIPTS_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SCRIPTS_DIR))
sys.path.insert(0, str(SCRIPTS_DIR / "lib"))

import pytest  # noqa: E402

import pull_remote  # noqa: E402
import remote_utils  # noqa: E402
from remote_utils import RemoteState  # noqa: E402

STATUS = ("status", "--porcelain")
BRANCH = ("branch", "--show-current")


def _stub_git(monkeypatch, responses):
    """Stub pull_remote's git runners with a probe table.

    `responses` maps the first two git arguments to (success, stdout). Anything
    not named succeeds with empty output, which is what the pass-through probes
    (rev-parse, fetch, merge, branch -f) need.

    A plain helper rather than a pytest fixture: a fixture's name and the test
    parameter receiving it are necessarily the same identifier, which pylint
    reports as redefined-outer-name (the convention test_sync_repos.py set).
    """

    def fake_run_git(repo_path, args, dry_run=False):
        success, out = responses.get(tuple(args[:2]), (True, ""))
        return success, out, "" if success else "fatal: stubbed failure"

    monkeypatch.setattr(pull_remote, "run_git", fake_run_git)
    monkeypatch.setattr(pull_remote, "run_git_network", fake_run_git)
    return fake_run_git


def _stub_repo(monkeypatch, responses, default_branch="main"):
    """Stub the whole per-repo environment: git probes, the default-branch
    lookup (its own fallback chain is out of scope here), and remote presence.
    """
    fake = _stub_git(monkeypatch, responses)
    monkeypatch.setattr(pull_remote, "get_default_branch", lambda repo_path, v: default_branch)
    monkeypatch.setattr(pull_remote, "remote_probe", lambda repo_path, remote: RemoteState.PRESENT)
    return fake


def _args(**kw):
    """The parsed argparse namespace process_repo reads."""
    fields = {"remote": "gitcloud", "pull": False, "branch": None, "dry_run": False}
    fields.update(kw)
    return SimpleNamespace(**fields)


# --------------------------------------------------------------------------
# is_dirty() — three states, not two
# --------------------------------------------------------------------------


def test_is_dirty_true_when_status_lists_changes(monkeypatch):
    _stub_git(monkeypatch, {STATUS: (True, " M file.txt")})
    assert pull_remote.is_dirty(Path("/repo")) is True


def test_is_dirty_false_when_status_is_empty(monkeypatch):
    _stub_git(monkeypatch, {STATUS: (True, "")})
    assert pull_remote.is_dirty(Path("/repo")) is False


def test_is_dirty_none_when_status_fails(monkeypatch):
    """The regression this exists for: a failed probe used to read as clean."""
    _stub_git(monkeypatch, {STATUS: (False, "")})
    assert pull_remote.is_dirty(Path("/repo")) is None


def test_is_dirty_none_even_when_a_failed_status_printed_changes(monkeypatch):
    """`success and bool(output)` and a bare `bool(output)` alike would call
    this clean-or-dirty; neither answer is knowable when the command failed."""
    _stub_git(monkeypatch, {STATUS: (False, " M file.txt")})
    assert pull_remote.is_dirty(Path("/repo")) is None


# --------------------------------------------------------------------------
# get_current_branch() — "" (detached) and None (unreadable) are different
# --------------------------------------------------------------------------


def test_current_branch_returns_the_name(monkeypatch):
    _stub_git(monkeypatch, {BRANCH: (True, "jazzy")})
    assert pull_remote.get_current_branch(Path("/repo")) == "jazzy"


def test_current_branch_is_empty_string_on_detached_head(monkeypatch):
    """git prints nothing and exits 0 on a detached HEAD — a benign skip."""
    _stub_git(monkeypatch, {BRANCH: (True, "")})
    assert pull_remote.get_current_branch(Path("/repo")) == ""


def test_current_branch_is_none_when_git_fails(monkeypatch):
    """Not a repo / corrupt .git. Used to be indistinguishable from detached."""
    _stub_git(monkeypatch, {BRANCH: (False, "")})
    assert pull_remote.get_current_branch(Path("/repo")) is None


# --------------------------------------------------------------------------
# --pull: the merge gate, through process_repo
# --------------------------------------------------------------------------


def test_unreadable_working_tree_is_an_error_not_a_skip(monkeypatch):
    """The most consequential site: --pull merges on is_dirty()'s answer."""
    _stub_repo(monkeypatch, {STATUS: (False, "")})
    status, msg = pull_remote.process_repo(Path("/repo"), "r", "main", _args(pull=True))
    assert (status, msg) == ("error", pull_remote.UNREADABLE_TREE)


def test_unreadable_git_state_is_an_error_not_a_detached_head_skip(monkeypatch):
    _stub_repo(monkeypatch, {STATUS: (True, ""), BRANCH: (False, "")})
    status, msg = pull_remote.process_repo(Path("/repo"), "r", "main", _args(pull=True))
    assert (status, msg) == ("error", pull_remote.UNREADABLE_STATE)


def test_detached_head_is_still_a_benign_skip(monkeypatch):
    """A real detached HEAD must stay exit-0: a red that fires on a repo the
    operator parked deliberately is a signal everyone learns to ignore."""
    _stub_repo(monkeypatch, {STATUS: (True, ""), BRANCH: (True, "")})
    status, msg = pull_remote.process_repo(Path("/repo"), "r", "main", _args(pull=True))
    assert (status, msg) == ("skip", "detached HEAD — skipping merge")


def test_dirty_tree_is_still_a_benign_skip(monkeypatch):
    _stub_repo(monkeypatch, {STATUS: (True, " M f")})
    status, msg = pull_remote.process_repo(Path("/repo"), "r", "main", _args(pull=True))
    assert (status, msg) == ("skip", "uncommitted changes — skipping merge")


def test_wrong_branch_is_still_a_benign_skip(monkeypatch):
    _stub_repo(monkeypatch, {STATUS: (True, ""), BRANCH: (True, "feature/issue-1")})
    status, msg = pull_remote.process_repo(Path("/repo"), "r", "main", _args(pull=True))
    assert status == "skip"
    assert "feature/issue-1" in msg


def test_clean_repo_on_default_branch_merges(monkeypatch):
    """The happy path still reaches the merge — the guards did not swallow it."""
    _stub_repo(monkeypatch, {STATUS: (True, ""), BRANCH: (True, "main")})
    status, msg = pull_remote.process_repo(Path("/repo"), "r", "main", _args(pull=True))
    assert status == "ok"
    assert "gitcloud/main" in msg


def test_pull_mode_never_reaches_the_merge_when_the_tree_is_unreadable(monkeypatch):
    """The point of the fix: no `git merge` and no `git fetch` run on a working
    tree we could not read. Recorded, not inferred from the status string."""
    ran = []

    def recording_run_git(repo_path, args, dry_run=False):
        ran.append(args[0])
        if tuple(args[:2]) == STATUS:
            return False, "", "fatal: stubbed failure"
        return True, "", ""

    monkeypatch.setattr(pull_remote, "run_git", recording_run_git)
    monkeypatch.setattr(pull_remote, "run_git_network", recording_run_git)
    monkeypatch.setattr(pull_remote, "get_default_branch", lambda repo_path, v: "main")
    monkeypatch.setattr(pull_remote, "remote_probe", lambda repo_path, remote: RemoteState.PRESENT)
    pull_remote.process_repo(Path("/repo"), "r", "main", _args(pull=True))
    assert "merge" not in ran
    assert "fetch" not in ran


# --------------------------------------------------------------------------
# --branch: the same probe, a different consequence
# --------------------------------------------------------------------------


def test_branch_mode_errors_when_the_current_branch_is_unreadable(monkeypatch):
    """--branch force-updates a branch; if we cannot read which branch is
    checked out we cannot know `git branch -f` is safe."""
    _stub_repo(monkeypatch, {BRANCH: (False, "")})
    status, msg = pull_remote.process_repo(
        Path("/repo"), "r", "main", _args(branch="sync/gitcloud")
    )
    assert (status, msg) == ("error", pull_remote.UNREADABLE_STATE)


def test_branch_mode_still_skips_when_the_target_is_checked_out(monkeypatch):
    """The pre-existing benign skip must survive the None carve-out."""
    _stub_repo(monkeypatch, {BRANCH: (True, "sync/gitcloud")})
    status, msg = pull_remote.process_repo(
        Path("/repo"), "r", "main", _args(branch="sync/gitcloud")
    )
    assert status == "skip"
    assert "currently checked out" in msg


def test_branch_mode_updates_the_branch_when_the_state_is_readable(monkeypatch):
    _stub_repo(monkeypatch, {BRANCH: (True, "main")})
    status, msg = pull_remote.process_repo(
        Path("/repo"), "r", "main", _args(branch="sync/gitcloud")
    )
    assert status == "ok"
    assert "sync/gitcloud" in msg


# --------------------------------------------------------------------------
# Status string -> process exit status
# --------------------------------------------------------------------------


def test_error_status_makes_the_summary_exit_non_zero():
    """Closes the loop from status string to process exit status, so the
    "error" classification above is not just a label."""
    with pytest.raises(SystemExit) as exc:
        remote_utils.print_summary_and_exit({"ok": 1, "skip": 1, "error": 1}, [("error", "errors")])
    assert exc.value.code == 1


def test_skip_status_leaves_the_summary_at_exit_zero():
    """The other direction: benign skips must not turn the run red."""
    remote_utils.print_summary_and_exit({"ok": 1, "skip": 2, "error": 0}, [("skip", "skipped")])


# --------------------------------------------------------------------------
# remote_probe(): "we could not look" is not "the remote is not configured"
# --------------------------------------------------------------------------


def test_unreadable_remote_is_an_error_not_a_missing_remote_skip(monkeypatch):
    """`git remote` failing meant we never learned whether the remote exists;
    "remote not found" kept a repo that was never fetched inside a green run."""
    _stub_git(monkeypatch, {})
    monkeypatch.setattr(pull_remote, "get_default_branch", lambda repo_path, v: "main")
    monkeypatch.setattr(
        pull_remote, "remote_probe", lambda repo_path, remote: RemoteState.UNREADABLE
    )
    status, msg = pull_remote.process_repo(Path("/repo"), "r", "main", _args(pull=True))
    assert (status, msg) == ("error", pull_remote.UNREADABLE_STATE)


def test_absent_remote_is_still_a_benign_skip(monkeypatch):
    """Most repos legitimately have no secondary remote — that must stay 0."""
    _stub_git(monkeypatch, {})
    monkeypatch.setattr(pull_remote, "get_default_branch", lambda repo_path, v: "main")
    monkeypatch.setattr(pull_remote, "remote_probe", lambda repo_path, remote: RemoteState.ABSENT)
    status, msg = pull_remote.process_repo(Path("/repo"), "r", "main", _args(pull=True))
    assert status == "skip"
    assert "not found" in msg


def test_an_unusable_repo_directory_lands_as_an_error_not_a_clean_tree(monkeypatch):
    """End to end for the pair: the OSError becomes a failed probe, and the
    failed probe becomes an error rather than "clean, nothing to merge"."""

    def boom(*_a, **_kw):
        raise OSError(2, "No such file or directory")

    monkeypatch.setattr(remote_utils.subprocess, "run", boom)
    monkeypatch.setattr(pull_remote, "get_default_branch", lambda repo_path, v: "main")
    monkeypatch.setattr(pull_remote, "remote_probe", lambda repo_path, remote: RemoteState.PRESENT)
    status, msg = pull_remote.process_repo(Path("/gone"), "r", "main", _args(pull=True))
    assert (status, msg) == ("error", pull_remote.UNREADABLE_TREE)


def test_json_mode_reports_an_unreadable_repo_instead_of_silently_skipping(monkeypatch, tmp_path):
    """--json's loop `continue`d on a falsy remote check, so a repo whose git
    state could not be read vanished from both the report and the errors, and
    the process exited 0 with a JSON document that looked complete (#609)."""
    monkeypatch.setattr(pull_remote, "SCRIPT_DIR", tmp_path / ".agent" / "scripts")
    monkeypatch.setattr(
        pull_remote, "get_repos", lambda args: [{"name": "alpha", "version": "jazzy"}]
    )
    monkeypatch.setattr(pull_remote, "resolve_repo_path", lambda root, repo: tmp_path / "alpha")
    monkeypatch.setattr(pull_remote, "get_default_branch", lambda repo_path, v: "main")
    monkeypatch.setattr(pull_remote, "_json_report", lambda *a: None)
    monkeypatch.setattr(
        pull_remote, "run_git_network", lambda repo_path, args, dry_run=False: (True, "", "")
    )
    monkeypatch.setattr(
        pull_remote, "remote_probe", lambda repo_path, remote: RemoteState.UNREADABLE
    )
    monkeypatch.setattr(sys, "argv", ["pull_remote.py", "--remote", "gitcloud", "--json"])
    with pytest.raises(SystemExit) as exc:
        pull_remote.main()
    assert exc.value.code == 1


def test_json_mode_stays_silent_about_a_repo_with_no_such_remote(monkeypatch, tmp_path):
    """The other direction: a repo that simply has no secondary remote is not
    an error, and must not turn --json red."""
    monkeypatch.setattr(pull_remote, "SCRIPT_DIR", tmp_path / ".agent" / "scripts")
    monkeypatch.setattr(
        pull_remote, "get_repos", lambda args: [{"name": "alpha", "version": "jazzy"}]
    )
    monkeypatch.setattr(pull_remote, "resolve_repo_path", lambda root, repo: tmp_path / "alpha")
    monkeypatch.setattr(pull_remote, "get_default_branch", lambda repo_path, v: "main")
    monkeypatch.setattr(pull_remote, "_json_report", lambda *a: None)
    monkeypatch.setattr(
        pull_remote, "run_git_network", lambda repo_path, args, dry_run=False: (True, "", "")
    )
    monkeypatch.setattr(pull_remote, "remote_probe", lambda repo_path, remote: RemoteState.ABSENT)
    monkeypatch.setattr(sys, "argv", ["pull_remote.py", "--remote", "gitcloud", "--json"])
    with pytest.raises(SystemExit) as exc:
        pull_remote.main()
    assert exc.value.code == 0
