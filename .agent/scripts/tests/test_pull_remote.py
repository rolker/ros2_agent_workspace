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
    monkeypatch.setattr(pull_remote, "json_report", lambda *a: None)
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
    monkeypatch.setattr(pull_remote, "json_report", lambda *a: None)
    monkeypatch.setattr(
        pull_remote, "run_git_network", lambda repo_path, args, dry_run=False: (True, "", "")
    )
    monkeypatch.setattr(pull_remote, "remote_probe", lambda repo_path, remote: RemoteState.ABSENT)
    monkeypatch.setattr(sys, "argv", ["pull_remote.py", "--remote", "gitcloud", "--json"])
    with pytest.raises(SystemExit) as exc:
        pull_remote.main()
    assert exc.value.code == 0


# --------------------------------------------------------------------------
# --json: a failed probe is not "no field changes to import" (#609)
#
# The sole consumer is the /import-field-changes skill, which is instructed to
# stop when the report is empty. A repo silently dropped here is a field
# hotfix that never reconciles to GitHub, reported green.
# --------------------------------------------------------------------------

REV_PARSE = ("rev-parse", "--verify")
REV_LIST = ("rev-list", "--left-right")
LOG = ("log", "--oneline")


def _report_git(monkeypatch, table):
    """Stub run_git with a table keyed on the *full* argv tuple.

    json_report distinguishes the remote ref from the local branch with two
    otherwise-identical `rev-parse --verify` calls, so these tests cannot key
    on the first two arguments the way the classification tests do.
    """

    def fake_run_git(repo_path, args, dry_run=False):
        success, out = table.get(tuple(args), (True, ""))
        return success, out, "" if success else "fatal: stubbed failure"

    monkeypatch.setattr(pull_remote, "run_git", fake_run_git)
    monkeypatch.setattr(pull_remote, "get_default_branch", lambda repo_path, v: "jazzy")


def test_json_report_errors_when_rev_list_fails(monkeypatch):
    """The reproduced trigger: rev-list exits 128 and the repo vanished from
    both the report and the errors, leaving --json exit 0."""
    _report_git(
        monkeypatch,
        {("rev-list", "--left-right", "--count", "jazzy...gitcloud/jazzy"): (False, "")},
    )
    entry, err = pull_remote.json_report(Path("/repo"), "alpha", "jazzy", "gitcloud")
    assert entry is None
    assert err and "jazzy...gitcloud/jazzy" in err


def test_json_report_names_the_no_local_branch_state_without_erroring(monkeypatch):
    """What produces that rc 128: a default branch that resolved through
    refs/remotes/origin/ only — what `vcs import` leaves for a SHA- or
    tag-pinned manifest entry. A normal supported state, so it is neither an
    error (it was one, while default mode skipped the identical repo) nor a
    silence: the consumer must see the state, not infer "nothing to import"."""
    _report_git(monkeypatch, {("rev-parse", "--verify", "refs/heads/jazzy"): (False, "")})
    entry, err = pull_remote.json_report(Path("/repo"), "alpha", "jazzy", "gitcloud")
    assert err is None
    assert entry["state"] == pull_remote.STATE_NO_LOCAL_BRANCH
    assert entry["repo"] == "alpha" and entry["remote_ref"] == "gitcloud/jazzy"
    assert "no local 'jazzy'" in entry["detail"]
    assert "commits" not in entry  # nothing was compared, so nothing is claimed


def test_json_report_no_local_branch_does_not_turn_the_run_red(monkeypatch, tmp_path, capsys):
    """End to end for the operator-decided contract: reported, not failed. It
    reaches stdout as its own entry and leaves the exit status at 0."""
    code = _json_main(
        monkeypatch,
        tmp_path,
        repos=[{"name": "alpha", "version": "jazzy", "source_file": "core.repos"}],
        report=lambda *a: (
            {"repo": "alpha", "state": pull_remote.STATE_NO_LOCAL_BRANCH, "detail": "d"},
            None,
        ),
    )
    out = capsys.readouterr()
    assert code == 0
    assert pull_remote.STATE_NO_LOCAL_BRANCH in out.out
    assert out.err == ""


def _default_mode(monkeypatch, table):
    """Drive the *default* (fetch-and-report) mode through process_repo().

    Default mode is what `make pull-remote` runs and what dashboard reads; its
    comparison arm had no test at all, so both of _compare_branches' error
    returns survived mutation to a bare `("ok", "fetched")`.
    """
    _report_git(monkeypatch, table)
    monkeypatch.setattr(
        pull_remote, "run_git_network", lambda repo_path, args, dry_run=False: (True, "", "")
    )
    monkeypatch.setattr(pull_remote, "remote_probe", lambda repo_path, remote: RemoteState.PRESENT)
    return pull_remote.process_repo(Path("/repo"), "alpha", "jazzy", _args())


def test_default_mode_errors_when_rev_list_fails(monkeypatch):
    """Both refs verified, so a failing rev-list means we could not read the
    repo — and a bare "fetched" would claim we checked whether the remote is
    ahead. Surviving mutation from Round 4: this arm had no test."""
    status, msg = _default_mode(
        monkeypatch,
        {("rev-list", "--left-right", "--count", "jazzy...gitcloud/jazzy"): (False, "")},
    )
    assert status == "error"
    assert "cannot compare jazzy...gitcloud/jazzy" in msg


def test_default_mode_errors_on_unreadable_rev_list_output(monkeypatch):
    """The other surviving mutation: a successful rev-list whose output is not
    two integers is not a comparison either."""
    status, msg = _default_mode(
        monkeypatch,
        {("rev-list", "--left-right", "--count", "jazzy...gitcloud/jazzy"): (True, "garbage")},
    )
    assert status == "error"
    assert "unreadable rev-list output" in msg


def test_default_mode_reports_up_to_date_when_the_comparison_succeeds(monkeypatch):
    """False-RED direction for the pair above: a repo level with its remote
    must stay "ok", or every clean run of `make pull-remote` goes red."""
    status, msg = _default_mode(
        monkeypatch,
        {("rev-list", "--left-right", "--count", "jazzy...gitcloud/jazzy"): (True, "0\t0")},
    )
    assert (status, msg) == ("ok", "up to date")


def test_default_mode_skips_a_remote_that_has_no_such_branch(monkeypatch):
    """Benign: nothing to compare against, and not this repo's problem."""
    status, msg = _default_mode(
        monkeypatch,
        {("rev-parse", "--verify", "refs/remotes/gitcloud/jazzy"): (False, "")},
    )
    assert status == "skip"
    assert "no gitcloud/jazzy on remote" in msg


def test_both_modes_agree_on_the_no_local_branch_state(monkeypatch):
    """The false RED this must-fix exists for: the same repo in the same state
    was an error in --json and a benign skip in default mode. Pinned together,
    through each mode's real entry point, so the two cannot drift apart."""
    table = {("rev-parse", "--verify", "refs/heads/jazzy"): (False, "")}
    status, msg = _default_mode(monkeypatch, table)
    entry, err = pull_remote.json_report(Path("/repo"), "alpha", "jazzy", "gitcloud")
    assert status == "skip"  # default mode: exits 0 via print_summary_and_exit
    assert err is None  # --json: exits 0 too
    assert entry["state"] == pull_remote.STATE_NO_LOCAL_BRANCH
    assert msg.endswith(f"({entry['detail']})")  # and they say the same thing


def test_json_report_ahead_entry_is_labelled_and_marks_truncation(monkeypatch):
    """`commits` is capped at 50; without a marker a consumer diffing
    len(commits) against `behind` sees an unexplained shortfall."""
    _report_git(
        monkeypatch,
        {
            ("rev-list", "--left-right", "--count", "jazzy...gitcloud/jazzy"): (True, "0\t60"),
            ("log", "--oneline", "--max-count=50", "jazzy..gitcloud/jazzy"): (
                True,
                "\n".join(f"sha{i} subject {i}" for i in range(50)),
            ),
        },
    )
    entry, err = pull_remote.json_report(Path("/repo"), "alpha", "jazzy", "gitcloud")
    assert err is None
    assert entry["state"] == pull_remote.STATE_AHEAD
    assert entry["commits_truncated"] is True


def test_json_report_does_not_claim_truncation_when_every_commit_is_listed(monkeypatch):
    """The false-RED direction of the marker: an untruncated list must not
    tell the consumer commits are missing."""
    _report_git(
        monkeypatch,
        {
            ("rev-list", "--left-right", "--count", "jazzy...gitcloud/jazzy"): (True, "0\t2"),
            ("log", "--oneline", "--max-count=50", "jazzy..gitcloud/jazzy"): (
                True,
                "abc123 one\ndef456 two",
            ),
        },
    )
    entry, _ = pull_remote.json_report(Path("/repo"), "alpha", "jazzy", "gitcloud")
    assert entry["commits_truncated"] is False


def test_local_branch_probe_does_not_accept_a_tag_of_the_same_name(monkeypatch):
    """The probe was a loose `rev-parse --verify <branch>`, which also matches
    refs/tags/<branch> — and the rev-list would then compare the tag."""
    seen = []

    def fake_run_git(repo_path, args, dry_run=False):
        seen.append(tuple(args))
        return True, "0\t0", ""

    monkeypatch.setattr(pull_remote, "run_git", fake_run_git)
    monkeypatch.setattr(pull_remote, "get_default_branch", lambda repo_path, v: "jazzy")
    pull_remote.json_report(Path("/repo"), "alpha", "jazzy", "gitcloud")
    assert ("rev-parse", "--verify", "refs/heads/jazzy") in seen
    assert ("rev-parse", "--verify", "refs/remotes/gitcloud/jazzy") in seen


def test_json_report_is_silent_when_the_remote_has_no_such_branch(monkeypatch):
    """False-RED direction: a remote that simply does not carry the branch is
    a real answer — nothing to import — and must stay out of the errors."""
    _report_git(
        monkeypatch, {("rev-parse", "--verify", "refs/remotes/gitcloud/jazzy"): (False, "")}
    )
    assert pull_remote.json_report(Path("/repo"), "alpha", "jazzy", "gitcloud") == (None, None)


def test_json_report_is_silent_when_the_remote_is_not_ahead(monkeypatch):
    """The other benign answer: the branch exists on both sides and the remote
    carries nothing new."""
    _report_git(
        monkeypatch,
        {("rev-list", "--left-right", "--count", "jazzy...gitcloud/jazzy"): (True, "2\t0")},
    )
    assert pull_remote.json_report(Path("/repo"), "alpha", "jazzy", "gitcloud") == (None, None)


def test_json_report_returns_the_entry_when_the_remote_is_ahead(monkeypatch):
    """And the case the skill actually acts on, so none of the above can be
    satisfied by never producing an entry at all."""
    _report_git(
        monkeypatch,
        {
            ("rev-list", "--left-right", "--count", "jazzy...gitcloud/jazzy"): (True, "0\t2"),
            (
                "log",
                "--oneline",
                "--max-count=50",
                "jazzy..gitcloud/jazzy",
            ): (True, "abc123 field fix\ndef456 another"),
        },
    )
    entry, err = pull_remote.json_report(Path("/repo"), "alpha", "jazzy", "gitcloud")
    assert err is None
    assert entry["behind"] == 2 and entry["ahead"] == 0
    assert entry["diverged"] is False
    assert [c["sha"] for c in entry["commits"]] == ["abc123", "def456"]


def test_json_report_errors_when_the_commit_log_fails(monkeypatch):
    """We already know the remote is ahead here, so an empty commit list from a
    *failed* log would describe a repo with pending field commits as carrying
    none — the same collapse one probe further down."""
    _report_git(
        monkeypatch,
        {
            ("rev-list", "--left-right", "--count", "jazzy...gitcloud/jazzy"): (True, "0\t2"),
            ("log", "--oneline", "--max-count=50", "jazzy..gitcloud/jazzy"): (False, ""),
        },
    )
    entry, err = pull_remote.json_report(Path("/repo"), "alpha", "jazzy", "gitcloud")
    assert entry is None
    assert err and "cannot list" in err


def test_json_report_errors_on_unreadable_rev_list_output(monkeypatch):
    """A successful rev-list whose output is not two integers used to fall
    through to `return None`, and would now raise ValueError on int()."""
    _report_git(
        monkeypatch,
        {("rev-list", "--left-right", "--count", "jazzy...gitcloud/jazzy"): (True, "garbage")},
    )
    entry, err = pull_remote.json_report(Path("/repo"), "alpha", "jazzy", "gitcloud")
    assert entry is None
    assert err and "unreadable rev-list output" in err


def _json_main(monkeypatch, tmp_path, **opts):
    """Drive main() in --json mode with the workspace fully stubbed.

    Options: `repos` (the enumeration), `report` (the json_report stub),
    `probe` (per-path RemoteState), `resolve` (resolve_repo_path stub),
    `optional_layers` (declared optional on this host).
    """
    monkeypatch.setattr(pull_remote, "SCRIPT_DIR", tmp_path / ".agent" / "scripts")
    monkeypatch.setattr(pull_remote, "get_repos", lambda args: list(opts.get("repos", [])))
    monkeypatch.setattr(
        pull_remote,
        "resolve_repo_path",
        opts.get("resolve", lambda root, repo: tmp_path / repo["name"]),
    )
    monkeypatch.setattr(pull_remote, "get_default_branch", lambda repo_path, v: "jazzy")
    if "optional_layers" in opts:
        monkeypatch.setattr(
            pull_remote, "get_optional_layers", lambda root: set(opts["optional_layers"])
        )
    else:
        monkeypatch.setattr(pull_remote, "get_optional_layers", lambda root: set())
    monkeypatch.setattr(pull_remote, "json_report", opts.get("report", lambda *a: (None, None)))
    monkeypatch.setattr(
        pull_remote, "run_git_network", lambda repo_path, args, dry_run=False: (True, "", "")
    )
    monkeypatch.setattr(
        pull_remote,
        "remote_probe",
        opts.get("probe", lambda repo_path, remote: RemoteState.PRESENT),
    )
    monkeypatch.setattr(sys, "argv", ["pull_remote.py", "--remote", "gitcloud", "--json"])
    with pytest.raises(SystemExit) as exc:
        pull_remote.main()
    return exc.value.code


def test_json_mode_exits_non_zero_when_a_report_probe_failed(monkeypatch, tmp_path, capsys):
    """End to end for must-fix 3: the failed probe reaches the exit status and
    names the repo on stderr, instead of leaving a gap in a green report."""
    code = _json_main(
        monkeypatch,
        tmp_path,
        repos=[{"name": "alpha", "version": "jazzy", "source_file": "core.repos"}],
        report=lambda *a: (None, "cannot compare jazzy...gitcloud/jazzy: fatal"),
    )
    assert code == 1
    assert "alpha: cannot compare" in capsys.readouterr().err


def test_json_mode_still_exits_zero_on_a_clean_report(monkeypatch, tmp_path):
    """False-RED direction: a workspace where every repo genuinely has nothing
    to import must stay 0, or /import-field-changes fails on every run."""
    code = _json_main(
        monkeypatch,
        tmp_path,
        repos=[{"name": "alpha", "version": "jazzy", "source_file": "core.repos"}],
    )
    assert code == 0


def test_json_mode_errors_when_no_repos_were_enumerated(monkeypatch, tmp_path, capsys):
    """Run from a workspace worktree, --json printed `[]` and exited 0 — which
    /import-field-changes reads as "no field changes to import" for the entire
    workspace."""
    code = _json_main(monkeypatch, tmp_path, repos=[])
    assert code == 1
    assert "configs/manifest" in capsys.readouterr().err


def test_json_mode_errors_on_a_repo_missing_from_a_required_layer(monkeypatch, tmp_path, capsys):
    """A configured repo with no checkout was `continue`d over silently."""
    code = _json_main(
        monkeypatch,
        tmp_path,
        repos=[{"name": "alpha", "version": "jazzy", "source_file": "core.repos"}],
        resolve=lambda root, repo: None,
    )
    assert code == 1
    assert "alpha: no local checkout" in capsys.readouterr().err


def test_json_mode_stays_green_for_a_repo_missing_from_an_optional_layer(monkeypatch, tmp_path):
    """The field-facing false-RED guard: `site` is optional on every manifest
    this workspace ships and absent on salmon/gabby."""
    code = _json_main(
        monkeypatch,
        tmp_path,
        repos=[{"name": "alpha", "version": "jazzy", "source_file": "site.repos"}],
        resolve=lambda root, repo: None,
        optional_layers={"site"},
    )
    assert code == 0


def test_json_mode_skips_a_workspace_root_with_no_such_remote(monkeypatch, tmp_path, capsys):
    """The root was fetched without a remote_probe, so a workspace whose root
    has no secondary remote reported a fetch failure where non-JSON mode
    benignly skips it — contradicting the documented contract."""

    def fetch_fails(repo_path, args, dry_run=False):
        return False, "", "fatal: 'gitcloud' does not appear to be a git repository"

    monkeypatch.setattr(pull_remote, "run_git_network", fetch_fails, raising=True)
    code = _json_main(
        monkeypatch,
        tmp_path,
        repos=[],
        probe=lambda repo_path, remote: RemoteState.ABSENT,
    )
    # Exit 1 is from the empty enumeration above, not from the root's fetch.
    assert code == 1
    assert "ros2_agent_workspace" not in capsys.readouterr().err
