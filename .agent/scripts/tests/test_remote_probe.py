"""Tests for the shared git-probe seams in lib/remote_utils.py (#609).

Two false greens lived here, one level below pull_remote.py's:

  - `remote_exists()` answered False when `git remote` itself failed, so both
    pull_remote.py and push_remote.py reported "remote 'gitcloud' not found"
    and skipped — green — for a repo they had never been able to read.
  - `run_git()` caught only CalledProcessError, so an unusable repo directory
    (unreadable, vanished mid-run, a dangling symlink) raised OSError, escaped,
    and killed the whole run with no summary, stranding every later repo. This
    is the hole sync_repos.py's run_git_cmd closed on the same branch.

All subprocess activity is stubbed; nothing touches a real repo.
"""

import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

SCRIPTS_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SCRIPTS_DIR))
sys.path.insert(0, str(SCRIPTS_DIR / "lib"))

import push_remote  # noqa: E402
import remote_utils  # noqa: E402
from remote_utils import RemoteState  # noqa: E402


def _stub_run_git(monkeypatch, success, output):
    monkeypatch.setattr(
        remote_utils,
        "run_git",
        lambda repo_path, args, dry_run=False: (success, output, "" if success else "fatal"),
    )


def test_remote_probe_finds_a_configured_remote(monkeypatch):
    _stub_run_git(monkeypatch, True, "origin\ngitcloud")
    assert remote_utils.remote_probe(Path("/repo"), "gitcloud") is RemoteState.PRESENT


def test_remote_probe_reports_a_genuinely_absent_remote(monkeypatch):
    _stub_run_git(monkeypatch, True, "origin")
    assert remote_utils.remote_probe(Path("/repo"), "gitcloud") is RemoteState.ABSENT


def test_remote_probe_reports_an_unreadable_repo_separately(monkeypatch):
    """The regression: this used to be indistinguishable from ABSENT."""
    _stub_run_git(monkeypatch, False, "")
    assert remote_utils.remote_probe(Path("/repo"), "gitcloud") is RemoteState.UNREADABLE


def test_remote_exists_stays_boolean_for_add_remote(monkeypatch):
    """add_remote.py skips when the remote is already there and otherwise goes
    on to read the origin URL, reporting an explicit error when that fails — so
    False is the right answer for an unreadable repo there, and the boolean
    wrapper is kept rather than changed under its one remaining caller."""
    _stub_run_git(monkeypatch, False, "")
    assert remote_utils.remote_exists(Path("/repo"), "gitcloud") is False
    _stub_run_git(monkeypatch, True, "gitcloud")
    assert remote_utils.remote_exists(Path("/repo"), "gitcloud") is True


def test_run_git_reports_an_unusable_repo_directory_instead_of_raising(monkeypatch):
    def boom(*_a, **_kw):
        raise OSError(2, "No such file or directory")

    monkeypatch.setattr(remote_utils.subprocess, "run", boom)
    success, _out, err = remote_utils.run_git(Path("/gone"), ["status", "--porcelain"])
    assert success is False
    assert "cannot run git in" in err


def test_run_git_still_reports_a_normal_git_failure(monkeypatch):
    """The OSError arm must not swallow the CalledProcessError one."""

    def fail(*_a, **_kw):
        raise subprocess.CalledProcessError(128, "git", output="", stderr="fatal: not a repo")

    monkeypatch.setattr(remote_utils.subprocess, "run", fail)
    success, _out, err = remote_utils.run_git(Path("/repo"), ["status"])
    assert success is False
    assert "not a repo" in err


def _push_args():
    return SimpleNamespace(
        remote="gitcloud", dry_run=False, all_branches=False, tags=False, force=False
    )


def test_push_reports_an_unreadable_repo_as_an_error(monkeypatch):
    """push_remote.py carried the identical false skip: a repo it could not
    read was reported as "remote not found" and never pushed, green."""
    monkeypatch.setattr(
        push_remote, "remote_probe", lambda repo_path, remote: RemoteState.UNREADABLE
    )
    status, msg = push_remote.process_repo(Path("/repo"), "r", "main", _push_args())
    assert status == "error"
    assert "cannot read git state" in msg


def test_push_still_skips_a_repo_with_no_such_remote(monkeypatch):
    monkeypatch.setattr(push_remote, "remote_probe", lambda repo_path, remote: RemoteState.ABSENT)
    status, msg = push_remote.process_repo(Path("/repo"), "r", "main", _push_args())
    assert status == "skip"
    assert "not found" in msg
