"""Tests for the transient-network retry logic (issue #579 field import).

Covers lib/remote_utils.py (TRANSIENT_ERRORS, retry_transient, run_git_network)
and sync_repos.py's delegation (run_network_cmd, sync_gitbug routing). All
subprocess/network activity is stubbed; time.sleep is patched out.
"""

import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SCRIPTS_DIR))
sys.path.insert(0, str(SCRIPTS_DIR / "lib"))

import pytest  # noqa: E402

import remote_utils  # noqa: E402
import sync_repos  # noqa: E402


@pytest.fixture(autouse=True)
def reset_transient_seen():
    """Isolate the module-level dropped-connection flag between tests."""
    remote_utils.reset_transient_error_seen()
    yield
    remote_utils.reset_transient_error_seen()


def trip_transient_flag(monkeypatch):
    """Set the dropped-connection flag the way production code does: by
    running a network op through retry_transient that hits a drop."""
    monkeypatch.setattr(remote_utils.time, "sleep", lambda _s: None)
    drop = (False, "Connection reset by peer")
    remote_utils.retry_transient(make_run_stub([drop, drop]), "arg")
    assert remote_utils.transient_error_seen()


def make_run_stub(results):
    """Callable standing in for run_git / run_git_cmd; returns queued results
    and records every call on the returned function's .calls attribute."""
    queue = list(results)
    calls = []

    def stub(*args):
        calls.append(args)
        return queue.pop(0)

    stub.calls = calls
    return stub


class TestIsTransientError:
    def test_matches_every_signature(self):
        for fragment in remote_utils.TRANSIENT_ERRORS:
            assert remote_utils.is_transient_error(f"blah: {fragment}: blah")

    def test_rejects_ordinary_errors(self):
        assert not remote_utils.is_transient_error("fatal: not a git repository")
        assert not remote_utils.is_transient_error("")


class TestRetryTransient:
    def test_success_first_try_no_retry(self, monkeypatch):
        sleeps = []
        monkeypatch.setattr(remote_utils.time, "sleep", sleeps.append)
        stub = make_run_stub([(True, "ok")])
        assert remote_utils.retry_transient(stub, "arg") == (True, "ok")
        assert len(stub.calls) == 1
        assert not sleeps

    def test_transient_failure_retries_once_after_backoff(self, monkeypatch):
        sleeps = []
        monkeypatch.setattr(remote_utils.time, "sleep", sleeps.append)
        stub = make_run_stub(
            [
                (False, "kex_exchange_identification: read: Connection reset by peer"),
                (True, "recovered"),
            ]
        )
        assert remote_utils.retry_transient(stub, "arg") == (True, "recovered")
        assert len(stub.calls) == 2
        assert stub.calls[0] == stub.calls[1] == ("arg",)
        assert sleeps == [remote_utils.RETRY_BACKOFF]

    def test_transient_failure_retries_only_once(self, monkeypatch):
        monkeypatch.setattr(remote_utils.time, "sleep", lambda _s: None)
        drop = (False, "Connection closed by remote host")
        stub = make_run_stub([drop, drop])
        assert remote_utils.retry_transient(stub, "arg") == drop
        assert len(stub.calls) == 2

    def test_non_transient_failure_no_retry(self, monkeypatch):
        sleeps = []
        monkeypatch.setattr(remote_utils.time, "sleep", sleeps.append)
        stub = make_run_stub([(False, "fatal: repository not found")])
        success, output = remote_utils.retry_transient(stub, "arg")
        assert not success and output == "fatal: repository not found"
        assert len(stub.calls) == 1
        assert not sleeps

    def test_transient_failure_sets_seen_flag(self, monkeypatch):
        monkeypatch.setattr(remote_utils.time, "sleep", lambda _s: None)
        assert not remote_utils.transient_error_seen()
        stub = make_run_stub([(False, "Connection reset by peer"), (True, "recovered")])
        remote_utils.retry_transient(stub, "arg")
        # Flag stays set even though the retry recovered — the remote IS
        # rate-limiting, so subsequent operations should be paced.
        assert remote_utils.transient_error_seen()

    def test_success_and_ordinary_failure_leave_flag_clear(self, monkeypatch):
        monkeypatch.setattr(remote_utils.time, "sleep", lambda _s: None)
        remote_utils.retry_transient(make_run_stub([(True, "ok")]), "arg")
        remote_utils.retry_transient(make_run_stub([(False, "fatal: repository not found")]), "arg")
        assert not remote_utils.transient_error_seen()

    def test_three_tuple_runner_uses_stderr(self, monkeypatch):
        """run_git returns (success, stdout, stderr) — the LAST element is the
        error text the transient check must read."""
        monkeypatch.setattr(remote_utils.time, "sleep", lambda _s: None)
        stub = make_run_stub(
            [
                (False, "some stdout", "ssh: Connection timed out"),
                (True, "out", ""),
            ]
        )
        assert remote_utils.retry_transient(stub, "repo", ["fetch"]) == (True, "out", "")
        assert len(stub.calls) == 2


def test_run_git_network_delegates_to_run_git_with_retry(monkeypatch):
    monkeypatch.setattr(remote_utils.time, "sleep", lambda _s: None)
    stub = make_run_stub(
        [
            (False, "", "kex_exchange_identification: Connection reset"),
            (True, "fetched", ""),
        ]
    )
    monkeypatch.setattr(remote_utils, "run_git", stub)
    result = remote_utils.run_git_network("repo", ["fetch", "gitcloud"], False)
    assert result == (True, "fetched", "")
    assert stub.calls == [("repo", ["fetch", "gitcloud"], False)] * 2


class TestSyncReposDelegation:
    def test_run_network_cmd_retries_via_shared_helper(self, monkeypatch):
        monkeypatch.setattr(remote_utils.time, "sleep", lambda _s: None)
        stub = make_run_stub(
            [
                (False, "Connection reset by peer"),
                (True, "Already up to date."),
            ]
        )
        monkeypatch.setattr(sync_repos, "run_git_cmd", stub)
        success, output = sync_repos.run_network_cmd("repo", ["pull", "--rebase"])
        assert success and output == "Already up to date."
        assert len(stub.calls) == 2

    def test_sync_gitbug_routes_through_retry_wrapper(self, monkeypatch, tmp_path):
        """git-bug pull/push must go through run_network_cmd (issue #579
        pre-review finding: they previously bypassed the retry path)."""
        monkeypatch.setattr(sync_repos.shutil, "which", lambda _cmd: "/usr/bin/git-bug")
        monkeypatch.setattr(
            sync_repos.subprocess,
            "run",
            lambda *a, **k: type("R", (), {"returncode": 0, "stdout": "bridge", "stderr": ""})(),
        )
        network_calls = []

        def fake_network_cmd(repo_path, cmd_args, dry_run=False):
            network_calls.append(cmd_args)
            return True, ""

        monkeypatch.setattr(sync_repos, "run_network_cmd", fake_network_cmd)
        sync_repos.sync_gitbug(tmp_path)
        assert network_calls == [["bug", "pull"], ["bug", "push"]]

    def test_sync_gitbug_stops_after_failed_pull(self, monkeypatch, tmp_path):
        monkeypatch.setattr(sync_repos.shutil, "which", lambda _cmd: "/usr/bin/git-bug")
        monkeypatch.setattr(
            sync_repos.subprocess,
            "run",
            lambda *a, **k: type("R", (), {"returncode": 0, "stdout": "bridge", "stderr": ""})(),
        )
        network_calls = []

        def fake_network_cmd(repo_path, cmd_args, dry_run=False):
            network_calls.append(cmd_args)
            return False, "permission denied"

        monkeypatch.setattr(sync_repos, "run_network_cmd", fake_network_cmd)
        sync_repos.sync_gitbug(tmp_path)
        assert network_calls == [["bug", "pull"]]


class TestMakeThrottler:
    """Pacing policy (issue #582): off by default, adaptive after a dropped
    connection, fixed when --throttle is given explicitly."""

    def _sleeps(self, monkeypatch):
        """Record throttle pauses. Must be installed AFTER trip_transient_flag —
        sync_repos.time and remote_utils.time are the same module, so the trip
        helper's sleep patch would otherwise replace the recorder."""
        sleeps = []
        monkeypatch.setattr(sync_repos.time, "sleep", sleeps.append)
        return sleeps

    def test_default_no_pause_without_transient_error(self, monkeypatch):
        sleeps = self._sleeps(monkeypatch)
        pause = sync_repos.make_throttler(None, dry_run=False)
        pause()
        pause()
        assert not sleeps

    def test_default_paces_after_transient_error(self, monkeypatch):
        pause = sync_repos.make_throttler(None, dry_run=False)
        trip_transient_flag(monkeypatch)
        sleeps = self._sleeps(monkeypatch)
        pause()
        pause()
        assert sleeps == [sync_repos.ADAPTIVE_THROTTLE] * 2

    def test_explicit_throttle_paces_from_the_start(self, monkeypatch):
        sleeps = self._sleeps(monkeypatch)
        pause = sync_repos.make_throttler(5.0, dry_run=False)
        pause()
        pause()
        assert sleeps == [5.0, 5.0]

    def test_explicit_zero_disables_pacing_even_after_transient(self, monkeypatch):
        trip_transient_flag(monkeypatch)
        sleeps = self._sleeps(monkeypatch)
        pause = sync_repos.make_throttler(0.0, dry_run=False)
        pause()
        assert not sleeps

    def test_negative_throttle_rejected(self, monkeypatch, capsys):
        """A fat-fingered negative would otherwise silently disable even the
        adaptive safety net — argparse must reject it."""
        monkeypatch.setattr(sys, "argv", ["sync_repos.py", "--throttle", "-5"])
        with pytest.raises(SystemExit):
            sync_repos.main()
        assert "--throttle must be >= 0" in capsys.readouterr().err

    def test_dry_run_never_pauses(self, monkeypatch):
        trip_transient_flag(monkeypatch)
        sleeps = self._sleeps(monkeypatch)
        for explicit in (None, 5.0):
            pause = sync_repos.make_throttler(explicit, dry_run=True)
            pause()
        assert not sleeps
