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

import pytest  # noqa: E402

import push_remote  # noqa: E402
import remote_utils  # noqa: E402
from remote_utils import RemoteState  # noqa: E402
from workspace import WorkspaceConfigError  # noqa: E402


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


def _push_args(**kw):
    """The parsed argparse namespace push_remote.process_repo() actually reads.

    Mirrors main()'s parser exactly — `tags`/`force` were invented and
    `set_default_branch` (read at the end of process_repo) was missing, so any
    test reaching the push path would have died on AttributeError rather than
    exercising it.
    """
    fields = {
        "remote": "gitcloud",
        "dry_run": False,
        "manifest": None,
        "include_underlay": False,
        "all_branches": False,
        "set_default_branch": False,
    }
    fields.update(kw)
    return SimpleNamespace(**fields)


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


def test_remote_probe_matches_a_whole_line_not_a_substring(monkeypatch):
    """`gitcloud` contains `cloud`, and this one line is the sole PRESENT /
    ABSENT discriminator. A substring match would report a remote named
    `cloud` as configured, and every push/pull to it would then fail on a
    remote that does not exist — or worse, be skipped as "not found" by the
    inverse case."""
    _stub_run_git(monkeypatch, True, "origin\ngitcloud")
    assert remote_utils.remote_probe(Path("/repo"), "cloud") is RemoteState.ABSENT
    assert remote_utils.remote_probe(Path("/repo"), "git") is RemoteState.ABSENT
    assert remote_utils.remote_probe(Path("/repo"), "gitcloud") is RemoteState.PRESENT


def test_push_process_repo_reaches_the_end_with_a_real_arg_namespace(monkeypatch):
    """Guards the fixture above: process_repo() reads args.set_default_branch
    on its success path, so a namespace missing it fails the whole test."""
    monkeypatch.setattr(push_remote, "remote_probe", lambda repo_path, remote: RemoteState.PRESENT)
    monkeypatch.setattr(push_remote, "get_default_branch", lambda repo_path, v: "jazzy")
    monkeypatch.setattr(
        push_remote, "run_git", lambda repo_path, args, dry_run=False: (True, "", "")
    )
    monkeypatch.setattr(
        push_remote, "run_git_network", lambda repo_path, args, dry_run=False: (True, "", "")
    )
    status, msg = push_remote.process_repo(Path("/repo"), "r", "jazzy", _push_args())
    assert (status, msg) == ("ok", "pushed")


# --------------------------------------------------------------------------
# The enumeration layer: honest probes over repos nobody enumerated is still
# a green run over nothing (#609)
# --------------------------------------------------------------------------


def _iter(monkeypatch, tmp_path, repos, **opts):
    """Run iter_repos() with the repo list and the filesystem stubbed out.

    Options: `optional_layers` (declared optional on this host), `on_disk`
    (whether resolve_repo_path finds a checkout), `raises` (an exception for
    get_repos to throw). Collected into **opts rather than named parameters so
    the helper stays under pylint's argument limit.
    """
    optional_layers = opts.get("optional_layers", ())
    on_disk = opts.get("on_disk", True)
    raises = opts.get("raises")
    if raises is not None:
        monkeypatch.setattr(remote_utils, "get_repos", lambda args: (_ for _ in ()).throw(raises))
    else:
        monkeypatch.setattr(remote_utils, "get_repos", lambda args: list(repos))
    monkeypatch.setattr(
        remote_utils, "resolve_repo_path", lambda root, repo: (tmp_path if on_disk else None)
    )
    monkeypatch.setattr(remote_utils, "get_default_branch", lambda repo_path, v: "main")
    monkeypatch.setattr(remote_utils, "get_optional_layers", lambda root: set(optional_layers))
    args = SimpleNamespace(remote="gitcloud", manifest=None, include_underlay=False, dry_run=False)
    return remote_utils.iter_repos(
        tmp_path / ".agent" / "scripts",
        args,
        lambda path, name, version, a: ("ok", "fetched"),
        {"ok": 0, "skip": 0, "error": 0, "missing": 0},
    )


def _repo(name="alpha", source_file="core.repos"):
    return {"name": name, "version": "jazzy", "source_file": source_file}


def test_enumerating_zero_repos_is_an_error_not_a_one_repo_success(monkeypatch, tmp_path):
    """In a workspace worktree (no configs/manifest symlink) this printed
    "Summary: 1 repos — 1 up to date, 0 errors" and exited 0, quantifying an
    all-clear over every repo it never saw. sync_repos.py already exits 1
    naming configs/manifest in exactly this state."""
    results = _iter(monkeypatch, tmp_path, [])
    assert results["error"] == 1
    with pytest.raises(SystemExit) as exc:
        remote_utils.print_summary_and_exit(results, [("error", "errors")])
    assert exc.value.code == 1


def test_enumerating_zero_repos_names_configs_manifest(monkeypatch, tmp_path, capsys):
    _iter(monkeypatch, tmp_path, [])
    assert "configs/manifest" in capsys.readouterr().out


def test_an_unparseable_manifest_is_an_error_not_an_empty_repo_list(monkeypatch, tmp_path):
    """WorkspaceConfigError must not escape as a traceback, and must not be
    quietly converted into "no repos configured" either."""
    results = _iter(
        monkeypatch, tmp_path, [], raises=WorkspaceConfigError("cannot parse core.repos: boom")
    )
    assert results["error"] == 1


def test_a_repo_missing_from_a_required_layer_makes_the_run_non_zero(monkeypatch, tmp_path):
    """`missing` was tallied into a bucket print_summary_and_exit() never read,
    so a repo with no checkout at all left the run green."""
    results = _iter(monkeypatch, tmp_path, [_repo()], on_disk=False)
    assert results["missing"] == 1
    with pytest.raises(SystemExit) as exc:
        remote_utils.print_summary_and_exit(results, [("missing", "missing")])
    assert exc.value.code == 1


def test_a_repo_missing_from_an_optional_layer_stays_green(monkeypatch, tmp_path):
    """The false-RED direction, and the one that reaches the field: `site` is
    optional on every manifest this workspace ships, and salmon/gabby do not
    carry it. Counting that as a failure would put those hosts permanently red
    — the false red this design calls worse than the false green it removes."""
    results = _iter(
        monkeypatch,
        tmp_path,
        [_repo(source_file="site.repos")],
        optional_layers=("site",),
        on_disk=False,
    )
    assert results["missing"] == 0
    assert results["skip"] == 1
    remote_utils.print_summary_and_exit(results, [("skip", "skipped")])


def test_a_fully_enumerated_healthy_run_still_exits_zero(monkeypatch, tmp_path):
    """The other false-RED guard: nothing above may turn an ordinary run red."""
    results = _iter(monkeypatch, tmp_path, [_repo(), _repo("beta")])
    assert (results["error"], results["missing"]) == (0, 0)
    assert results["ok"] == 3  # two repos plus the workspace root
    remote_utils.print_summary_and_exit(results, [("ok", "ok")])


def test_the_summary_total_counts_each_repo_once(monkeypatch, tmp_path):
    """`missing` is a bucket of its own, so a missing repo must not also land
    in skip/error — the printed "N repos" is a count operators read."""
    results = _iter(monkeypatch, tmp_path, [_repo(), _repo("beta")], on_disk=False)
    assert sum(results.values()) == 3
