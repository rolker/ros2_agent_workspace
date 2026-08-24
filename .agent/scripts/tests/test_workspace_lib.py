"""Tests for the shared workspace helpers in `.agent/scripts/lib/workspace.py`.

`get_optional_layers()` was extracted in #609 so `sync_repos.py` and
`validate_workspace.py` decide "this layer is allowed to be absent" from one
parser instead of two. Both now classify missing repos on its answer, and the
extraction claims to be byte-compatible with `setup_layers.sh`'s
`is_optional_layer()` — strip from the first `#`, trim, skip blanks. That claim
is load-bearing for two callers, so it is asserted here rather than asserted in
a comment.
"""

import subprocess
import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SCRIPTS_DIR / "lib"))

import pytest  # noqa: E402

sys.path.insert(0, str(SCRIPTS_DIR))

import list_overlay_repos  # noqa: E402
import workspace  # noqa: E402
from workspace import WorkspaceConfigError, get_optional_layers  # noqa: E402


def write_optional(root, text):
    """Create configs/manifest/optional_layers.txt with exactly `text`."""
    manifest = root / "configs" / "manifest"
    manifest.mkdir(parents=True, exist_ok=True)
    (manifest / "optional_layers.txt").write_text(text)
    return root


def test_missing_file_declares_no_optional_layers(tmp_path):
    """No optional_layers.txt at all means nothing is optional — never
    everything. Returning a non-empty set here would make every unlocatable
    repo a benign skip on a workspace that never declared one (#609)."""
    assert get_optional_layers(tmp_path) == set()


def test_comments_blanks_and_whitespace_are_stripped(tmp_path):
    """Byte-compatible with setup_layers.sh's is_optional_layer(): strip from
    the first `#`, trim surrounding whitespace, skip what is left empty."""
    write_optional(
        tmp_path,
        "# layers this host may not have access to\n"
        "\n"
        "site\n"
        "   spaced   \n"
        "trailing # inline comment\n"
        "   # indented comment\n"
        "\t\n",
    )
    assert get_optional_layers(tmp_path) == {"site", "spaced", "trailing"}


def test_an_empty_file_declares_no_optional_layers(tmp_path):
    """Present but empty is the same statement as absent: nothing is optional."""
    write_optional(tmp_path, "")
    assert get_optional_layers(tmp_path) == set()


def test_matches_setup_layers_sh_on_the_same_file(tmp_path):
    """The compatibility claim, checked against the shell implementation itself
    rather than against a restatement of it."""
    setup_layers = SCRIPTS_DIR / "setup_layers.sh"
    if not setup_layers.exists():  # pragma: no cover - defensive
        return
    content = "# comment\n\nsite\n  padded  \ninline # tail\n"
    write_optional(tmp_path, content)
    optional_file = tmp_path / "configs" / "manifest" / "optional_layers.txt"

    # Re-implementing is_optional_layer() here would test the restatement, so
    # source the real function out of setup_layers.sh and ask it directly.
    script = (
        f'MANIFEST_SYMLINK="{optional_file.parent}"\n'
        + _extract_is_optional_layer(setup_layers)
        + "\nfor l in site padded inline comment tail absent; do"
        ' if is_optional_layer "$l"; then echo "$l"; fi; done\n'
    )
    result = subprocess.run(["bash", "-c", script], capture_output=True, text=True, check=True)
    assert (
        set(result.stdout.split())
        == get_optional_layers(tmp_path)
        == {
            "site",
            "padded",
            "inline",
        }
    )


def _extract_is_optional_layer(path):
    """The `is_optional_layer() { ... }` block, verbatim, out of setup_layers.sh."""
    lines = path.read_text().splitlines()
    start = next(i for i, line in enumerate(lines) if line.startswith("is_optional_layer()"))
    end = next(i for i in range(start, len(lines)) if lines[i] == "}")
    return "\n".join(lines[start : end + 1])


# --------------------------------------------------------------------------
# A manifest that will not parse is an error, not an empty answer (#609)
# --------------------------------------------------------------------------


def write_manifest(root, filename, text):
    """Create configs/manifest/repos/<filename> containing exactly `text`."""
    repos_dir = root / "configs" / "manifest" / "repos"
    repos_dir.mkdir(parents=True, exist_ok=True)
    (repos_dir / filename).write_text(text)
    return root


GOOD_MANIFEST = """\
repositories:
  alpha:
    type: git
    url: git@example:alpha.git
    version: jazzy
"""

# Tabs are illegal YAML indentation — the shape a hand-edited .repos file
# actually takes when it breaks.
BAD_MANIFEST = "repositories:\n\talpha:\n\t\turl: git@example:alpha.git\n"


def test_unparseable_manifest_raises_rather_than_dropping_its_repos(tmp_path, monkeypatch):
    """get_overlay_repos() printed the parse error and carried on with the
    repos it *did* read. Every caller then reported an all-clear over repos
    nothing had enumerated — the enumeration-layer twin of the per-repo false
    green this issue is about (#609)."""
    write_manifest(tmp_path, "core.repos", BAD_MANIFEST)
    monkeypatch.setattr(workspace, "get_workspace_root", lambda: str(tmp_path))
    with pytest.raises(WorkspaceConfigError) as exc:
        workspace.get_overlay_repos()
    assert "core.repos" in str(exc.value)


def test_a_readable_manifest_still_enumerates_its_repos(tmp_path, monkeypatch):
    """The false-RED direction: raising must be reserved for a manifest that
    genuinely will not parse, or every healthy workspace goes red."""
    write_manifest(tmp_path, "core.repos", GOOD_MANIFEST)
    monkeypatch.setattr(workspace, "get_workspace_root", lambda: str(tmp_path))
    repos = workspace.get_overlay_repos()
    assert [r["name"] for r in repos] == ["alpha"]
    assert repos[0]["version"] == "jazzy"


def test_a_manifest_with_no_repositories_key_is_not_an_error(tmp_path, monkeypatch):
    """Also the false-RED direction: an empty or comment-only .repos file is a
    valid file that declares nothing. Only a *parse failure* is an error."""
    write_manifest(tmp_path, "core.repos", "# nothing here yet\n")
    monkeypatch.setattr(workspace, "get_workspace_root", lambda: str(tmp_path))
    assert workspace.get_overlay_repos() == []


def test_a_non_mapping_repositories_key_is_an_error(tmp_path, monkeypatch):
    """Valid YAML, wrong shape: `repositories:` as a list parses cleanly and
    then blows up on .items(). Reported as a config error, not a traceback."""
    write_manifest(tmp_path, "core.repos", "repositories:\n  - alpha\n")
    monkeypatch.setattr(workspace, "get_workspace_root", lambda: str(tmp_path))
    with pytest.raises(WorkspaceConfigError):
        workspace.get_overlay_repos()


def test_find_repo_version_raises_on_the_same_manifest(tmp_path, monkeypatch):
    """find_repo_version() swallowed the identical error and returned
    "unknown"; worktree_create.sh branches a worktree off that answer."""
    write_manifest(tmp_path, "core.repos", BAD_MANIFEST)
    monkeypatch.setattr(workspace, "get_workspace_root", lambda: str(tmp_path))
    with pytest.raises(WorkspaceConfigError):
        workspace.find_repo_version("alpha")


def test_find_repo_version_still_answers_from_a_good_manifest(tmp_path, monkeypatch):
    write_manifest(tmp_path, "core.repos", GOOD_MANIFEST)
    monkeypatch.setattr(workspace, "get_workspace_root", lambda: str(tmp_path))
    assert workspace.find_repo_version("alpha") == "jazzy"
    assert workspace.find_repo_version("nope") == "unknown"


# --------------------------------------------------------------------------
# One shared rule for "this repo is allowed to be absent"
# --------------------------------------------------------------------------


def test_absence_is_allowed_only_for_an_optional_layer():
    """sync_repos.py and the remote scripts must agree; they now decide through
    this one predicate rather than two copies of the same `in` test."""
    site = {"name": "r", "source_file": "site.repos"}
    core = {"name": "r", "source_file": "core.repos"}
    assert workspace.repo_absence_is_allowed(site, {"site"}) is True
    assert workspace.repo_absence_is_allowed(core, {"site"}) is False
    assert workspace.repo_absence_is_allowed(site, set()) is False


def test_list_overlay_repos_cli_exits_non_zero_on_an_unparseable_manifest(monkeypatch, capsys):
    """The CLI wrapper is what shell callers read. Printing `[]` for a manifest
    it could not parse hands them an empty array indistinguishable from "no
    repos are configured" (#609)."""

    def boom(include_underlay=False):
        raise WorkspaceConfigError("cannot parse core.repos: mapping values not allowed")

    monkeypatch.setattr(list_overlay_repos, "get_overlay_repos", boom)
    monkeypatch.setattr(sys, "argv", ["list_overlay_repos.py"])
    with pytest.raises(SystemExit) as exc:
        list_overlay_repos.main()
    assert exc.value.code == 1
    captured = capsys.readouterr()
    assert "core.repos" in captured.err
    assert captured.out.strip() == ""


def test_list_overlay_repos_cli_still_prints_a_healthy_manifest(monkeypatch, capsys):
    """False-RED guard for the arm above."""
    monkeypatch.setattr(
        list_overlay_repos, "get_overlay_repos", lambda include_underlay=False: [{"name": "alpha"}]
    )
    monkeypatch.setattr(sys, "argv", ["list_overlay_repos.py", "--format", "names"])
    list_overlay_repos.main()
    assert capsys.readouterr().out.strip() == "alpha"
