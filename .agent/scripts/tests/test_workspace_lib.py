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

from workspace import get_optional_layers  # noqa: E402


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
