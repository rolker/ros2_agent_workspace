"""
Workspace Management Library

Provides common functions for discovering and managing repositories in the
ROS2 Agent Workspace.
"""

import glob
import os
from pathlib import Path

import yaml


class WorkspaceConfigError(RuntimeError):
    """A .repos file was found but could not be parsed (#609).

    Every repo that file declares goes unenumerated, so callers that answered
    "here is the repo list" over the survivors were reporting an all-clear over
    repos nothing ever looked at. Raising makes the enumeration layer as honest
    as the per-repo probes above it: a manifest we could not read is an error,
    never an empty answer.
    """


def _load_repos_file(repo_file):
    """Parse one .repos file. Returns its `repositories` mapping (possibly empty).

    Raises WorkspaceConfigError if the YAML is malformed. Shared by
    get_overlay_repos() and find_repo_version() so the two cannot drift on what
    an unreadable manifest means.
    """
    try:
        with open(repo_file, "r") as f:
            data = yaml.safe_load(f)
    except yaml.YAMLError as e:
        raise WorkspaceConfigError(f"cannot parse {os.path.basename(repo_file)}: {e}") from e
    except OSError as e:
        raise WorkspaceConfigError(f"cannot read {os.path.basename(repo_file)}: {e}") from e
    if not data or "repositories" not in data:
        return {}
    repositories = data["repositories"]
    if not isinstance(repositories, dict):
        raise WorkspaceConfigError(
            f"cannot parse {os.path.basename(repo_file)}: "
            f"'repositories' is {type(repositories).__name__}, expected a mapping"
        )
    return repositories


def get_workspace_root():
    """Get the absolute path to the workspace root directory."""
    # This file is in .agent/scripts/lib/, so go up 3 levels
    lib_dir = Path(__file__).parent
    scripts_dir = lib_dir.parent
    agent_dir = scripts_dir.parent
    workspace_root = agent_dir.parent
    return str(workspace_root)


def get_overlay_repos(include_underlay=False):
    """
    Get a list of all repositories defined in workspace .repos files.

    Args:
        include_underlay (bool): If True, include repositories from underlay.repos

    Returns:
        list: List of dictionaries containing repository information:
            - name: Repository name
            - url: Git URL
            - version: Branch/tag/commit
            - source_file: Name of the .repos file

    Raises:
        WorkspaceConfigError: a .repos file exists but could not be parsed. An
            unreadable manifest is never reported as "no repos are configured".
    """
    workspace_root = get_workspace_root()
    ignored_files = ["underlay.repos"]
    repos_list = []

    # Find all .repos files via configs/manifest symlink and configs/ fallback
    config_dirs = [
        os.path.join(workspace_root, "configs", "manifest", "repos"),
        os.path.join(workspace_root, "configs"),
    ]

    repo_files = []
    for d in config_dirs:
        if os.path.isdir(d):
            repo_files.extend(glob.glob(os.path.join(d, "*.repos")))

    for repo_file in repo_files:
        filename = os.path.basename(repo_file)
        if filename in ignored_files and not include_underlay:
            continue

        # A malformed manifest raises rather than being printed-and-skipped:
        # dropping its repos here left every caller reporting success over
        # repos it never enumerated (#609).
        for name, info in _load_repos_file(repo_file).items():
            repos_list.append(
                {
                    "name": name,
                    "url": info.get("url", ""),
                    "version": info.get("version", ""),
                    "source_file": filename,
                }
            )

    return repos_list


def get_optional_layers(workspace_root=None):
    """Layer names that are allowed to be absent (configs/manifest/optional_layers.txt).

    setup_layers.sh treats these layers as optional: if `vcs import` fails (a
    private repo this host has no access to) it removes the layer directory and
    exits 0. Anything that reports on missing repos must know that, or it flags
    a supported host configuration as broken.

    Format: one layer name per line; `#` comments and blank lines ignored —
    kept byte-compatible with setup_layers.sh's is_optional_layer().
    """
    if workspace_root is None:
        workspace_root = get_workspace_root()
    optional_file = Path(workspace_root) / "configs" / "manifest" / "optional_layers.txt"
    if not optional_file.exists():
        return set()
    layers = set()
    for line in optional_file.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            layers.add(line)
    return layers


def layer_name_for(repo):
    """The layer a repo record belongs to, from its source .repos file name."""
    return repo["source_file"][: -len(".repos")] if repo["source_file"].endswith(".repos") else ""


def repo_absence_is_allowed(repo, optional_layers):
    """True if this configured repo is allowed to be missing from disk.

    The one rule both sync_repos.py and the remote scripts classify on: a repo
    from a layer listed in configs/manifest/optional_layers.txt may legitimately
    be absent (this host has no access to that layer's private repos, and
    setup_layers.sh exits 0 having removed or never populated it). A repo from a
    *required* layer that is not on disk is a real failure — "left stale, nobody
    noticed" is what #609 exists to kill.

    Deliberately does NOT consider whether the layer directory exists: a
    partially-imported optional layer keeps its src/, so the directory cannot
    tell "fully imported" from "supported host, repos legitimately absent".
    Gating on it would turn a supported configuration permanently red — the
    false red this design calls worse than the false green it removes.
    See sync_repos.classify_unlocatable_repo() for the long form.
    """
    return layer_name_for(repo) in optional_layers


def find_repo_version(target_repo):
    """
    Find the version/branch for a specific repository.

    Args:
        target_repo (str): Name of the repository to look up

    Returns:
        str: Version string (e.g., "jazzy", "main", "feature/foo") or "unknown" if not found

    Raises:
        WorkspaceConfigError: a .repos file exists but could not be parsed.
    """
    workspace_root = get_workspace_root()

    # Find all .repos files via configs/manifest symlink and configs/ fallback
    # Note: We include underlay.repos here because we want to validate ALL repos
    config_dirs = [
        os.path.join(workspace_root, "configs", "manifest", "repos"),
        os.path.join(workspace_root, "configs"),
    ]

    repo_files = []
    for d in config_dirs:
        if os.path.isdir(d):
            repo_files.extend(glob.glob(os.path.join(d, "*.repos")))

    for repo_file in repo_files:
        # Same contract as get_overlay_repos(): a manifest we cannot parse is an
        # error, not a file that declares nothing. Swallowing it here returned
        # "unknown", and callers branch a worktree off that answer (#609).
        repositories = _load_repos_file(repo_file)
        if target_repo in repositories:
            return repositories[target_repo].get("version", "unknown")

    return "unknown"


def extract_github_owner_repo(url):
    """
    Extract owner and repo name from a GitHub URL.

    Args:
        url (str): GitHub URL (https or git format)

    Returns:
        tuple: (owner, repo_name) or (None, None) if not a valid GitHub URL

    Examples:
        >>> extract_github_owner_repo("https://github.com/rolker/project11.git")
        ('rolker', 'project11')
        >>> extract_github_owner_repo("git@github.com:rolker/project11.git")
        ('rolker', 'project11')
    """
    if not url:
        return None, None

    # Handle https URLs
    if url.startswith("https://github.com/"):
        path = url.replace("https://github.com/", "")
        path = path.rstrip("/")
        if path.endswith(".git"):
            path = path[:-4]
        parts = path.split("/")
        if len(parts) >= 2:
            return parts[0], parts[1]

    # Handle git@ URLs
    if url.startswith("git@github.com:"):
        path = url.replace("git@github.com:", "")
        if path.endswith(".git"):
            path = path[:-4]
        parts = path.split("/")
        if len(parts) >= 2:
            return parts[0], parts[1]

    return None, None
