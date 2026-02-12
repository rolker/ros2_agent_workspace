"""
Workspace Management Library

Provides common functions for discovering and managing repositories in the
ROS2 Agent Workspace.
"""

import glob
import os
import sys
from pathlib import Path

import yaml


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
    """
    workspace_root = get_workspace_root()
    ignored_files = ["underlay.repos"]
    repos_list = []

    # Find all .repos files in configs/ and migrated Key Repo paths
    config_dirs = [
        os.path.join(workspace_root, "configs"),
        os.path.join(workspace_root, "layers/main/core_ws/src/unh_marine_autonomy/config/repos"),
    ]

    repo_files = []
    for d in config_dirs:
        if os.path.isdir(d):
            repo_files.extend(glob.glob(os.path.join(d, "*.repos")))

    for repo_file in repo_files:
        filename = os.path.basename(repo_file)
        if filename in ignored_files and not include_underlay:
            continue

        with open(repo_file, "r") as f:
            try:
                data = yaml.safe_load(f)
                if not data or "repositories" not in data:
                    continue

                for name, info in data["repositories"].items():
                    entry = {
                        "name": name,
                        "url": info.get("url", ""),
                        "version": info.get("version", ""),
                        "source_file": filename,
                    }
                    repos_list.append(entry)
            except yaml.YAMLError as e:
                print(f"Error parsing {filename}: {e}", file=sys.stderr)

    return repos_list


def find_repo_version(target_repo):
    """
    Find the version/branch for a specific repository.

    Args:
        target_repo (str): Name of the repository to look up

    Returns:
        str: Version string (e.g., "jazzy", "main", "feature/foo") or "unknown" if not found
    """
    workspace_root = get_workspace_root()

    # Find all .repos files in configs/ and migrated Key Repo paths
    # Note: We include underlay.repos here because we want to validate ALL repos
    config_dirs = [
        os.path.join(workspace_root, "configs"),
        os.path.join(workspace_root, "layers/main/core_ws/src/unh_marine_autonomy/config/repos"),
    ]

    repo_files = []
    for d in config_dirs:
        if os.path.isdir(d):
            repo_files.extend(glob.glob(os.path.join(d, "*.repos")))

    for repo_file in repo_files:
        with open(repo_file, "r") as f:
            try:
                data = yaml.safe_load(f)
                if not data or "repositories" not in data:
                    continue

                if target_repo in data["repositories"]:
                    repo_info = data["repositories"][target_repo]
                    return repo_info.get("version", "unknown")

            except yaml.YAMLError:
                continue

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
