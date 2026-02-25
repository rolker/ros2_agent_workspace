#!/usr/bin/env python3
"""
Validate workspace matches .repos configuration.

This script verifies that the actual workspace repositories match what is
configured in the .repos files. It checks:
1. All repos in configs are cloned in valid layers
2. All cloned repos are in configs (detect orphans)
3. Repos are on expected branches/versions
4. Enforces agent isolation (ignores .agent/ directories)

Usage:
    python3 validate_workspace.py [--fix] [--verbose]
"""

import os
import sys
import argparse
import subprocess
from pathlib import Path

# Add lib directory to path
SCRIPT_DIR = Path(__file__).parent.resolve()
sys.path.insert(0, str(SCRIPT_DIR / "lib"))

from workspace import get_workspace_root, get_overlay_repos


def get_actual_repos(workspace_root):
    """
    Get all repositories actually present in workspace layers.

    Args:
        workspace_root (str): Path to workspace root

    Returns:
        dict: Repository information keyed by repo name
    """
    actual_repos = {}
    root_path = Path(workspace_root)
    layers_dir = root_path / "layers"

    if not layers_dir.exists():
        # Fallback to workspaces/ if layers/ doesn't exist (legacy support)
        workspaces_dir = root_path / "workspaces"
        if workspaces_dir.exists():
            search_dirs = [workspaces_dir]
        else:
            return actual_repos
    else:
        search_dirs = [layers_dir]

    for search_dir in search_dirs:
        # Walk through the directory structure (limit depth for performance)
        # Repos are typically at: layers/<layer>/<ws>/src/<repo>
        for root, dirs, _files in os.walk(search_dir):
            root_path_obj = Path(root)

            # Performance optimization: limit depth
            # If we're more than 5 levels deep, we've gone too far
            depth = len(root_path_obj.relative_to(search_dir).parts)
            if depth > 5:
                dirs[:] = []  # Stop descending
                continue

            # Skip .agent, .git, and hidden directories
            dirs[:] = [d for d in dirs if not d.startswith(".")]
            if ".agent" in root_path_obj.parts:
                continue

            # Check if this directory is a git repo
            if (root_path_obj / ".git").exists():
                repo_name = root_path_obj.name

                # Identify workspace/layer context
                # Structure: layers/<layer>/<ws>/src/<repo>
                try:
                    rel_path = root_path_obj.relative_to(search_dir)
                    # Use the first part as layer/workspace indicator mostly for reporting
                    context = str(rel_path.parent)
                except ValueError:
                    context = "unknown"

                branch = get_git_branch(root_path_obj)

                actual_repos[repo_name] = {
                    "path": root_path_obj,
                    "context": context,
                    "branch": branch,
                }

                # Don't recurse into a git repo
                dirs[:] = []

    return actual_repos


def get_git_commit(repo_path):
    """Get the current commit SHA of a git repository."""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=str(repo_path),
            capture_output=True,
            text=True,
            check=True,
        )
        return result.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def get_git_branch(repo_path):
    """Get the current branch of a git repository."""
    try:
        result = subprocess.run(
            ["git", "branch", "--show-current"],
            cwd=str(repo_path),
            capture_output=True,
            text=True,
            check=True,
        )
        branch = result.stdout.strip()
        return branch if branch else None  # None indicates detached HEAD or error
    except subprocess.CalledProcessError as e:
        print(f"Warning: git command failed for {repo_path}: {e}", file=sys.stderr)
        return None
    except FileNotFoundError:
        print("Error: git command not found. Please ensure git is installed.", file=sys.stderr)
        return None
    except Exception as e:
        print(f"Error: Unexpected error getting branch for {repo_path}: {e}", file=sys.stderr)
        return None


def validate_workspace(verbose=False):
    """Validate workspace matches configuration."""
    print("Validating workspace against .repos configuration...")
    print()

    root = get_workspace_root()

    # Get configured repos using shared library
    configured_list = get_overlay_repos(include_underlay=True)
    # Convert list to dict for easy lookup
    config_repos = {item["name"]: item for item in configured_list}

    # Get actual repos
    actual_repos = get_actual_repos(root)

    if verbose:
        print(f"Found {len(config_repos)} configured repositories")
        print(f"Found {len(actual_repos)} actual repositories in workspace")
        print()

    # Find missing repos
    missing_repos = []
    for name, config in config_repos.items():
        if name not in actual_repos:
            missing_repos.append({"name": name, "config": config})

    # Find extra repos (orphans)
    extra_repos = []
    for name, actual in actual_repos.items():
        if name not in config_repos:
            extra_repos.append({"name": name, "actual": actual})

    # Check versions
    version_mismatches = []
    for name in set(config_repos.keys()) & set(actual_repos.keys()):
        config = config_repos[name]
        actual = actual_repos[name]

        expected = config.get("version", "")
        current_branch = actual.get("branch")
        current_commit = get_git_commit(actual["path"])

        # Improved version check: handle branches, tags, and commit SHAs
        # Try to resolve expected version to a commit SHA
        try:
            result = subprocess.run(
                ["git", "rev-parse", expected],
                cwd=str(actual["path"]),
                capture_output=True,
                text=True,
                check=False,  # Don't raise on error
            )
            expected_commit = result.stdout.strip() if result.returncode == 0 else None
        except (subprocess.CalledProcessError, FileNotFoundError):
            expected_commit = None

        # Compare commits if we have both
        if current_commit and expected_commit:
            if current_commit != expected_commit:
                # Check if it's a branch name mismatch or actual version mismatch
                if current_branch and current_branch != expected:
                    version_mismatches.append(
                        {
                            "name": name,
                            "type": "version_mismatch",
                            "expected": expected,
                            "actual": current_branch
                            if current_branch
                            else f"{current_commit[:8]} (detached)",
                            "path": actual["path"],
                        }
                    )
        elif current_branch is None and expected_commit is None:
            # Detached HEAD but expected version couldn't be resolved
            # Only flag if expected looks like a branch name (not a SHA)
            if expected and not (
                len(expected) == 40 and all(c in "0123456789abcdef" for c in expected.lower())
            ):
                version_mismatches.append(
                    {
                        "name": name,
                        "type": "detached_head",
                        "expected": expected,
                        "actual": "DETACHED HEAD",
                        "path": actual["path"],
                    }
                )
        elif current_branch != expected:
            # Simple branch name comparison fallback
            version_mismatches.append(
                {
                    "name": name,
                    "type": "version_mismatch",
                    "expected": expected,
                    "actual": current_branch if current_branch else "DETACHED HEAD",
                    "path": actual["path"],
                }
            )

    # Print results
    is_valid = not (missing_repos or extra_repos or version_mismatches)

    print("=" * 60)
    print("Workspace Validation Results")
    print("=" * 60)
    print(f"Configured repos: {len(config_repos)}")
    print(f"Actual repos:     {len(actual_repos)}")
    print()

    if missing_repos:
        print(f"❌ Missing repos ({len(missing_repos)}):")
        for item in sorted(missing_repos, key=lambda x: x["name"]):
            print(f"   - {item['name']} (from {item['config'].get('source_file')})")
        print()

    if extra_repos:
        print(f"⚠️  Extra repos ({len(extra_repos)}):")
        for item in sorted(extra_repos, key=lambda x: x["name"]):
            print(f"   - {item['name']} (in {item['actual']['context']})")
        print()

    if version_mismatches:
        print(f"⚠️  Version mismatches ({len(version_mismatches)}):")
        for item in sorted(version_mismatches, key=lambda x: x["name"]):
            print(f"   - {item['name']}: {item['expected']} vs {item['actual']}")
            if verbose:
                print(f"     at {item['path']}")
        print()

    if is_valid:
        print("✅ Workspace validation PASSED!")
    else:
        print("❌ Workspace validation FAILED")
        if missing_repos:
            print("   Run with --fix to import missing repositories.")

    print("=" * 60)

    return is_valid, missing_repos


def fix_workspace(missing_repos, verbose=False):
    """Import missing repositories."""
    if not missing_repos:
        return True

    root = Path(get_workspace_root())

    # Default target directory for fixes is layers/main/core_ws/src/
    # In a real scenario, we might want to be smarter about which layer/ws matches the repo file
    # For now, we'll try to infer from source_file or default to main/core_ws

    print(f"Attempting to import {len(missing_repos)} missing repositories...")

    # Group by source config file
    by_config = {}
    for item in missing_repos:
        src = item["config"].get("source_file")
        if src not in by_config:
            by_config[src] = []
        by_config[src].append(item)

    success = True

    for src_file, _items in by_config.items():
        # Determine target workspace based on filename (heuristic)
        # e.g. core.repos -> layers/main/core_ws/src
        # e.g. platforms.repos -> layers/main/platforms_ws/src

        # Validate src_file path to prevent path traversal
        if not src_file or ".." in src_file or src_file.startswith("/"):
            print(f"Error: Invalid config file name: {src_file}", file=sys.stderr)
            success = False
            continue

        ws_name = src_file.replace(".repos", "_ws")

        # Dynamically discover workspace directory under layers/*
        # Don't assume layers/main - search all layer directories
        layers_root = root / "layers"
        candidate_dirs = []
        if layers_root.exists():
            for layer_dir in layers_root.iterdir():
                if not layer_dir.is_dir() or layer_dir.name.startswith("."):
                    continue
                candidate = layer_dir / ws_name / "src"
                if candidate.exists():
                    candidate_dirs.append(candidate)

        if not candidate_dirs:
            # No existing workspace found, default to layers/main
            target_dir = layers_root / "main" / ws_name / "src"
            print(
                f"Warning: No existing workspace found for {ws_name}, using default: {target_dir}"
            )
        elif len(candidate_dirs) > 1:
            print(
                f"Error: Multiple workspace directories found for {ws_name}: {candidate_dirs}",
                file=sys.stderr,
            )
            success = False
            continue
        else:
            target_dir = candidate_dirs[0]

        # Ensure target dir exists
        target_dir.mkdir(parents=True, exist_ok=True)

        # Use vcs import
        # We need to filter the original .repos file to only include missing repos?
        # vcs import allows importing selected repos but it's easier to just feed the whole file
        # and vcs skip existing ones. However, we want to be explicit.
        # But we don't have the original file path easily handy without searching again.

        # Re-find the config file
        config_path = None
        for path_candidate in [
            root / "configs" / "manifest" / "repos" / src_file,
            root / "configs" / src_file,
        ]:
            if path_candidate.exists():
                config_path = path_candidate
                break

        if not config_path:
            print(f"Error: Could not find config file for {src_file}", file=sys.stderr)
            success = False
            continue

        print(f"Importing from {src_file} into {target_dir}...")
        cmd = ["vcs", "import", str(target_dir)]

        # We only want to import the missing ones.
        # vcs tool doesn't easily support "only these names".
        # But vcs import skips existing directories.
        # So running it for the whole file is safe.

        try:
            with open(config_path, "r") as f:
                result = subprocess.run(cmd, stdin=f, cwd=str(root), capture_output=True, text=True)
            if result.returncode != 0:
                print(f"Error importing: {result.stderr}")
                success = False
            elif verbose:
                print(result.stdout)

        except Exception as e:
            print(f"Exception importing: {e}")
            success = False

    return success


def main():
    parser = argparse.ArgumentParser(description="Validate workspace")
    parser.add_argument("--fix", action="store_true", help="Fix missing repos")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
    args = parser.parse_args()

    is_valid, missing = validate_workspace(args.verbose)

    if not is_valid and args.fix and missing:
        print("\nFixing workspace...")
        if fix_workspace(missing, args.verbose):
            print("Fix complete. Re-validating...")
            validate_workspace(args.verbose)
        else:
            print("Fix encountered errors.")
            sys.exit(1)

    sys.exit(0 if is_valid else 1)


if __name__ == "__main__":
    main()
