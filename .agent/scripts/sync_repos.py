#!/usr/bin/env python3
"""
Sync Workspace Repositories

This script safely synchronizes workspace repositories by pulling updates on
default branches (main/jazzy) and fetching on feature branches. It respects
dirty working directories and detached HEAD states.

Usage:
    python3 sync_repos.py [--dry-run] [--throttle SECONDS]

Options:
    --dry-run    Simulate actions without executing
    --throttle   Pause between per-repo network operations. Off by default;
                 if a dropped connection is detected mid-run (an upstream
                 firewall rate-limiting rapid successive SSH connections),
                 the remaining repos are automatically paced at 2.0s.
                 Pass a value to force a fixed pace from the start
                 (0 disables pacing entirely, including the adaptive one).
"""

import sys
import math
import time
import shutil
import subprocess
import argparse
from pathlib import Path

# Add script directory to path to import list_overlay_repos
SCRIPT_DIR = Path(__file__).parent.resolve()
sys.path.append(str(SCRIPT_DIR))
sys.path.insert(0, str(SCRIPT_DIR / "lib"))

try:
    import list_overlay_repos
except ImportError:
    print(f"Error: Could not import list_overlay_repos from {SCRIPT_DIR}", file=sys.stderr)
    sys.exit(1)

from remote_utils import (  # noqa: E402
    reset_transient_error_seen,
    retry_transient,
    transient_error_seen,
)

# Pace applied to the rest of the run once the remote starts dropping
# connections (only when --throttle was not given explicitly).
ADAPTIVE_THROTTLE = 2.0


def make_throttler(explicit_throttle, dry_run):
    """Return a pause() callable implementing the pacing policy.

    explicit_throttle is the --throttle value or None if not given:
      None -> adaptive: no pause until a dropped connection has been seen,
              then ADAPTIVE_THROTTLE for the rest of the run.
      N    -> fixed pace of N seconds (0 disables pacing entirely).
    Dry runs never pause.
    """
    noticed = [False]

    def pause():
        if dry_run:
            return
        if explicit_throttle is not None:
            if explicit_throttle > 0:
                time.sleep(explicit_throttle)
            return
        if transient_error_seen():
            if not noticed[0]:
                noticed[0] = True
                print(
                    f"ℹ️  Remote dropped a connection — pacing remaining repos "
                    f"by {ADAPTIVE_THROTTLE}s."
                )
            time.sleep(ADAPTIVE_THROTTLE)

    return pause


def run_git_cmd(repo_path, cmd_args, dry_run=False):
    """Run a git command in the given repo path."""
    full_cmd = ["git"] + cmd_args
    if dry_run:
        print(f"[DRY-RUN] {repo_path.name}: {' '.join(full_cmd)}")
        return True, ""

    try:
        result = subprocess.run(
            full_cmd, cwd=str(repo_path), capture_output=True, text=True, check=True
        )
        return True, result.stdout.strip()
    except subprocess.CalledProcessError as e:
        return False, e.stderr.strip()


def run_network_cmd(repo_path, cmd_args, dry_run=False):
    """Run a git network command, retrying once after a backoff if the
    connection was dropped (rate-limit style failures — signatures and
    backoff live in lib/remote_utils.py, shared with push/pull_remote)."""
    return retry_transient(run_git_cmd, repo_path, cmd_args, dry_run)


def is_dirty(repo_path, dry_run=False):
    """Check if repo has uncommitted changes."""
    # Dirty check is a read-only operation, always execute regardless of dry_run
    # but we need to call run_git_cmd without dry_run flag to actually execute
    success, output = run_git_cmd(repo_path, ["status", "--porcelain"], dry_run=False)
    return success and bool(output)


def get_current_branch(repo_path, dry_run=False):
    """Get the current checked out branch."""
    # Getting branch is a read-only operation, always execute regardless of dry_run
    success, output = run_git_cmd(repo_path, ["branch", "--show-current"], dry_run=False)
    if success:
        return output
    return None


def sync_repo(repo_path, repo_name, dry_run=False):
    """Synchronize a single repository. Returns True if sync proceeded."""
    print(f"Checking {repo_name}...")

    if not repo_path.exists():
        print(f"  ❌ Path does not exist: {repo_path}")
        return False

    # 1. Check for local modifications
    if is_dirty(repo_path, dry_run):
        if dry_run:
            print("  ⚠️  (Dry run) Would skip: Uncommitted changes detected.")
        else:
            print("  ⚠️  Skipping: Uncommitted changes detected.")
        return False

    branch = get_current_branch(repo_path, dry_run)
    if not branch:
        print("  ❌ Skipping: Detached HEAD or invalid git state.")
        return False

    # 2. Sync Logic
    if branch in ["main", "jazzy", "rolling"]:
        print(f"  On default branch '{branch}'. Pulling updates...")
        success, output = run_network_cmd(repo_path, ["pull", "--rebase"], dry_run)
        if success:
            if dry_run:
                print("     (Dry run successful)")
            elif "Already up to date." in output:
                print("     ✅ Already up to date.")
            else:
                print(f"     ✅ Updated:\n{output}")
        else:
            print(f"     ❌ Update failed: {output}")

    else:
        print(f"  On feature branch '{branch}'. Fetching only...")
        success, output = run_network_cmd(repo_path, ["fetch"], dry_run)

        if success:
            if dry_run:
                print("     (Dry run successful)")
            else:
                # Check status relative to upstream
                # Assuming upstream is 'origin'
                s_success, s_msg = run_git_cmd(repo_path, ["status", "-sb"], dry_run)
                if s_success and "behind" in s_msg:
                    print(
                        "     ⚠️  Branch is behind remote."
                        " Run 'git merge' or 'git rebase' manually."
                    )
                else:
                    print("     ✅ Fetched.")
        else:
            print(f"     ❌ Fetch failed: {output}")

    return True


def sync_gitbug(repo_path, dry_run=False):
    """Sync git-bug issues for a repo if git-bug is installed and a bridge is configured."""
    if not shutil.which("git-bug"):
        return

    # Check if a bridge is configured in this repo
    try:
        result = subprocess.run(
            ["git", "bug", "bridge", "list"],
            cwd=str(repo_path),
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0 or not result.stdout.strip():
            return
    except OSError:
        return

    repo_name = repo_path.name
    if dry_run:
        print(f"  [DRY-RUN] {repo_name}: git bug pull")
        print(f"  [DRY-RUN] {repo_name}: git bug push")
        return

    print(f"  Syncing git-bug issues for {repo_name}...")
    # git-bug pull/push are network operations against the same remote —
    # route them through the retry wrapper so a rate-limited SSH connection
    # gets the same second chance as the plain git pulls.
    for cmd_args in (["bug", "pull"], ["bug", "push"]):
        success, output = run_network_cmd(repo_path, cmd_args)
        if not success:
            print(f"     ⚠️  git {' '.join(cmd_args)} failed: {output}")
            return
    print("     ✅ git-bug synced.")


def main():
    parser = argparse.ArgumentParser(description="Safely sync workspace repositories.")
    parser.add_argument(
        "--dry-run", action="store_true", help="Simulate actions without executing."
    )
    parser.add_argument(
        "--throttle",
        type=float,
        default=None,
        help=f"Seconds to pause between per-repo network operations. "
        f"Default: none, auto-enabling a {ADAPTIVE_THROTTLE}s pace for the rest "
        f"of the run if the remote drops a connection. "
        f"Pass 0 to disable pacing entirely.",
    )
    args = parser.parse_args()
    if args.throttle is not None and not (math.isfinite(args.throttle) and args.throttle >= 0):
        parser.error("--throttle must be a finite value >= 0")

    # Each run adapts from a clean slate — matters only for in-process
    # callers, but makes the fresh-process assumption explicit.
    reset_transient_error_seen()

    root_dir = SCRIPT_DIR.parent.parent

    # Get repos list using the existing tool
    repos = list_overlay_repos.get_overlay_repos(include_underlay=False)

    throttle = make_throttler(args.throttle, args.dry_run)

    # Also include the root repo itself
    if sync_repo(root_dir, "ros2_agent_workspace", args.dry_run):
        sync_gitbug(root_dir, args.dry_run)

    for repo in repos:
        throttle()
        # Determine workspace directory from source file (e.g. core.repos -> core_ws)
        ws_name = repo["source_file"].replace(".repos", "_ws")
        candidate_path = root_dir / "layers" / "main" / ws_name / "src" / repo["name"]

        repo_path = None
        tried_paths = [str(candidate_path)]

        # First, try the conventional workspace layout.
        if candidate_path.exists():
            repo_path = candidate_path
        else:
            # Fall back to an explicit path if provided by list_overlay_repos.
            explicit_path = repo.get("path")
            if explicit_path:
                explicit_path = Path(explicit_path)
                if not explicit_path.is_absolute():
                    explicit_path = root_dir / explicit_path
                tried_paths.append(str(explicit_path))
                if explicit_path.exists():
                    repo_path = explicit_path

        if repo_path is None:
            paths_str = ", ".join(tried_paths)
            print(
                f"Skipping {repo['name']}: could not resolve repository path (tried {paths_str})."
            )
            continue

        if sync_repo(repo_path, repo["name"], args.dry_run):
            sync_gitbug(repo_path, args.dry_run)

    print("\n✅ Sync complete.")


if __name__ == "__main__":
    main()
