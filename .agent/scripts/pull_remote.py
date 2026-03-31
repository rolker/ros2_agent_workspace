#!/usr/bin/env python3
"""
Fetch/pull from a named remote for all workspace repositories.

Default mode: fetch and report which repos have new commits, listing them.
With --pull: merge remote changes into the current local branch.
With --branch: pull remote changes into a named local branch.

Usage:
    python3 pull_remote.py --remote gitcloud                    # fetch + report
    python3 pull_remote.py --remote gitcloud --pull             # fetch + merge
    python3 pull_remote.py --remote gitcloud --branch sync/gc   # fetch into branch

Prerequisites:
    Remotes must already exist in each repo. Use add_remote.py for one-time setup.
"""

import argparse
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
sys.path.insert(0, str(SCRIPT_DIR / "lib"))

from remote_utils import (
    add_common_args,
    get_default_branch,
    remote_exists,
    run_git,
    run_script,
)


def is_dirty(repo_path):
    """Check if the repo has uncommitted changes."""
    success, output, _ = run_git(repo_path, ["status", "--porcelain"])
    return success and bool(output)


def get_current_branch(repo_path):
    """Get the current branch name."""
    success, output, _ = run_git(repo_path, ["branch", "--show-current"])
    return output if success and output else None


def _compare_branches(repo_path, branch, remote_ref):
    """Compare local and remote branches. Returns (status, message) or None if not comparable."""
    # Verify both refs exist
    for ref, label in [(remote_ref, "remote"), (branch, "local branch")]:
        success, _, _ = run_git(repo_path, ["rev-parse", "--verify", ref])
        if not success:
            return "skip", f"fetched (no {ref} on {label})"

    # Count commits ahead/behind
    success, output, _ = run_git(
        repo_path,
        ["rev-list", "--left-right", "--count", f"{branch}...{remote_ref}"],
    )
    if not success or not output:
        return "ok", "fetched"

    parts = output.split()
    if len(parts) != 2:
        return "ok", "fetched"

    ahead, behind = int(parts[0]), int(parts[1])
    if ahead == 0 and behind == 0:
        return "ok", "up to date"

    lines = []
    if behind > 0:
        lines.append(f"local is {behind} commit(s) behind {remote_ref}")
    if ahead > 0:
        lines.append(f"local is {ahead} commit(s) ahead of {remote_ref}")

    # List new commits from the remote
    if behind > 0:
        success, log_output, _ = run_git(
            repo_path,
            ["log", "--oneline", f"{branch}..{remote_ref}", "--max-count=20"],
        )
        if success and log_output:
            lines.append("  new remote commits:")
            for line in log_output.splitlines():
                lines.append(f"    {line}")
            if behind > 20:
                lines.append(f"    ... and {behind - 20} more")

    return "changes", "\n  ".join(lines)


def _fetch_and_report(repo_path, remote_name, version, dry_run):
    """Fetch from remote and report new commits. Returns (status, message)."""
    success, _, err = run_git(repo_path, ["fetch", remote_name], dry_run)
    if not success:
        return "error", f"fetch failed: {err}"
    if dry_run:
        return "ok", "fetched (dry run)"

    branch = get_default_branch(repo_path, version)
    remote_ref = f"{remote_name}/{branch}"
    return _compare_branches(repo_path, branch, remote_ref)


def _check_pull_preconditions(repo_path, version):
    """Check if repo is ready for a merge. Returns (branch, skip_reason)."""
    if is_dirty(repo_path):
        return None, "uncommitted changes — skipping merge"

    branch = get_default_branch(repo_path, version)
    current = get_current_branch(repo_path)
    if current != branch:
        return None, f"not on default branch (on '{current}', expected '{branch}')"
    return branch, None


def _fetch_and_pull(repo_path, remote_name, version, dry_run):
    """Fetch and merge from remote. Returns (status, message)."""
    branch, skip_reason = _check_pull_preconditions(repo_path, version)
    if skip_reason:
        return "skip", skip_reason

    # Fetch
    success, _, err = run_git(repo_path, ["fetch", remote_name], dry_run)
    if not success:
        return "error", f"fetch failed: {err}"

    # Merge
    remote_ref = f"{remote_name}/{branch}"
    success, out, err = run_git(repo_path, ["merge", remote_ref, "--ff-only"], dry_run)
    if not success:
        return "error", f"merge failed (non-fast-forward?): {err}"
    if dry_run:
        return "ok", "merged (dry run)"
    if "Already up to date" in out:
        return "ok", "already up to date"
    return "ok", f"merged from {remote_ref}"


def _fetch_into_branch(repo_path, remote_name, version, target_branch, dry_run):
    """Fetch and update a local branch from the remote. Returns (status, message)."""
    # git branch -f fails if the target branch is currently checked out
    current = get_current_branch(repo_path)
    if current == target_branch:
        return "skip", (
            f"branch '{target_branch}' is currently checked out — "
            "cannot force-update; use --pull instead"
        )

    branch = get_default_branch(repo_path, version)

    # Fetch
    success, _, err = run_git(repo_path, ["fetch", remote_name], dry_run)
    if not success:
        return "error", f"fetch failed: {err}"

    remote_ref = f"{remote_name}/{branch}"

    # Check if remote ref exists
    if not dry_run:
        success, _, _ = run_git(repo_path, ["rev-parse", "--verify", remote_ref])
        if not success:
            return "ok", f"fetched (no {remote_ref} on remote)"

    # Create or update the target branch to point at the remote ref
    success, _, err = run_git(repo_path, ["branch", "-f", target_branch, remote_ref], dry_run)
    if not success:
        return "error", f"branch update failed: {err}"

    return "ok", f"branch '{target_branch}' updated to {remote_ref}"


def process_repo(repo_path, repo_name, version, args):
    """Dispatch to the appropriate pull mode. Returns (status, message)."""
    if not remote_exists(repo_path, args.remote):
        return "skip", f"remote '{args.remote}' not found"

    if args.pull:
        return _fetch_and_pull(repo_path, args.remote, version, args.dry_run)
    if args.branch:
        return _fetch_into_branch(repo_path, args.remote, version, args.branch, args.dry_run)
    return _fetch_and_report(repo_path, args.remote, version, args.dry_run)


def main():
    parser = argparse.ArgumentParser(
        description="Fetch/pull from a named remote for all workspace repositories."
    )
    add_common_args(parser)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--pull",
        action="store_true",
        help="Merge remote changes into current branch (fast-forward only)",
    )
    mode.add_argument(
        "--branch",
        help="Create/update a local branch with remote changes (e.g., sync/gitcloud)",
    )
    args = parser.parse_args()

    if args.pull or args.branch:
        labels = [
            ("ok", "updated"),
            ("skip", "skipped"),
            ("error", "errors"),
            ("missing", "missing"),
        ]
    else:
        labels = [
            ("ok", "up to date"),
            ("changes", "with changes"),
            ("skip", "skipped"),
            ("error", "errors"),
            ("missing", "missing"),
        ]

    run_script(
        SCRIPT_DIR,
        args,
        process_repo,
        {"ok": 0, "changes": 0, "skip": 0, "error": 0, "missing": 0},
        labels,
    )


if __name__ == "__main__":
    main()
