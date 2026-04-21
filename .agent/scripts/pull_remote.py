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
import json as json_mod
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
sys.path.insert(0, str(SCRIPT_DIR / "lib"))

from remote_utils import (  # noqa: E402
    add_common_args,
    get_default_branch,
    get_repos,
    remote_exists,
    resolve_repo_path,
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
    """Compare local and remote branches. Returns (status, message)."""
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
    if current is None:
        return None, "detached HEAD — skipping merge"
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


def _get_ahead_commits(repo_path, branch, remote_ref):
    """Get list of commits on remote not on local. Returns list of dicts."""
    success, output, _ = run_git(repo_path, ["log", "--oneline", f"{branch}..{remote_ref}"])
    if not success or not output:
        return []
    commits = []
    for line in output.splitlines():
        parts = line.split(" ", 1)
        commits.append({"sha": parts[0], "subject": parts[1] if len(parts) > 1 else ""})
    return commits


def _json_report(repo_path, repo_name, version, remote_name):
    """Generate JSON-friendly report for a single repo."""
    branch = get_default_branch(repo_path, version)
    remote_ref = f"{remote_name}/{branch}"

    # Check remote ref exists
    success, _, _ = run_git(repo_path, ["rev-parse", "--verify", remote_ref])
    if not success:
        return None  # no remote branch

    # Count ahead/behind
    success, output, _ = run_git(
        repo_path, ["rev-list", "--left-right", "--count", f"{branch}...{remote_ref}"]
    )
    if not success or not output:
        return None

    parts = output.split()
    if len(parts) != 2:
        return None

    ahead, behind = int(parts[0]), int(parts[1])
    if behind == 0:
        return None  # nothing new on remote

    commits = _get_ahead_commits(repo_path, branch, remote_ref)
    return {
        "repo": repo_name,
        "path": str(repo_path),
        "default_branch": branch,
        "remote_ref": remote_ref,
        "ahead": ahead,
        "behind": behind,
        "diverged": ahead > 0 and behind > 0,
        "commits": commits,
    }


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
    parser.add_argument(
        "--json",
        action="store_true",
        help="Output JSON report of repos with remote-ahead commits (fetch-only mode)",
    )
    args = parser.parse_args()

    if args.json:
        # JSON mode: fetch all, report repos with changes as structured data
        if args.pull or args.branch:
            parser.error("--json cannot be combined with --pull or --branch")

        root_dir = SCRIPT_DIR.parent.parent
        repos = get_repos(args)
        results = []

        # Workspace repo
        run_git(root_dir, ["fetch", args.remote], args.dry_run)
        ws_version = get_default_branch(root_dir, None)
        entry = _json_report(root_dir, "ros2_agent_workspace", ws_version, args.remote)
        if entry:
            results.append(entry)

        for repo in repos:
            repo_path = resolve_repo_path(root_dir, repo)
            if repo_path is None:
                continue
            if not remote_exists(repo_path, args.remote):
                continue
            run_git(repo_path, ["fetch", args.remote], args.dry_run)
            entry = _json_report(repo_path, repo["name"], repo["version"], args.remote)
            if entry:
                results.append(entry)

        print(json_mod.dumps(results, indent=2))
        sys.exit(0)

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
