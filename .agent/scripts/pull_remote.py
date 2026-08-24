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
    python3 pull_remote.py --remote gitcloud --json             # fetch + JSON output

Prerequisites:
    Remotes must already exist in each repo. Use add_remote.py for one-time setup.

Exit status:
    0  every repo was fetched/merged, or deliberately skipped (dirty tree,
       detached HEAD, off the default branch, remote not configured).
    1  at least one repo errored: a failed fetch/merge, or a working tree or
       git state that could not be read at all. A probe that failed is not a
       benign answer — see is_dirty()/get_current_branch() (#609).
    2  argparse usage error.

    `make pull-remote` reports GNU make's own 2 for any non-zero recipe status,
    so branch on these codes only when calling the script directly.
"""

import argparse
import json as json_mod
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
sys.path.insert(0, str(SCRIPT_DIR / "lib"))

from remote_utils import (  # noqa: E402
    RemoteState,
    add_common_args,
    get_default_branch,
    get_repos,
    remote_probe,
    resolve_repo_path,
    run_git,
    run_git_network,
    run_script,
)


# Reasons for a repo whose git state could not be read at all. These are
# "error", never "skip": a probe that failed is not a benign answer, and
# reporting it as one is what let a failed run exit 0 (#609).
UNREADABLE_TREE = "cannot read working tree (corrupt .git, or not a repo)"
UNREADABLE_STATE = "cannot read git state (not a repo, or corrupt .git)"


def is_dirty(repo_path):
    """Check if the repo has uncommitted changes.

    Returns True (dirty), False (clean), or None when `git status` itself
    failed. The three cases must stay distinct: a working tree we could not
    read is NOT a clean one. The old `success and bool(output)` reported
    exactly that, and this script is where it costs the most — `--pull`
    *merges* on this answer, so an unreadable working tree with real
    uncommitted changes would have been merged into (#609).
    """
    success, output, _ = run_git(repo_path, ["status", "--porcelain"])
    if not success:
        return None
    return bool(output)


def get_current_branch(repo_path):
    """Get the current branch name.

    Returns the branch name, or "" on a genuine detached HEAD (the command
    succeeded and printed nothing), or None when the git command itself failed
    (not a repo, corrupt .git). Callers MUST distinguish those two: "" is a
    benign skip, None is a repo we cannot read at all (#609).
    """
    success, output, _ = run_git(repo_path, ["branch", "--show-current"])
    if not success:
        return None
    return output


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
    success, _, err = run_git_network(repo_path, ["fetch", remote_name], dry_run)
    if not success:
        return "error", f"fetch failed: {err}"
    if dry_run:
        return "ok", "fetched (dry run)"

    branch = get_default_branch(repo_path, version)
    remote_ref = f"{remote_name}/{branch}"
    return _compare_branches(repo_path, branch, remote_ref)


def _check_pull_preconditions(repo_path, version):
    """Check if repo is ready for a merge. Returns (branch, problem).

    `problem` is None when the repo is ready to merge, otherwise a
    (status, message) pair ready to return from process_repo(). "skip" is for
    a benign, operator-visible reason that leaves the repo deliberately alone
    (dirty tree, detached HEAD, off the default branch); "error" is for a git
    state we could not read, which must not be reported as a benign skip and
    must make the run exit non-zero (#609).
    """
    dirty = is_dirty(repo_path)
    if dirty is None:
        return None, ("error", UNREADABLE_TREE)
    if dirty:
        return None, ("skip", "uncommitted changes — skipping merge")

    branch = get_default_branch(repo_path, version)
    current = get_current_branch(repo_path)
    if current is None:
        # The git command itself failed. NOT the same as a detached HEAD:
        # collapsing the two reported a repo we cannot read as a benign skip,
        # and the run still exited 0 (#609).
        return None, ("error", UNREADABLE_STATE)
    if not current:
        return None, ("skip", "detached HEAD — skipping merge")
    if current != branch:
        return None, ("skip", f"not on default branch (on '{current}', expected '{branch}')")
    return branch, None


def _fetch_and_pull(repo_path, remote_name, version, dry_run):
    """Fetch and merge from remote. Returns (status, message)."""
    branch, problem = _check_pull_preconditions(repo_path, version)
    if problem:
        return problem

    # Fetch
    success, _, err = run_git_network(repo_path, ["fetch", remote_name], dry_run)
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
    if current is None:
        # We cannot tell whether target_branch is checked out, so we cannot
        # know that `git branch -f` is safe here (#609).
        return "error", UNREADABLE_STATE
    if current == target_branch:
        return "skip", (
            f"branch '{target_branch}' is currently checked out — "
            "cannot force-update; use --pull instead"
        )

    branch = get_default_branch(repo_path, version)

    # Fetch
    success, _, err = run_git_network(repo_path, ["fetch", remote_name], dry_run)
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
    state = remote_probe(repo_path, args.remote)
    if state is RemoteState.UNREADABLE:
        # `git remote` failed, so we never learned whether the remote is
        # configured. Skipping on that reads as "nothing to do here" and keeps
        # the run green over a repo that was never fetched (#609).
        return "error", UNREADABLE_STATE
    if state is RemoteState.ABSENT:
        return "skip", f"remote '{args.remote}' not found"

    if args.pull:
        return _fetch_and_pull(repo_path, args.remote, version, args.dry_run)
    if args.branch:
        return _fetch_into_branch(repo_path, args.remote, version, args.branch, args.dry_run)
    return _fetch_and_report(repo_path, args.remote, version, args.dry_run)


def _get_ahead_commits(repo_path, branch, remote_ref, max_count=50):
    """Get list of commits on remote not on local. Returns list of dicts."""
    success, output, _ = run_git(
        repo_path, ["log", "--oneline", f"--max-count={max_count}", f"{branch}..{remote_ref}"]
    )
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
    mode.add_argument(
        "--json",
        action="store_true",
        help="Output JSON report of repos with remote-ahead commits (fetch-only mode)",
    )
    args = parser.parse_args()

    if args.json:
        root_dir = SCRIPT_DIR.parent.parent
        repos = get_repos(args)
        results = []
        errors = []

        # Workspace repo
        success, _, err = run_git_network(root_dir, ["fetch", args.remote], args.dry_run)
        if not success:
            errors.append(f"ros2_agent_workspace: fetch failed: {err}")
        else:
            ws_version = get_default_branch(root_dir, None)
            entry = _json_report(root_dir, "ros2_agent_workspace", ws_version, args.remote)
            if entry:
                results.append(entry)

        for repo in repos:
            repo_path = resolve_repo_path(root_dir, repo)
            if repo_path is None:
                continue
            state = remote_probe(repo_path, args.remote)
            if state is RemoteState.UNREADABLE:
                errors.append(f"{repo['name']}: {UNREADABLE_STATE}")
                continue
            if state is RemoteState.ABSENT:
                continue
            success, _, err = run_git_network(repo_path, ["fetch", args.remote], args.dry_run)
            if not success:
                errors.append(f"{repo['name']}: fetch failed: {err}")
                continue
            entry = _json_report(repo_path, repo["name"], repo["version"], args.remote)
            if entry:
                results.append(entry)

        if errors:
            for e in errors:
                print(f"ERROR: {e}", file=sys.stderr)

        print(json_mod.dumps(results, indent=2))
        sys.exit(1 if errors else 0)

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
