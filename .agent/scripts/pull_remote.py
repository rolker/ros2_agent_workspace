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
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
sys.path.insert(0, str(SCRIPT_DIR / "lib"))

from workspace import get_overlay_repos


def run_git(repo_path, args, dry_run=False):
    """Run a git command in the given repo. Returns (success, stdout, stderr)."""
    cmd = ["git"] + args
    if dry_run:
        print(f"  [DRY-RUN] {' '.join(cmd)}")
        return True, "", ""
    try:
        result = subprocess.run(cmd, cwd=str(repo_path), capture_output=True, text=True, check=True)
        return True, result.stdout.strip(), result.stderr.strip()
    except subprocess.CalledProcessError as e:
        return False, e.stdout.strip(), e.stderr.strip()


def remote_exists(repo_path, remote_name):
    """Check if a remote exists in the repo."""
    success, output, _ = run_git(repo_path, ["remote"])
    if not success:
        return False
    return remote_name in output.splitlines()


def is_dirty(repo_path):
    """Check if the repo has uncommitted changes."""
    success, output, _ = run_git(repo_path, ["status", "--porcelain"])
    return success and bool(output)


def get_current_branch(repo_path):
    """Get the current branch name."""
    success, output, _ = run_git(repo_path, ["branch", "--show-current"])
    return output if success and output else None


def resolve_repo_path(root_dir, repo):
    """Resolve the local path for a repo entry. Returns Path or None."""
    ws_name = repo["source_file"].replace(".repos", "_ws")
    candidate = root_dir / "layers" / "main" / ws_name / "src" / repo["name"]
    if candidate.exists():
        return candidate
    return None


def get_default_branch(repo_path, version):
    """Get the default branch for comparison."""
    if version and version != "unknown":
        return version
    success, out, _ = run_git(repo_path, ["symbolic-ref", "refs/remotes/origin/HEAD", "--short"])
    return out.replace("origin/", "") if success and out else "main"


def fetch_and_report(
    repo_path, remote_name, version, dry_run
):  # pylint: disable=too-many-return-statements
    """Fetch from remote and report new commits. Returns (status, message)."""
    if not remote_exists(repo_path, remote_name):
        return "skip", f"remote '{remote_name}' not found"

    # Fetch
    success, _, err = run_git(repo_path, ["fetch", remote_name], dry_run)
    if not success:
        return "error", f"fetch failed: {err}"
    if dry_run:
        return "ok", "fetched (dry run)"

    # Compare local default branch with remote's
    branch = get_default_branch(repo_path, version)
    remote_ref = f"{remote_name}/{branch}"

    # Check if the remote ref exists
    success, _, _ = run_git(repo_path, ["rev-parse", "--verify", remote_ref])
    if not success:
        return "ok", f"fetched (no {remote_ref} on remote)"

    # Check if local branch exists
    success, _, _ = run_git(repo_path, ["rev-parse", "--verify", branch])
    if not success:
        return "ok", f"fetched (local branch {branch} not found)"

    # Count commits ahead/behind
    success, output, _ = run_git(
        repo_path, ["rev-list", "--left-right", "--count", f"{branch}...{remote_ref}"]
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


def fetch_and_pull(
    repo_path, remote_name, version, dry_run
):  # pylint: disable=too-many-return-statements
    """Fetch and merge from remote. Returns (status, message)."""
    if not remote_exists(repo_path, remote_name):
        return "skip", f"remote '{remote_name}' not found"

    if not dry_run and is_dirty(repo_path):
        return "skip", "uncommitted changes — skipping merge"

    branch = get_default_branch(repo_path, version)
    current = get_current_branch(repo_path)
    if not dry_run and current != branch:
        return "skip", f"not on default branch (on '{current}', expected '{branch}')"

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


def fetch_into_branch(repo_path, remote_name, version, target_branch, dry_run):
    """Fetch and update a local branch from the remote. Returns (status, message)."""
    if not remote_exists(repo_path, remote_name):
        return "skip", f"remote '{remote_name}' not found"

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


def main():
    parser = argparse.ArgumentParser(
        description="Fetch/pull from a named remote for all workspace repositories."
    )
    parser.add_argument("--remote", required=True, help="Remote name (e.g., gitcloud)")
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
        "--manifest",
        help="Filter by manifest name(s), comma-separated (e.g., core,platforms)",
    )
    parser.add_argument(
        "--include-underlay",
        action="store_true",
        help="Include underlay repositories",
    )
    parser.add_argument("--dry-run", action="store_true", help="Show what would be done")
    args = parser.parse_args()

    root_dir = SCRIPT_DIR.parent.parent
    manifest_filter = (
        set(f"{m.strip()}.repos" for m in args.manifest.split(",")) if args.manifest else None
    )

    repos = get_overlay_repos(include_underlay=args.include_underlay)
    if manifest_filter:
        repos = [r for r in repos if r["source_file"] in manifest_filter]

    results = {"ok": 0, "changes": 0, "skip": 0, "error": 0, "missing": 0}

    def process_repo(repo_path, repo_name, version):
        print(f"{repo_name}")
        if args.pull:
            status, msg = fetch_and_pull(repo_path, args.remote, version, args.dry_run)
        elif args.branch:
            status, msg = fetch_into_branch(
                repo_path, args.remote, version, args.branch, args.dry_run
            )
        else:
            status, msg = fetch_and_report(repo_path, args.remote, version, args.dry_run)
        print(f"  {status}: {msg}")
        results[status] = results.get(status, 0) + 1

    # Workspace repo itself
    process_repo(root_dir, "ros2_agent_workspace (workspace root)", "main")

    for repo in repos:
        repo_path = resolve_repo_path(root_dir, repo)
        if repo_path is None:
            print(f"{repo['name']}")
            print("  missing: could not find local checkout")
            results["missing"] += 1
            continue

        process_repo(repo_path, repo["name"], repo["version"])

    # Summary
    total = sum(results.values())
    if args.pull or args.branch:
        print(
            f"\nSummary: {total} repos — "
            f"{results['ok']} updated, {results['skip']} skipped, "
            f"{results['error']} errors, {results['missing']} missing"
        )
    else:
        print(
            f"\nSummary: {total} repos — "
            f"{results['ok']} up to date, {results['changes']} with changes, "
            f"{results['skip']} skipped, {results['error']} errors, "
            f"{results['missing']} missing"
        )

    if results["error"] > 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
