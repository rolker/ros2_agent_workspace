#!/usr/bin/env python3
"""
Push workspace repositories to a named remote.

Iterates all workspace repos and pushes the manifest-declared default branch
plus tags to the named remote. Skips repos where the remote doesn't exist.

Usage:
    python3 push_remote.py --remote gitcloud
    python3 push_remote.py --remote gitcloud --all-branches
    python3 push_remote.py --remote gitcloud --manifest core,platforms --dry-run

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


def resolve_repo_path(root_dir, repo):
    """Resolve the local path for a repo entry. Returns Path or None."""
    ws_name = repo["source_file"].replace(".repos", "_ws")
    candidate = root_dir / "layers" / "main" / ws_name / "src" / repo["name"]
    if candidate.exists():
        return candidate
    return None


def push_repo(
    repo_path, repo_name, remote_name, version, all_branches, dry_run
):  # pylint: disable=too-many-arguments
    """Push a single repo to the named remote. Returns (status, message)."""
    if not remote_exists(repo_path, remote_name):
        return "skip", f"remote '{remote_name}' not found"

    errors = []

    if all_branches:
        # Push all branches
        success, out, err = run_git(repo_path, ["push", remote_name, "--all"], dry_run)
        if not success:
            errors.append(f"push --all failed: {err}")
    else:
        # Push only the manifest-declared branch
        branch = version if version and version != "unknown" else None
        if not branch:
            # Try to detect default branch
            success, out, _ = run_git(
                repo_path, ["symbolic-ref", "refs/remotes/origin/HEAD", "--short"]
            )
            branch = out.replace("origin/", "") if success and out else "main"

        success, out, err = run_git(repo_path, ["push", remote_name, branch], dry_run)
        if not success:
            errors.append(f"push {branch} failed: {err}")

    # Always push tags
    success, out, err = run_git(repo_path, ["push", remote_name, "--tags"], dry_run)
    if not success:
        errors.append(f"push --tags failed: {err}")

    if errors:
        return "error", "; ".join(errors)
    return "ok", "pushed"


def main():
    parser = argparse.ArgumentParser(description="Push workspace repositories to a named remote.")
    parser.add_argument("--remote", required=True, help="Remote name (e.g., gitcloud)")
    parser.add_argument(
        "--all-branches",
        action="store_true",
        help="Push all branches (default: push only manifest-declared branch)",
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

    results = {"ok": 0, "skip": 0, "error": 0, "missing": 0}

    # Workspace repo itself
    print("ros2_agent_workspace (workspace root)")
    status, msg = push_repo(
        root_dir,
        "ros2_agent_workspace",
        args.remote,
        "main",
        args.all_branches,
        args.dry_run,
    )
    print(f"  {status}: {msg}")
    results[status] = results.get(status, 0) + 1

    for repo in repos:
        repo_path = resolve_repo_path(root_dir, repo)
        if repo_path is None:
            print(f"{repo['name']}")
            print("  missing: could not find local checkout")
            results["missing"] += 1
            continue

        print(f"{repo['name']}")
        status, msg = push_repo(
            repo_path,
            repo["name"],
            args.remote,
            repo["version"],
            args.all_branches,
            args.dry_run,
        )
        print(f"  {status}: {msg}")
        results[status] = results.get(status, 0) + 1

    # Summary
    total = sum(results.values())
    print(
        f"\nSummary: {total} repos — "
        f"{results['ok']} pushed, {results['skip']} skipped, "
        f"{results['error']} errors, {results['missing']} missing"
    )

    if results["error"] > 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
