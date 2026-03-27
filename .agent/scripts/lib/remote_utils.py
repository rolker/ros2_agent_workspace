"""
Shared utilities for secondary remote management scripts.

Used by add_remote.py, push_remote.py, and pull_remote.py.
"""

import subprocess
import sys

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
    """Check if a named remote exists in the repo."""
    success, output, _ = run_git(repo_path, ["remote"])
    if not success:
        return False
    return remote_name in output.splitlines()


def resolve_repo_path(root_dir, repo):
    """Resolve the local path for a repo entry. Returns Path or None.

    Only checks the main layer tree (layers/main/). These scripts operate
    on the primary workspace checkout, not worktrees.
    """
    ws_name = repo["source_file"].replace(".repos", "_ws")
    candidate = root_dir / "layers" / "main" / ws_name / "src" / repo["name"]
    if candidate.exists():
        return candidate
    return None


def get_default_branch(repo_path, version):
    """Get the default branch for a repo.

    Uses the manifest-declared version if it resolves to a branch,
    otherwise detects from origin/HEAD, falling back to 'main'.
    Tags and commit SHAs are not usable as branch targets for push/pull.
    """
    if version and version != "unknown":
        # Only use the manifest version if it's actually a branch ref
        for ref in [f"refs/heads/{version}", f"refs/remotes/origin/{version}"]:
            success, _, _ = run_git(repo_path, ["rev-parse", "--verify", ref])
            if success:
                return version

    # Detect from origin/HEAD
    success, out, _ = run_git(repo_path, ["symbolic-ref", "refs/remotes/origin/HEAD", "--short"])
    if success and out:
        return out.replace("origin/", "")

    # Fall back to current branch
    success, out, _ = run_git(repo_path, ["symbolic-ref", "HEAD", "--short"])
    if success and out:
        return out

    return "main"


def add_common_args(parser):
    """Add the common --remote, --manifest, --include-underlay, --dry-run args."""
    parser.add_argument("--remote", required=True, help="Remote name (e.g., gitcloud)")
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


def get_repos(args):
    """Get the filtered repo list based on parsed args."""
    repos = get_overlay_repos(include_underlay=args.include_underlay)
    if args.manifest:
        manifest_filter = set(f"{m.strip()}.repos" for m in args.manifest.split(","))
        repos = [r for r in repos if r["source_file"] in manifest_filter]
    return repos


def iter_repos(script_dir, args, process_fn, initial_results):
    """Iterate over all workspace repos, calling process_fn for each.

    Args:
        script_dir: Path to the script's directory (for resolving workspace root).
        args: Parsed argparse namespace (must include common args).
        process_fn: Callable(repo_path, repo_name, version, args) -> (status, message).
        initial_results: Dict of status -> count to accumulate into.

    Returns:
        The results dict with updated counts. Exits non-zero if errors occurred.
    """
    root_dir = script_dir.parent.parent
    repos = get_repos(args)
    results = dict(initial_results)

    # Workspace repo itself — detect its default branch rather than hard-coding
    ws_version = get_default_branch(root_dir, None)
    print("ros2_agent_workspace (workspace root)")
    status, msg = process_fn(root_dir, "ros2_agent_workspace", ws_version, args)
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
        status, msg = process_fn(repo_path, repo["name"], repo["version"], args)
        print(f"  {status}: {msg}")
        results[status] = results.get(status, 0) + 1

    return results


def print_summary_and_exit(results, labels):
    """Print a summary line and exit non-zero if there were errors.

    Args:
        results: Dict of status -> count.
        labels: List of (status_key, label_text) pairs for the summary.
    """
    total = sum(results.values())
    parts = [f"{results.get(key, 0)} {label}" for key, label in labels]
    print(f"\nSummary: {total} repos — {', '.join(parts)}")
    if results.get("error", 0) > 0:
        sys.exit(1)


def run_script(script_dir, args, process_fn, initial_results, labels):
    """Run iter_repos then print_summary_and_exit. Typical main() body."""
    results = iter_repos(script_dir, args, process_fn, initial_results)
    print_summary_and_exit(results, labels)
