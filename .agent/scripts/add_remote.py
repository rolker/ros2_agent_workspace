#!/usr/bin/env python3
"""
Add a named remote to all workspace repositories.

One-time setup for secondary git server sync. Derives the target repo name
from each repo's origin URL, then constructs the remote URL as:
  <url-prefix><repo-name>.git

Usage:
    python3 add_remote.py --remote gitcloud --url-prefix git@gitcloud:field/
    python3 add_remote.py --remote gitcloud --url-prefix git@gitcloud:field/ --dry-run

After setup, use push_remote.py / pull_remote.py for ongoing sync.
"""

import argparse
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
sys.path.insert(0, str(SCRIPT_DIR / "lib"))

from workspace import extract_github_owner_repo, get_overlay_repos


def run_git(repo_path, args, dry_run=False):
    """Run a git command in the given repo. Returns (success, stdout)."""
    cmd = ["git"] + args
    if dry_run:
        print(f"  [DRY-RUN] {' '.join(cmd)}")
        return True, ""
    try:
        result = subprocess.run(cmd, cwd=str(repo_path), capture_output=True, text=True, check=True)
        return True, result.stdout.strip()
    except subprocess.CalledProcessError as e:
        return False, e.stderr.strip()


def get_origin_url(repo_path):
    """Get the origin remote URL for a repo."""
    success, url = run_git(repo_path, ["remote", "get-url", "origin"])
    return url if success else None


def remote_exists(repo_path, remote_name):
    """Check if a remote already exists."""
    success, output = run_git(repo_path, ["remote"])
    if not success:
        return False
    return remote_name in output.splitlines()


def derive_repo_name(origin_url):
    """Derive the repository name from the origin URL."""
    _, repo_name = extract_github_owner_repo(origin_url)
    if repo_name:
        return repo_name
    # Fallback: parse the last path component from any git URL
    path = origin_url.rstrip("/")
    if path.endswith(".git"):
        path = path[:-4]
    return path.rsplit("/", 1)[-1] if "/" in path else None


def resolve_repo_path(root_dir, repo):
    """Resolve the local path for a repo entry. Returns Path or None."""
    ws_name = repo["source_file"].replace(".repos", "_ws")
    candidate = root_dir / "layers" / "main" / ws_name / "src" / repo["name"]
    if candidate.exists():
        return candidate
    return None


def add_remote_to_repo(repo_path, repo_label, remote_name, url_prefix, dry_run):
    """Add a named remote to a single repo. Returns (status, message)."""
    if remote_exists(repo_path, remote_name):
        return "skip", f"remote '{remote_name}' already exists"

    origin_url = get_origin_url(repo_path)
    if not origin_url:
        return "error", "could not read origin URL"

    repo_name = derive_repo_name(origin_url)
    if not repo_name:
        return "error", f"could not derive repo name from: {origin_url}"

    remote_url = f"{url_prefix}{repo_name}.git"
    success, output = run_git(repo_path, ["remote", "add", remote_name, remote_url], dry_run)
    if success:
        return "added", remote_url
    return "error", output


def main():
    parser = argparse.ArgumentParser(
        description="Add a named remote to all workspace repositories."
    )
    parser.add_argument("--remote", required=True, help="Remote name (e.g., gitcloud)")
    parser.add_argument(
        "--url-prefix",
        required=True,
        help="URL prefix (e.g., git@gitcloud:field/)",
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

    results = {"added": 0, "skip": 0, "error": 0, "missing": 0}

    # Workspace repo itself
    print("ros2_agent_workspace (workspace root)")
    status, msg = add_remote_to_repo(
        root_dir, "ros2_agent_workspace", args.remote, args.url_prefix, args.dry_run
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
        status, msg = add_remote_to_repo(
            repo_path, repo["name"], args.remote, args.url_prefix, args.dry_run
        )
        print(f"  {status}: {msg}")
        results[status] = results.get(status, 0) + 1

    # Summary
    total = sum(results.values())
    print(
        f"\nSummary: {total} repos — "
        f"{results['added']} added, {results['skip']} already existed, "
        f"{results['error']} errors, {results['missing']} missing"
    )


if __name__ == "__main__":
    main()
