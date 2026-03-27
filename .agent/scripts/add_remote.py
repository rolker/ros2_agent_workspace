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
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
sys.path.insert(0, str(SCRIPT_DIR / "lib"))

from remote_utils import (
    add_common_args,
    remote_exists,
    run_git,
    run_script,
)
from workspace import extract_github_owner_repo


def get_origin_url(repo_path):
    """Get the origin remote URL for a repo."""
    success, url, _ = run_git(repo_path, ["remote", "get-url", "origin"])
    return url if success else None


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


def process_repo(repo_path, repo_name, version, args):
    """Add a named remote to a single repo. Returns (status, message)."""
    if remote_exists(repo_path, args.remote):
        return "skip", f"remote '{args.remote}' already exists"

    origin_url = get_origin_url(repo_path)
    if not origin_url:
        return "error", "could not read origin URL"

    derived_name = derive_repo_name(origin_url)
    if not derived_name:
        return "error", f"could not derive repo name from: {origin_url}"

    remote_url = f"{args.url_prefix}{derived_name}.git"
    success, _, err = run_git(repo_path, ["remote", "add", args.remote, remote_url], args.dry_run)
    if success:
        return "added", remote_url
    return "error", err


def main():
    parser = argparse.ArgumentParser(
        description="Add a named remote to all workspace repositories."
    )
    add_common_args(parser)
    parser.add_argument(
        "--url-prefix",
        required=True,
        help="URL prefix (e.g., git@gitcloud:field/)",
    )
    run_script(
        SCRIPT_DIR,
        parser.parse_args(),
        process_repo,
        {"added": 0, "skip": 0, "error": 0, "missing": 0},
        [
            ("added", "added"),
            ("skip", "already existed"),
            ("error", "errors"),
            ("missing", "missing"),
        ],
    )


if __name__ == "__main__":
    main()
