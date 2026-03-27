#!/usr/bin/env python3
"""
Push workspace repositories to a named remote.

Iterates all workspace repos and pushes the manifest-declared default branch
plus tags to the named remote. Skips repos where the remote doesn't exist.

Usage:
    python3 push_remote.py --remote gitcloud
    python3 push_remote.py --remote gitcloud --all-branches
    python3 push_remote.py --remote gitcloud --set-default-branch
    python3 push_remote.py --remote gitcloud --manifest core,platforms --dry-run

Prerequisites:
    Remotes must already exist in each repo. Use add_remote.py for one-time setup.

Environment variables:
    FORGEJO_TOKEN   API token for Forgejo/Gitea (required for --set-default-branch).
"""

import argparse
import json
import os
import subprocess
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


def parse_remote_url(repo_path, remote_name):
    """Parse a remote URL into (scheme, host, owner, repo_name).

    Supports SSH (git@host:owner/repo.git) and HTTPS (https://host/owner/repo.git).
    Returns (None, None, None, None) if unparseable.
    """
    success, url, _ = run_git(repo_path, ["remote", "get-url", remote_name])
    if not success or not url:
        return None, None, None, None

    # SSH: git@host:owner/repo.git
    if url.startswith("git@"):
        try:
            host_part, path_part = url[4:].split(":", 1)
            path_part = path_part.rstrip("/")
            if path_part.endswith(".git"):
                path_part = path_part[:-4]
            parts = path_part.split("/")
            if len(parts) >= 2:
                return "ssh", host_part, parts[0], parts[1]
        except ValueError:
            pass

    # HTTPS: https://host/owner/repo.git
    if url.startswith("https://") or url.startswith("http://"):
        scheme = url.split("://")[0]
        path = url.split("://", 1)[1].rstrip("/")
        if path.endswith(".git"):
            path = path[:-4]
        parts = path.split("/")
        if len(parts) >= 3:
            return scheme, parts[0], parts[1], parts[2]

    return None, None, None, None


def _build_forgejo_api_url(repo_path, remote_name):
    """Build the Forgejo API URL for a repo. Returns (url, error_msg)."""
    scheme, host, owner, repo_name = parse_remote_url(repo_path, remote_name)
    if not host:
        return None, "could not parse remote URL for API call"
    api_scheme = "https" if scheme == "ssh" else scheme
    return f"{api_scheme}://{host}/api/v1/repos/{owner}/{repo_name}", None


def set_forgejo_default_branch(repo_path, remote_name, branch, dry_run):
    """Set the default branch on a Forgejo/Gitea server via API.

    Uses the FORGEJO_TOKEN environment variable for authentication.
    Returns (success: bool, message: str).
    """
    api_url, err = _build_forgejo_api_url(repo_path, remote_name)
    if err:
        return False, err

    token = os.environ.get("FORGEJO_TOKEN", "")
    if not token:
        return False, "FORGEJO_TOKEN not set — required for API calls"

    payload = json.dumps({"default_branch": branch})

    if dry_run:
        print(f"  [DRY-RUN] PATCH {api_url} {payload}")
        return True, "default branch set (dry run)"

    try:
        subprocess.run(
            [
                "curl",
                "-sf",
                "-X",
                "PATCH",
                api_url,
                "-H",
                "Content-Type: application/json",
                "-H",
                f"Authorization: token {token}",
                "-d",
                payload,
            ],
            capture_output=True,
            text=True,
            check=True,
            timeout=15,
        )
    except (FileNotFoundError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        return False, f"API call failed: {exc}"
    return True, f"default branch set to '{branch}'"


def process_repo(repo_path, repo_name, version, args):
    """Push a single repo to the named remote. Returns (status, message)."""
    if not remote_exists(repo_path, args.remote):
        return "skip", f"remote '{args.remote}' not found"

    errors = []
    branch = get_default_branch(repo_path, version)

    if args.all_branches:
        success, _, err = run_git(repo_path, ["push", args.remote, "--all"], args.dry_run)
        if not success:
            errors.append(f"push --all failed: {err}")
    else:
        success, _, err = run_git(repo_path, ["push", args.remote, branch], args.dry_run)
        if not success:
            errors.append(f"push {branch} failed: {err}")

    # Always push tags
    success, _, err = run_git(repo_path, ["push", args.remote, "--tags"], args.dry_run)
    if not success:
        errors.append(f"push --tags failed: {err}")

    # Optionally set the default branch on the remote server
    if args.set_default_branch:
        ok, msg = set_forgejo_default_branch(repo_path, args.remote, branch, args.dry_run)
        print(f"  {msg}")
        if not ok:
            errors.append(f"set-default-branch failed: {msg}")

    if errors:
        return "error", "; ".join(errors)
    return "ok", "pushed"


def main():
    parser = argparse.ArgumentParser(description="Push workspace repositories to a named remote.")
    add_common_args(parser)
    parser.add_argument(
        "--all-branches",
        action="store_true",
        help="Push all branches (default: push only manifest-declared branch)",
    )
    parser.add_argument(
        "--set-default-branch",
        action="store_true",
        help="Set default branch on Forgejo/Gitea to match manifest "
        "version (requires FORGEJO_TOKEN)",
    )
    run_script(
        SCRIPT_DIR,
        parser.parse_args(),
        process_repo,
        {"ok": 0, "skip": 0, "error": 0, "missing": 0},
        [("ok", "pushed"), ("skip", "skipped"), ("error", "errors"), ("missing", "missing")],
    )


if __name__ == "__main__":
    main()
