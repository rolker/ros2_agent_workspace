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


def process_repo(repo_path, repo_name, version, args):
    """Push a single repo to the named remote. Returns (status, message)."""
    if not remote_exists(repo_path, args.remote):
        return "skip", f"remote '{args.remote}' not found"

    errors = []

    if args.all_branches:
        success, _, err = run_git(repo_path, ["push", args.remote, "--all"], args.dry_run)
        if not success:
            errors.append(f"push --all failed: {err}")
    else:
        branch = get_default_branch(repo_path, version)
        success, _, err = run_git(repo_path, ["push", args.remote, branch], args.dry_run)
        if not success:
            errors.append(f"push {branch} failed: {err}")

    # Always push tags
    success, _, err = run_git(repo_path, ["push", args.remote, "--tags"], args.dry_run)
    if not success:
        errors.append(f"push --tags failed: {err}")

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
    run_script(
        SCRIPT_DIR,
        parser.parse_args(),
        process_repo,
        {"ok": 0, "skip": 0, "error": 0, "missing": 0},
        [("ok", "pushed"), ("skip", "skipped"), ("error", "errors"), ("missing", "missing")],
    )


if __name__ == "__main__":
    main()
