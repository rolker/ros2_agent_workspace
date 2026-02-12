#!/usr/bin/env python3
"""
Full Workspace Status Check

This script performs a comprehensive status check combining:
1. Local git status (via status_report.sh)
2. Remote GitHub status (PRs and Issues)

This consolidates what previously required 5 separate commands into one.

Usage:
    python3 check_full_status.py [--no-local] [--no-remote] [--batch-size N]

Options:
    --no-local      Skip local git status check
    --no-remote     Skip remote GitHub status check
    --batch-size N  Number of repositories per GitHub query (default: 10)
"""

import os
import sys
import subprocess
import json
import argparse
from datetime import datetime

# Add lib directory to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(SCRIPT_DIR, "lib"))

from workspace import get_overlay_repos, extract_github_owner_repo, get_workspace_root

# Always include the root repository
ROOT_REPO = "rolker/ros2_agent_workspace"


def run_command(cmd, shell=False):
    """Run a command and return its output."""
    try:
        result = subprocess.run(cmd, shell=shell, capture_output=True, text=True, check=False)
        return result.returncode, result.stdout, result.stderr
    except Exception as e:
        return -1, "", str(e)


def check_gh_auth():
    """Check if GitHub CLI is authenticated."""
    returncode, stdout, stderr = run_command(["gh", "auth", "status"])
    return returncode == 0


def get_local_status():
    """Run status_report.sh and return its output."""
    workspace_root = get_workspace_root()
    status_script = os.path.join(workspace_root, ".agent/scripts/status_report.sh")

    if not os.path.exists(status_script):
        return "**Error**: status_report.sh not found"

    returncode, stdout, stderr = run_command(["bash", status_script])

    if returncode != 0:
        return f"**Error running status_report.sh**:\n{stderr}"

    return stdout


def batch_repos(repos, batch_size):
    """Split repositories into batches."""
    for i in range(0, len(repos), batch_size):
        yield repos[i : i + batch_size]


def build_repo_query(repo_specs, query_type):
    """
    Build a GitHub search query for multiple repositories.

    Args:
        repo_specs: List of "owner/repo" strings
        query_type: "pr" or "issue"

    Returns:
        str: GitHub search query
    """
    if not repo_specs:
        return ""

    # Build the query with repo filters
    repo_filters = " OR ".join([f"repo:{spec}" for spec in repo_specs])

    if query_type == "pr":
        return f"({repo_filters}) is:pr is:open"
    elif query_type == "issue":
        return f"({repo_filters}) is:issue is:open"
    else:
        return ""


def fetch_github_prs(repo_specs, batch_size=10):
    """
    Fetch open pull requests for the given repositories.

    Args:
        repo_specs: List of "owner/repo" strings
        batch_size: Number of repos per query

    Returns:
        list: List of PR dictionaries
    """
    all_prs = []

    for batch in batch_repos(repo_specs, batch_size):
        query = build_repo_query(batch, "pr")

        if not query:
            continue

        # Use gh search prs with the constructed query
        cmd = [
            "gh",
            "search",
            "prs",
            query,
            "--json",
            "repository,title,author,number,url,createdAt",
        ]
        returncode, stdout, stderr = run_command(cmd)

        if returncode != 0:
            print(f"Warning: Failed to fetch PRs for batch: {stderr}", file=sys.stderr)
            continue

        try:
            prs = json.loads(stdout)
            all_prs.extend(prs)
        except json.JSONDecodeError as e:
            print(f"Warning: Failed to parse PR JSON: {e}", file=sys.stderr)

    return all_prs


def fetch_github_issues(repo_specs, batch_size=10):
    """
    Fetch open issues for the given repositories.

    Args:
        repo_specs: List of "owner/repo" strings
        batch_size: Number of repos per query

    Returns:
        list: List of issue dictionaries
    """
    all_issues = []

    for batch in batch_repos(repo_specs, batch_size):
        query = build_repo_query(batch, "issue")

        if not query:
            continue

        # Use gh search issues with the constructed query
        cmd = [
            "gh",
            "search",
            "issues",
            query,
            "--json",
            "repository,title,number,url,labels,createdAt",
        ]
        returncode, stdout, stderr = run_command(cmd)

        if returncode != 0:
            print(f"Warning: Failed to fetch issues for batch: {stderr}", file=sys.stderr)
            continue

        try:
            issues = json.loads(stdout)
            all_issues.extend(issues)
        except json.JSONDecodeError as e:
            print(f"Warning: Failed to parse issue JSON: {e}", file=sys.stderr)

    return all_issues


def get_remote_status(batch_size=10):
    """Get GitHub status (PRs and Issues) for workspace repositories."""
    # Check if gh is authenticated
    if not check_gh_auth():
        return "**Error**: GitHub CLI is not authenticated. Run `gh auth login`."

    # Get repository list
    repos = get_overlay_repos(include_underlay=False)

    # Extract GitHub owner/repo specs
    repo_specs = {ROOT_REPO}  # Always include root repo

    for repo in repos:
        owner, repo_name = extract_github_owner_repo(repo.get("url", ""))
        if owner and repo_name:
            repo_specs.add(f"{owner}/{repo_name}")

    repo_specs = sorted(list(repo_specs))

    if not repo_specs:
        return "**Warning**: No GitHub repositories found in workspace configuration."

    # Fetch PRs and Issues
    prs = fetch_github_prs(repo_specs, batch_size)
    issues = fetch_github_issues(repo_specs, batch_size)

    # Generate report
    output = []
    output.append("## Remote GitHub Status")
    output.append(f"**Checked {len(repo_specs)} repositories**")
    output.append("")

    # PRs section
    output.append(f"### Open Pull Requests ({len(prs)})")
    output.append("")

    if prs:
        output.append("| Repository | Title | Author | PR # |")
        output.append("|------------|-------|--------|------|")

        for pr in prs:
            repo_name = pr.get("repository", {}).get("nameWithOwner", "unknown")
            title = pr.get("title", "")
            author = pr.get("author", {}).get("login", "unknown")
            number = pr.get("number", "")
            url = pr.get("url", "")

            # Truncate long titles
            if len(title) > 60:
                title = title[:57] + "..."

            output.append(f"| {repo_name} | [{title}]({url}) | @{author} | #{number} |")
    else:
        output.append("*No open pull requests*")

    output.append("")

    # Issues section
    output.append(f"### Open Issues ({len(issues)})")
    output.append("")

    if issues:
        output.append("| Repository | Title | Labels | Issue # |")
        output.append("|------------|-------|--------|---------|")

        for issue in issues:
            repo_name = issue.get("repository", {}).get("nameWithOwner", "unknown")
            title = issue.get("title", "")
            number = issue.get("number", "")
            url = issue.get("url", "")
            labels = issue.get("labels", [])
            label_names = ", ".join([l.get("name", "") for l in labels[:3]])  # Max 3 labels

            # Truncate long titles
            if len(title) > 50:
                title = title[:47] + "..."

            output.append(f"| {repo_name} | [{title}]({url}) | {label_names} | #{number} |")
    else:
        output.append("*No open issues*")

    output.append("")

    return "\n".join(output)


def generate_report(local_status=None, remote_status=None):
    """Generate the consolidated status report."""
    output = []

    output.append("# 🔍 Workspace Status Report")
    output.append(f"**Generated**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    output.append("")
    output.append("---")
    output.append("")

    if local_status:
        output.append(local_status)
        output.append("")
        output.append("---")
        output.append("")

    if remote_status:
        output.append(remote_status)

    return "\n".join(output)


def main():
    parser = argparse.ArgumentParser(
        description="Comprehensive workspace status check (local + remote)"
    )
    parser.add_argument("--no-local", action="store_true", help="Skip local git status check")
    parser.add_argument("--no-remote", action="store_true", help="Skip remote GitHub status check")
    parser.add_argument(
        "--batch-size",
        type=int,
        default=10,
        help="Number of repositories per GitHub query (default: 10)",
    )

    args = parser.parse_args()

    # Validate batch size
    if args.batch_size < 1:
        parser.error("--batch-size must be at least 1")

    local_status = None
    remote_status = None

    if not args.no_local:
        local_status = get_local_status()

    if not args.no_remote:
        remote_status = get_remote_status(args.batch_size)

    report = generate_report(local_status, remote_status)
    print(report)


if __name__ == "__main__":
    main()
