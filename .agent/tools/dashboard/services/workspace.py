"""Read test/build results and work plans from workspace scratchpad files."""

import json
import os
import subprocess


def get_test_summary(worktree_path):
    """Read test_summary.json from worktree scratchpad."""
    for scratchpad in [".scratchpad", ".agent/scratchpad"]:
        path = os.path.join(worktree_path, scratchpad, "test_summary.json")
        try:
            with open(path) as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            continue
    return None


def get_build_report(worktree_path):
    """Read build_report.md from worktree scratchpad. Returns raw markdown."""
    for scratchpad in [".scratchpad", ".agent/scratchpad"]:
        path = os.path.join(worktree_path, scratchpad, "build_report.md")
        try:
            with open(path) as f:
                return f.read()
        except FileNotFoundError:
            continue
    return None


def get_work_plan(issue_number, worktree_path, workspace_root):
    """Find the work plan for an issue.

    Checks:
    1. Worktree-local work-plans directory
    2. Workspace work-plans directory
    """
    if issue_number is None:
        return None

    filenames = [
        f"PLAN_ISSUE-{issue_number}.md",
        f"PLAN_ISSUE_{issue_number}.md",
    ]

    search_dirs = [
        os.path.join(worktree_path, ".agent", "work-plans"),
        os.path.join(workspace_root, ".agent", "work-plans"),
    ]

    for search_dir in search_dirs:
        for filename in filenames:
            path = os.path.join(search_dir, filename)
            if os.path.exists(path):
                with open(path) as f:
                    return {"source": "file", "path": path, "content": f.read()}

    return None


def get_changed_files(worktree_path):
    """Get list of changed files in worktree via git.

    Uses git status --porcelain to match the files_changed count reported by
    worktree_list.sh (which also uses git status). This includes untracked files,
    avoiding a UI discrepancy where the count exceeds the displayed list.

    Returns a list of {"status": str, "path": str} dicts parsed from porcelain
    output (format: "XY path" where XY is the 2-char status code).
    """
    try:
        result = subprocess.run(
            ["git", "status", "--porcelain"],
            capture_output=True,
            text=True,
            timeout=5,
            cwd=worktree_path,
        )
        if result.returncode == 0 and result.stdout.strip():
            entries = []
            for line in result.stdout.strip().split("\n"):
                if len(line) >= 4:  # "XY path" minimum
                    entries.append({"status": line[:2].strip() or "?", "path": line[3:]})
            return entries
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass
    return []
