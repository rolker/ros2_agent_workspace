---
name: issue-triage
description: Scan overlay repositories for GitHub issues, categorize them, flag stale items, and cross-reference with workspace tracking.
---

# Issue Triage

## Usage

```
/issue-triage [--repo <repo-name>] [--stale-days <N>]
```

Without `--repo`, scans all overlay repositories. Default stale threshold is
90 days.

## Overview

**Lifecycle position**: Utility/periodic — run to get a cross-repo view of
open issues, identify stale items, and ensure nothing is falling through the
cracks.

Scans overlay repositories for open GitHub issues using `gh` CLI, categorizes
them by type and priority, flags stale issues, and cross-references with
workspace-level tracking.

## Steps

### 1. Enumerate repositories

```bash
python3 .agent/scripts/list_overlay_repos.py
```

This outputs a JSON list of `{name, url, version, source_file}` for all overlay repos.
Parse the `owner/repo` from each URL.

If `--repo` was specified, filter to just that repository.

### 2. Fetch open issues per repo

For each repository:

```bash
gh issue list --repo <owner/repo> --state open --json number,title,labels,createdAt,updatedAt,url,assignees --limit 100
```

Collect all results into a unified list with the repo name attached.

### 3. Categorize issues

Classify each issue by type based on labels and title keywords:

| Category | Indicators |
|----------|------------|
| Bug | `bug` label, "fix", "crash", "error" in title |
| Enhancement | `enhancement` label, "add", "improve", "support" in title |
| Documentation | `documentation` label, "doc", "readme" in title |
| Test | `test` label, "test", "coverage" in title |
| Infrastructure | `ci`, `build`, `infra` labels |
| Uncategorized | No matching indicators |

### 4. Flag stale issues

An issue is stale if:
- `updatedAt` is more than `<stale-days>` days ago (default: 90)
- It has no assignee

The `updatedAt` field already reflects all activity (comments, label changes,
assignments), so no separate comment check is needed.

### 5. Cross-reference with workspace

Check whether issues are being tracked in the workspace:

```bash
# Check for existing worktrees or branches referencing the issue
git branch --list "feature/issue-<N>" "feature/ISSUE-<N>-*" 2>/dev/null
```

Also check if the issue number appears in any open PRs:

```bash
gh pr list --repo <workspace-repo> --state open --json title,url --jq '.[] | select(.title | test("\\b<N>\\b"))'
```

### 6. Generate report

```markdown
## Issue Triage Report

**Scanned**: <N> repositories
**Total open issues**: <N>
**Stale issues**: <N> (> <stale-days> days without update)

### By Repository

#### <repo-name>

| # | Title | Category | Age | Stale | Tracked |
|---|-------|----------|-----|-------|---------|
| <N> | <title> | Bug/Enhancement/... | <days> days | Yes/No | Yes/No |

### Summary by Category

| Category | Count |
|----------|-------|
| Bug | <N> |
| Enhancement | <N> |
| ... | ... |

### Stale Issues (Action Needed)

| Repo | # | Title | Last Updated | Suggestion |
|------|---|-------|--------------|------------|
| <repo> | <N> | <title> | <date> | Close / Assign / Update |

### Untracked Issues

Issues not referenced in any workspace branch or PR:

| Repo | # | Title | Category |
|------|---|-------|----------|
| <repo> | <N> | <title> | <category> |
```

## Guidelines

- **Read-only** — this skill reports, it does not create issues, close them,
  or modify labels.
- **Use `gh` CLI** — all GitHub queries go through `gh`, not the API directly.
- **Respect rate limits** — for workspaces with many repos, consider scanning
  in batches. The `--repo` flag helps focus on one repo at a time.
- **Cross-repo awareness** — issues may reference other repos. Note
  cross-references but don't follow them recursively.
- **Stale ≠ invalid** — stale issues may still be relevant. Flag them for
  human review, don't recommend closing without context.
