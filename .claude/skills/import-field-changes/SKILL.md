---
name: import-field-changes
description: Batch-import field changes from a secondary remote (e.g., gitcloud) back to GitHub for review. For each repo with remote-ahead commits, creates an issue, opens a draft PR, and pre-reviews the diff against the Quality Standard.
---

# Import Field Changes

## Usage

```
/import-field-changes
```

## Overview

Batch-import field changes from a secondary remote (e.g., gitcloud) back to
GitHub for review. For each repo with remote-ahead commits: creates an issue,
opens a draft PR, and pre-reviews the diff against the Quality Standard.

**Lifecycle position**: field deployment → push to gitcloud → **import-field-changes** → triage-reviews → merge

## Steps

### 1. Read project config

Read `.agent/project_config.yaml` for the field remote name:

```yaml
# .agent/project_config.yaml (gitignored)
field_remote: gitcloud
```

If the file doesn't exist or `field_remote` is missing, stop with:

> `.agent/project_config.yaml` not found or missing `field_remote` key.
> Create it with: `echo "field_remote: gitcloud" > .agent/project_config.yaml`

### 2. Fetch and detect changes

Run from the workspace root:

```bash
python3 .agent/scripts/pull_remote.py --remote <field_remote> --json
```

This fetches all repos and outputs a JSON array of repos with remote-ahead
commits, including: repo name, path, default branch, ahead/behind counts,
diverged flag, and commit list.

If the result is empty, report "No field changes to import" and stop.

### 3. For each repo with changes

Process repos sequentially:

#### 3a. Summarize the diff

Read the diff between local default branch and remote:

```bash
git -C <path> diff <default_branch>..<remote>/<default_branch> --stat
git -C <path> log --oneline <default_branch>..<remote>/<default_branch>
```

#### 3b. Pre-review against Quality Standard

Examine the diff for Quality Standard concerns:
- Are there tests for new functionality?
- Do topic names match actual published topics (check for remap mismatches)?
- Is error handling present for failure modes?
- Are scripts idempotent?
- Any hardcoded paths or credentials?

Note findings for the issue body.

#### 3c. Create issue in the project repo

Title: `Field import: <repo_name> (<YYYY-MM-DD>)`

Body should include:
- List of commits being imported
- Pre-review findings (if any)
- Whether the repo is diverged (merge needed)

Use `.agent/scripts/gh_create_issue.sh` from the project repo directory.

#### 3d. Create branch and PR

Create the branch without checking it out (avoids changing main tree HEAD):

**Non-diverged case** (remote ahead, local not ahead):
```bash
git -C <path> branch feature/issue-<N> <remote>/<default_branch>
git -C <path> push -u origin feature/issue-<N>
```

**Diverged case** (both sides have commits):
```bash
# Branch from remote HEAD — merge will happen in worktree
git -C <path> branch feature/issue-<N> <remote>/<default_branch>
git -C <path> push -u origin feature/issue-<N>
```

Create draft PR (run from within the project repo directory):
```bash
cd <path>
gh pr create --draft --title "Field import: <repo_name> (<date>)" \
  --body-file <body> --base <default_branch>
```

Note: for diverged repos, note in the PR body that a merge with the
default branch is needed before this can be merged.

#### 3e. Handle pre-review findings

If pre-review found issues that should be fixed:
1. Create a worktree for the issue
2. Apply fixes (add tests, fix topic names, etc.)
3. Commit and push to the PR branch
4. Note fixes in the PR description

If no issues found, the PR is ready for Copilot review as-is.

### 4. Report summary

Output a table:

```markdown
## Field Import Summary

| Repo | Commits | Diverged | Issue | PR | Pre-review |
|------|---------|----------|-------|-----|------------|
| <name> | <N> | Yes/No | #<N> | #<N> | Clean / <N> findings |
```

## Guidelines

- **Never edit in the main tree** — all fixes go through worktrees
- **One issue per repo** — even if the repo has multiple unrelated field commits
- **Pre-review is advisory** — findings go in the issue, not auto-fixed unless
  the fix is trivial and unambiguous (e.g., missing shebang)
- **Diverged repos need human judgment** — report them prominently, don't
  auto-merge
- **This skill does not resync gitcloud** — after PRs merge, use
  `push_remote.py` manually to update gitcloud
