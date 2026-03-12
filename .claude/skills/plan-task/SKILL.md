---
name: plan-task
description: Generate a principles-aware work plan for an issue. Saves to `.agent/work-plans/` in the repo that owns the issue and commits as the first step on the feature branch.
---

# Plan Task

## Usage

```
/plan-task <issue-number>
```

## Overview

Generate a work plan for an issue with principles and ADR awareness. This
complements EnterPlanMode — it adds the governance lens, not a competing
planning system. Use EnterPlanMode for general codebase exploration and
approach design; use this skill to ensure the plan accounts for workspace
principles, ADR compliance, and downstream consequences.

**Lifecycle position**: review-issue → **plan-task** → implement → review-pr

## Steps

### 1. Read the issue and any review comments

```bash
gh issue view <N> --json title,body,labels,comments,url
```

Check for review-issue comments — they contain scope assessment, principle
flags, and ADR notes that should inform the plan.

### 2. Load governance context

- `.agent/knowledge/principles_review_guide.md` — evaluation criteria
- `docs/PRINCIPLES.md` — workspace principles
- `docs/decisions/*.md` — ADR titles (read triggered ADRs in full)
- Project-level governance if the issue targets a project repo

### 3. Explore the codebase

Read relevant files to understand the current state:
- Files that will be modified
- Tests that exist for those files
- Documentation that references them
- Use the consequences map to identify what else may be affected

### 4. Ensure correct worktree

Before writing or committing the plan, verify you are in a worktree for the
target issue. If the plan is committed from a different worktree, it ends up
on the wrong branch.

Check `$WORKTREE_ISSUE` (set by `worktree_enter.sh`). If it does not match
`<N>`, or is unset, create and enter the correct worktree:

**Workspace issues** (changes to `.agent/`, `docs/`, configs, skills):

```bash
.agent/scripts/worktree_create.sh --issue <N> --type workspace
source .agent/scripts/worktree_enter.sh --issue <N>
```

**Project repo issues** (changes to ROS packages in `layers/main/`):

```bash
.agent/scripts/worktree_create.sh --issue <N> --type layer --layer <layer> --packages <project_repo>
source .agent/scripts/worktree_enter.sh --issue <N>
```

To determine the worktree type and parameters:

1. Check which GitHub repo the issue belongs to — use `gh issue view <N>
   --json url` and compare against the workspace repo URL. If the issue is
   in a project repo, use `--type layer`.
2. Infer the layer and package from the issue body, labels, or the repo
   name. Project repos live under `layers/main/<layer>_ws/src/<project_repo>/`
   — use `ls` to find the matching layer.
3. If the layer or package cannot be determined, ask the user.

If a worktree for the issue already exists, just enter it.

### 5. Generate the plan

Write a plan to `.agent/work-plans/PLAN_ISSUE-<N>.md` (relative to the current
repo — workspace repo for workspace issues, project repo for project issues):

```markdown
# Plan: <issue-title>

## Issue

<issue URL from: gh issue view <N> --json url --jq '.url'>

## Context

<Brief summary of current state and what needs to change>

## Approach

<Step-by-step implementation plan>

1. **<step>** — <what and why>
2. ...

## Files to Change

| File | Change |
|------|--------|
| `path/to/file` | Description of change |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| <relevant principle> | How this plan accounts for it |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| <relevant ADR> | Yes/No | <explanation> |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| <item from consequences map> | <dependent item> | Yes / No — follow-up |

## Open Questions

- <anything that needs human input before implementation>

## Estimated Scope

<single PR / multiple PRs / needs breakdown>
```

### 6. Commit the plan

The plan file path is relative to the current repo (workspace or project):

```bash
git add .agent/work-plans/PLAN_ISSUE-<N>.md
git commit -m "Add work plan for #<N>

<one-line summary of the approach>"
```

### 7. Create or update a draft PR

Push the branch and create (or update) a draft PR with a `[PLAN]` title
prefix and the plan as the body. The prefix prevents agents from confusing
the PR number with the issue number.

```bash
# Push current branch (name may vary: feature/issue-<N> or feature/ISSUE-<N>-<desc>)
git push -u origin HEAD

ISSUE_TITLE=$(gh issue view <N> --json title --jq '.title')
CURRENT_BRANCH=$(git branch --show-current)

# Check for existing PR on this branch
EXISTING_PR=$(gh pr list --head "$CURRENT_BRANCH" --json url --jq '.[0].url' 2>/dev/null || echo "")

if [ -n "$EXISTING_PR" ]; then
    # Update existing PR title and body
    gh pr edit "$EXISTING_PR" --title "[PLAN] $ISSUE_TITLE" --body-file .agent/work-plans/PLAN_ISSUE-<N>.md
else
    # Create new draft PR
    gh pr create --draft --title "[PLAN] $ISSUE_TITLE" --body-file .agent/work-plans/PLAN_ISSUE-<N>.md
fi
```

### 8. Report to user

Summarize:
- What the plan proposes
- Which principles and ADRs were considered
- Any open questions that need input
- Link to the draft PR

## Guidelines

- **Generate, don't just evaluate** — this skill produces a plan, not a
  review of an existing plan. For reviewing plans, use `review-pr` on the
  draft PR.
- **Concrete, not generic** — "Update tests" is not a plan step. "Add test
  for edge case X in `test_foo.py`" is.
- **Flag conflicts early** — if the plan would violate a principle or ADR,
  say so and propose an alternative.
- **Include open questions** — if the approach depends on a choice the user
  should make, list it. Don't guess.
- **Keep it short** — a plan should be 30-80 lines. If it's longer, the
  task may need to be broken down.
