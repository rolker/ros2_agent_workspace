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

**Lifecycle position**: review-issue → **plan-task** → review-plan → implement → review-code

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
EXISTING_PR=$(gh pr list --head "$CURRENT_BRANCH" --json url --jq '.[0].url // ""' 2>/dev/null || echo "")

# Build PR body: prepend Closes reference, then plan content
BODY_FILE=$(mktemp /tmp/gh_body.XXXXXX.md)
printf 'Closes #<N>\n\n' > "$BODY_FILE"
cat .agent/work-plans/PLAN_ISSUE-<N>.md >> "$BODY_FILE"

if [ -n "$EXISTING_PR" ]; then
    # Update existing PR title and body
    gh pr edit "$EXISTING_PR" --title "[PLAN] $ISSUE_TITLE" --body-file "$BODY_FILE"
else
    # Create new draft PR
    gh pr create --draft --title "[PLAN] $ISSUE_TITLE" --body-file "$BODY_FILE"
fi
rm -f "$BODY_FILE"
```

### During implementation

The `plan-task` skill produces its artifact at step 7 (the draft PR).
Implementation then proceeds on the same branch — and typically deviates
from the plan: types get refined, functions change signatures, a "pure
logic" module picks up a dependency. Keep the plan in sync as this
happens so the file stays usable reference material — for Copilot's
plan-file review, for `review-code`, and for the next agent who picks
up the package.

1. **Inline edits are the default.** When implementation diverges from the
   plan in any way — wording, method names, file paths, dependency
   choices, a test contract that tightened during coding — edit the plan
   inline to match the landed code. Git history preserves the original
   planning state; nobody benefits from stale text at the top of an
   otherwise-current plan.

2. **Appended "Implementation Notes" only for rationale-bearing design
   pivots.** If the deviation is a real design decision whose *why* is
   not obvious from the code diff alone (e.g. "switched from `QGridLayout`
   to a custom layout because `QGridLayout` can't express slack-to-outside
   with shared inner edges"), make the inline edit *and* add a one-liner
   to an `## Implementation Notes` section at the bottom of the plan
   capturing the rationale. The section is for rationale only, not a
   changelog.

3. **Never append-only.** Leaving stale text at the top of the plan while
   listing "changes" in a section below misleads Copilot, human reviewers,
   and future onboarding agents — all of whom read the top first.

4. **Commit discipline.** Plan edits flow in commits alongside the code
   changes that triggered them (same commit when caught while coding;
   follow-up commit if caught later during review triage). Never amend a
   pushed commit just to update the plan.

This guidance is enforced at the instructions layer only. There is no
local hook for "did you update the plan"; the enforcement signal is
review feedback — Copilot's plan-drift flags and `review-code` catching
misalignment between the plan and the PR. If drift becomes a recurring
finding across multiple PRs, that's a signal the rule needs more teeth,
not that the rule is wrong.

#### Worked example

From `rolker/rqt_operator_tools@9680d3e` (PR #22 round-8 fix pass —
the code change and its plan edit landed in the same commit):

**Before** — `.agent/work-plans/PLAN_ISSUE-20.md:108`:

```
│   │   ├── staleness_tracker.hpp                 # pure logic, no Qt/ROS
```

**After** — same file, same commit as the `rclcpp::Time` adoption in
`staleness_tracker.cpp`:

```
│   │   ├── staleness_tracker.hpp                 # pure logic, uses rclcpp::Time (no Qt)
```

The edit is inline (rule 1) and co-located with the code change that
motivated it (rule 4). No `Implementation Notes` entry is needed — the
*why* (consistent clock math under `use_sim_time`) is visible in the
`staleness_tracker.cpp` diff in the same commit (rule 2). An
`Implementation Notes` entry would have been warranted for a case like
the `QGridLayout` swap cited above, where the rationale doesn't fit in a
one-line plan edit.

### 8. Report to user

Summarize:
- What the plan proposes
- Which principles and ADRs were considered
- Any open questions that need input
- Link to the draft PR
- Pointer to the "During implementation" subsection above, so the
  implementer knows to keep the plan in sync with landed code

## Guidelines

- **Generate, don't just evaluate** — this skill produces a plan, not a
  review of an existing plan. For reviewing plans, use `review-plan` on the
  draft PR.
- **Concrete, not generic** — "Update tests" is not a plan step. "Add test
  for edge case X in `test_foo.py`" is.
- **Flag conflicts early** — if the plan would violate a principle or ADR,
  say so and propose an alternative.
- **Include open questions** — if the approach depends on a choice the user
  should make, list it. Don't guess.
- **Keep it short** — a plan should be 30-80 lines. If it's longer, the
  task may need to be broken down.
