---
name: plan-task
description: Generate a principles-aware work plan for an issue. Saves to `.agent/work-plans/` and commits as the first step on the feature branch.
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

**Lifecycle position**: `review-issue → **plan-task** → implement → review-pr`

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

### 4. Generate the plan

Write a plan to `.agent/work-plans/PLAN_ISSUE-<N>.md`:

```markdown
# Plan: <issue-title>

## Issue

<link to issue>

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

### 5. Commit the plan

```bash
git add .agent/work-plans/PLAN_ISSUE-<N>.md
git commit -m "Add work plan for #<N>

<one-line summary of the approach>"
```

### 6. Create a draft PR

Push the branch and create a draft PR with the plan as the body. This
allows Copilot and humans to review the plan before implementation.

```bash
git push -u origin feature/issue-<N>
gh pr create --draft --title "<issue-title>" --body-file .agent/work-plans/PLAN_ISSUE-<N>.md
```

### 7. Report to user

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
