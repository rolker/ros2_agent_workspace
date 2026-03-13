---
name: review-plan
description: Independent evaluation of a committed work plan before implementation begins. Checks scope, approach, principle alignment, consequences, and ROS conventions.
---

# Review Plan

## Usage

```
/review-plan <pr-number>
```

## Overview

**Lifecycle position**: review-issue → plan-task → **review-plan** → implement → review-code

Independent evaluation of a committed work plan. The planner should not grade
their own work — this skill provides a second opinion before implementation
begins. Runs on draft PRs created by `plan-task` (typically prefixed `[PLAN]`).

## Steps

### 1. Read the plan and PR

```bash
# PR metadata and body (plan is in the PR body)
gh pr view <N> --json title,body,baseRefName,headRefName,files,url

# Get the linked issue
gh pr view <N> --json body --jq '.body' | grep -o '#[0-9]*' | head -1
```

Find the plan file in the PR's changed files — it will be at
`.agent/work-plans/PLAN_ISSUE-*.md`. Read it in full.

### 2. Read the issue and any review-issue comments

```bash
# The linked issue
ISSUE_NUM=<extracted from step 1>
gh issue view "$ISSUE_NUM" --json title,body,labels,comments,url
```

Check for `review-issue` comments on the issue — they contain scope assessment,
principle flags, and ADR notes that the plan should address.

### 3. Load governance context

- `.agent/knowledge/principles_review_guide.md` — evaluation criteria
- `docs/PRINCIPLES.md` — workspace principles
- `docs/decisions/*.md` — ADR titles (read triggered ADRs in full)

For project repo plans, also read:
- Project `PRINCIPLES.md` if it exists
- `.agents/README.md` for architecture context
- `.agents/review-context.yaml` for the compact relevance map (if available)

### 4. Evaluate the plan

Assess each dimension and assign a verdict (**Good** / **Needs work** / **Concern**):

#### Scope

- Is the plan appropriately sized for a single PR?
- If too large (>10 files, >3 major components), should it be split?
- If too vague ("update tests"), does it need specifics ("add test for X in
  `test_foo.py` covering edge case Y")?

#### Issue alignment

- Does the plan address the issue's requirements?
- If `review-issue` was run, does the plan address its findings?
- Are there issue requirements not covered by the plan?

#### File targeting

- Are the right files identified for modification?
- For project repos: cross-reference with `.agents/README.md` or
  `review-context.yaml` — are there related files (dependencies, tests,
  downstream consumers) that should also be listed?
- Are any unnecessary files included? (scope creep)

#### Consequences

- Does the plan's consequences table cover all items from the consequences map?
- For each "If we change X, also update Y" — is Y included in the plan?
- Are there cross-repo consequences not captured?

#### Principle alignment

- Does the plan align with relevant workspace principles?
- Focus on principles most likely to be violated:
  - "A change includes its consequences" — is the plan complete?
  - "Only what's needed" — is the plan minimal?
  - "Enforcement over documentation" — does a new rule have enforcement?
  - "Test what breaks" — are tests planned for risky logic?

#### ADR compliance

- Which ADRs are triggered by this plan's approach?
- Does the plan comply with their key requirements?

#### ROS conventions (for project repo plans)

- Does the approach follow ROS 2 patterns for the type of change?
- Topic naming, QoS choices, parameter handling, lifecycle management?
- Does it reference the right REPs (103, 105, 2004)?

### 5. Produce the report

```markdown
## Plan Review: PR #<N> — <title>

**PR**: <url>
**Issue**: #<issue> — <issue-title>
**Plan file**: `.agent/work-plans/PLAN_ISSUE-<N>.md`

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good / Needs work / Concern | Assessment |
| Issue alignment | ... | ... |
| File targeting | ... | ... |
| Consequences | ... | ... |
| Principle alignment | ... | ... |
| ADR compliance | ... | ... |
| ROS conventions | ... | N/A for workspace plans |

### Findings

<Numbered list of specific findings, if any. Each with:>
1. **[Dimension]** — Description of finding and suggested resolution

### Summary

<1-3 sentence overall assessment. Is the plan ready for implementation?>

### Recommended Actions

- [ ] <specific action items before implementation begins>
```

If no findings, output:

```markdown
## Plan Review: PR #<N> — <title>

**PR**: <url>
Plan looks solid. Ready for implementation.
```

## Guidelines

- **Evaluate, don't rewrite** — flag gaps and concerns. Don't generate an
  alternative plan.
- **Plans are guides, not contracts** — minor deviations during implementation
  are expected. Focus on structural issues: missing files, missing consequences,
  scope problems, principle violations.
- **Be specific** — "Consequence missing: changing `marine_msgs` requires
  updating `mission_manager` subscriber" is useful. "Consider consequences" is
  not.
- **Skip N/A dimensions** — if the plan doesn't touch project repos, skip ROS
  conventions. If no ADRs are triggered, say so briefly.
- **review-issue feedback** — if `review-issue` was run, verify its findings are
  addressed. If they're not, flag it. If `review-issue` was not run, note this
  but don't penalize — it's an optional step.
