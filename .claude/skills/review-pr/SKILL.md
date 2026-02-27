---
name: review-pr
description: Evaluate a PR against workspace principles and ADRs. For project repos, also evaluates against project-level governance. Produces a structured report.
---

# Review PR

## Usage

```
/review-pr <pr-number-or-url>
```

## Overview

**Lifecycle position**: review-issue → plan-task → implement → **review-pr**

Evaluate a pull request against workspace principles, ADRs, and (when applicable)
project-level governance. Produces a structured report in the conversation. Does
not post comments or modify the PR unless the user asks.

## Steps

### 1. Gather PR context

```bash
# Get PR metadata
gh pr view <N> --json title,body,baseRefName,headRefName,files,additions,deletions,url,comments,reviews

# Get the diff
gh pr diff <N>

# Get the linked issue (if any)
gh pr view <N> --json body --jq '.body' | grep -o '#[0-9]*'
```

Identify:
- What repo the PR targets (workspace or project repo?)
- What files changed and in which directories
- The linked issue and its requirements
- Whether a work plan exists (`.agent/work-plans/PLAN_ISSUE-*.md`)

### 2. Load governance context

Read the evaluation criteria:

- `.agent/knowledge/principles_review_guide.md` — principle quick reference,
  ADR applicability, and consequences map

Read the governance docs:

- `docs/PRINCIPLES.md` — workspace principles
- `docs/decisions/*.md` — ADRs (scan titles, read those triggered by this change)

For project repo PRs, also check:
- The project repo's `PRINCIPLES.md` or `docs/PRINCIPLES.md`
- The project repo's `.agents/README.md`
- `.agent/project_knowledge/` if the symlink exists (aggregated knowledge)

### 3. Evaluate against principles

For each principle in the review guide, assess the PR:

| Principle | Verdict | Notes |
|---|---|---|
| Human control and transparency | Pass / Watch / Concern | Specific observations |
| ... | ... | ... |

**Verdicts**:
- **Pass** — PR clearly adheres
- **Watch** — Not a violation, but worth noting
- **Concern** — Potential violation that should be addressed
- **N/A** — Principle doesn't apply to this change

Skip principles that clearly don't apply. Focus on the ones triggered by this
type of change.

### 4. Check ADR compliance

Using the ADR applicability table from the review guide, identify which ADRs
are triggered by this PR's changes. For each triggered ADR:

- Does the PR comply with the ADR's key requirement?
- If not, what needs to change?

### 5. Check consequences

Using the consequences map from the review guide:

- Does this PR change something listed in the "If you change..." column?
- If so, are the corresponding "Also update..." items addressed in the PR?
- List all affected consequence mappings, marking each as Done or Missing.

### 6. Check for existing review comments

```bash
# Copilot and human review comments (derive repo slug dynamically)
REPO_SLUG="$(gh repo view --json nameWithOwner --jq '.nameWithOwner')"
gh api "repos/${REPO_SLUG}/pulls/<N>/comments" --jq '.[] | {user: .user.login, body: .body, path: .path}'
gh api "repos/${REPO_SLUG}/pulls/<N>/reviews" --jq '.[] | {user: .user.login, state: .state, body: .body}'
```

Note:
- Unresolved Copilot comments that appear valid
- Copilot comments that appear to be false positives (e.g., referencing stale `main`)
- Human review comments that haven't been addressed

### 7. Produce the report

Output a structured report:

```markdown
## PR Review: #<N> — <title>

**PR**: <url>
**Issue**: #<issue> — <issue-title>
**Repo**: workspace | <project-repo>
**Files changed**: <count> (+<additions> -<deletions>)

### Principle Evaluation

| Principle | Verdict | Notes |
|---|---|---|
| ... | ... | ... |

### ADR Compliance

| ADR | Triggered | Compliant | Notes |
|---|---|---|---|
| ... | ... | ... | ... |

### Consequence Check

| Changed | Required update | Status |
|---|---|---|
| ... | ... | Done / Missing |

### Existing Review Comments

- <summary of unresolved comments, if any>

### Summary

<1-3 sentence overall assessment>

### Recommended Actions

- [ ] <specific action items, if any>
```

## Guidelines

- **Report, don't act** — output the review in the conversation. The user
  decides whether to post it as a PR comment, request changes, or act on it.
- **Be specific** — "Concern: no tests for the new validation logic" is useful.
  "Watch: could add more tests" is not.
- **Read the diff** — don't just check file names. Read the actual changes to
  evaluate principle adherence.
- **Check the plan** — if a work plan exists, compare the implementation against
  it. Note deviations.
- **Project governance** — for project repo PRs, apply both workspace and project
  governance. Note conflicts between them if any.
