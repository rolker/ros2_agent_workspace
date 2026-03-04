---
name: triage-copilot
description: Evaluate Copilot review comments on a PR against local code, principles, and ADRs. Classifies each as valid or false positive and presents a fix plan.
---

# Triage Copilot

## Usage

```
/triage-copilot <pr-number>
```

## Overview

**Lifecycle position**: implement → push → Copilot review → **triage-copilot** → fix

Evaluate GitHub Copilot review comments on a PR against the local worktree code,
workspace principles, and ADRs. Classifies each comment as a valid issue or false
positive and presents a structured plan. Does not auto-fix or post comments.

## Steps

### 1. Confirm worktree

Verify the current worktree branch matches the PR's head branch:

```bash
# Get PR head branch
gh pr view <N> --json headRefName --jq '.headRefName'

# Compare with current branch
git branch --show-current
```

If they don't match, stop and inform the user.

### 2. Sync local branch

```bash
git pull --ff-only
```

Ensure the local worktree matches the remote HEAD so line numbers in Copilot
comments align with local files.

### 3. Fetch Copilot review comments

Run the helper script to get comments submitted after the most recent commit:

```bash
.agent/scripts/fetch_copilot_reviews.sh --pr <N>
```

The script:
- Gets the HEAD commit's committer date as the cutoff timestamp
- Fetches all Copilot reviews on the PR via `gh api`
- Filters to reviews submitted after the cutoff
- Fetches comments for matching reviews
- Outputs structured JSON with paths, line numbers, and comment bodies

If the result has zero comments, report "No new Copilot comments since last commit"
and stop.

### 4. Evaluate each comment

For each comment in the JSON output:

a. **Read the local file** at the referenced path and line using the Read tool
b. **Assess the comment** against the actual code:
   - Is the concern valid? Does the code actually have the issue Copilot flagged?
   - Is it a false positive? (e.g., Copilot comparing against stale `main`, or
     misunderstanding the intent)
c. **Check governance context** — does the comment align with or contradict:
   - Workspace principles (`docs/PRINCIPLES.md`)
   - Relevant ADRs (`docs/decisions/`)
   - Project-level governance (`.agents/README.md` in the project repo, if applicable)

### 5. Load governance context

Read the evaluation criteria (only if comments exist):

- `.agent/knowledge/principles_review_guide.md` — principle quick reference,
  ADR applicability, and consequences map
- `docs/PRINCIPLES.md` — workspace principles
- `docs/decisions/*.md` — ADRs (scan titles, read those relevant to the flagged issues)

For project repo PRs, also check:
- The project repo's `.agents/README.md`

### 6. Classify and present plan

Output a structured report:

```markdown
## Copilot Triage: PR #<N> — <title>

**PR**: <url>
**Head**: `<branch>` at `<short-sha>`
**Cutoff**: <HEAD committer date>
**Reviews found**: <count> review(s), <count> comment(s)

### Valid Issues

| # | File | Line | Issue | Suggested Fix |
|---|------|------|-------|---------------|
| 1 | `path/to/file` | 42 | Description of the valid issue | Brief fix description |

### False Positives

| # | File | Line | Comment | Reasoning |
|---|------|------|---------|-----------|
| 1 | `path/to/file` | 10 | What Copilot said | Why it's not applicable |

### Recommended Actions

- [ ] Fix: <specific action for each valid issue>
- [ ] (Optional) Dismiss false positive reviews on the PR

### Summary

<1-3 sentence overall assessment: how many valid vs false positive, severity>
```

## Guidelines

- **Triage, don't fix** — output the classified plan in the conversation. The user
  or agent decides what to fix and in what order.
- **Read the actual code** — don't classify based on the comment text alone. Read
  the local file at the referenced path and line to verify.
- **Be specific about fixes** — "Add null check before accessing `result.data`" is
  useful. "Fix the issue" is not.
- **Context matters** — Copilot reviews against `main`, so it may flag intentional
  changes as issues. Check whether the flagged code is the intended new behavior.
- **Group related comments** — if multiple comments point to the same underlying
  issue, group them in the valid issues table.
- **Governance alignment** — note when a Copilot comment aligns with or contradicts
  workspace principles or ADRs. This adds value beyond what Copilot provides alone.
- **No comments posted** — this skill is read-only. It does not post review comments,
  dismiss reviews, or modify the PR in any way.
