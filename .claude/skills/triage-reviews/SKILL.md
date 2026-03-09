---
name: triage-reviews
description: Evaluate PR review comments (human and bot) against local code, principles, and ADRs. Includes CI check status. Classifies each as valid or false positive and presents a fix plan.
---

# Triage Reviews

## Usage

```
/triage-reviews <pr-number>
```

## Overview

**Lifecycle position**: implement → push → review → **triage-reviews** → fix

Evaluate all PR review comments — from human reviewers, Copilot, and other
bots — against the local worktree code, workspace principles, and ADRs.
Classifies each comment as a valid issue or false positive and presents a
structured plan. Does not auto-fix or post comments.

## Steps

### 1. Confirm worktree (auto-enter if needed)

Verify the current worktree branch matches the PR's head branch:

```bash
# Get PR head branch
PR_BRANCH=$(gh pr view <N> --json headRefName --jq '.headRefName')

# Compare with current branch
CURRENT_BRANCH=$(git branch --show-current)
```

If they don't match:

1. Extract the issue number from the PR branch name (pattern: `feature/issue-<N>`).
2. Auto-enter the worktree:
   ```bash
   source .agent/scripts/worktree_enter.sh --issue <N>
   ```
3. After entering, verify the branch now matches. If it still doesn't (worktree
   doesn't exist or branch mismatch), stop and inform the user with instructions
   to create the worktree:
   ```
   Worktree for issue #<N> not found. Create it with:
     .agent/scripts/worktree_create.sh --issue <N> --type workspace
     source .agent/scripts/worktree_enter.sh --issue <N>
   ```

### 2. Sync local branch

```bash
git pull --ff-only
```

Ensure the local worktree matches the remote HEAD so line numbers in review
comments align with local files.

### 3. Fetch PR reviews and CI status

Run the helper script to get all reviews and CI check status:

```bash
.agent/scripts/fetch_pr_reviews.sh --pr <N>
```

The script:
- Fetches all reviews on the PR via `gh api` (no timestamp filter)
- Includes `commit_id` on each review for timeline reasoning
- Includes `user_login` and `user_type` for each review and comment
- Fetches PR conversation comments (issue-level comments, not code review threads)
- Fetches CI check-runs for the PR head SHA
- Outputs structured JSON with `head_sha`, `reviews`, `conversation_comments`, and `ci_checks`

If the result contains no reviews and no conversation comments, report
"No reviews or comments on this PR" and stop.

### 4. Load governance context

Read the evaluation criteria (only if comments exist):

- `.agent/knowledge/principles_review_guide.md` — principle quick reference,
  ADR applicability, and consequences map
- `docs/PRINCIPLES.md` — workspace principles
- `docs/decisions/*.md` — ADRs (scan titles, read those relevant to the flagged issues)

For project repo PRs, also check:
- The project repo's `.agents/README.md`

### 5. Evaluate each comment

For each comment in the JSON output:

a. **Read the local file** at the referenced path and line using the Read tool
b. **Check review freshness** — each review carries a `commit_id` (the commit
   it was submitted against). Compare it to `head_sha` from the script output:
   - If `commit_id` matches `head_sha`, the review is against current code.
   - If `commit_id` differs, the code has changed since the review. Read the
     file at the referenced path and line. If the concern appears addressed,
     classify as "Likely addressed — verify." If not, classify as valid.
   - For force-pushed branches where `commit_id` is unreachable, read current
     code and note the uncertainty. No shell commands needed — just read and
     assess.
c. **Identify the source** — check `user_type` and `user_login`:
   - **Human reviewers** (`user_type: "User"`): these carry highest authority.
     Check whether the current code already addresses the concern raised
     (e.g., the requested change is present or the issue no longer exists
     at the referenced location). If so, note it as "addressed". If not,
     treat as a valid issue.
   - **Copilot / bot reviewers** (`user_type: "Bot"`): evaluate as potential
     issues or false positives. Bots may compare against stale `main` or
     misunderstand intent.
d. **Assess the comment** against the actual code:
   - Is the concern valid? Does the code actually have the issue flagged?
   - Is it a false positive? (e.g., comparing against stale `main`, or
     misunderstanding the intent)
e. **Evaluate conversation comments** — `conversation_comments` are PR-level
   comments (not attached to specific files or lines). Treat them as general PR
   feedback:
   - **Human conversation comments** carry high authority — treat as actionable
     feedback even though they lack file/line references.
   - **Bot conversation comments** (CI bots, etc.) — evaluate for relevance.
   - Look for requested changes, questions, or concerns that apply to the PR
     as a whole.
f. **Check governance context** — does the comment align with or contradict:
   - Workspace principles (`docs/PRINCIPLES.md`)
   - Relevant ADRs (`docs/decisions/`)
   - Project-level governance (`.agents/README.md` in the project repo, if applicable)

### 6. Classify and present plan

Output a structured report:

```markdown
## PR Review Triage: PR #<N> — <title>

**PR**: <url>
**Head**: `<branch>` at `<short-sha>`
**Reviews**: <total> review(s), <total> inline comment(s), <total> conversation comment(s)

### Human Reviewer Comments

| # | Reviewer | File | Line | Comment | Status |
|---|----------|------|------|---------|--------|
| 1 | `user` | `path/to/file` | 42 | Summary of comment | Valid / Addressed / Likely addressed — verify / Needs discussion |

### Conversation Comments

| # | Author | Type | Comment | Status |
|---|--------|------|---------|--------|
| 1 | `user` | User | Summary of comment | Valid / Addressed / Needs discussion |

### Valid Issues (Bot)

| # | Source | File | Line | Issue | Suggested Fix |
|---|--------|------|------|-------|---------------|
| 1 | Copilot | `path/to/file` | 42 | Description of the valid issue | Brief fix description |

### False Positives (Bot)

| # | Source | File | Line | Comment | Reasoning |
|---|--------|------|------|---------|-----------|
| 1 | Copilot | `path/to/file` | 10 | What the bot said | Why it's not applicable |

### Recommended Actions

- [ ] Fix: <specific action for each valid issue>
- [ ] Address: <specific action for each unaddressed human comment>
- [ ] (Optional) Dismiss false positive reviews on the PR

### CI Status

| Check | Result | Link |
|-------|--------|------|
| <name> | <conclusion> | [link](<html_url>) |

### Summary

<1-3 sentence overall assessment>
```

## Guidelines

- **Triage, don't fix** — output the classified plan in the conversation. The user
  or agent decides what to fix and in what order.
- **Human comments take priority** — list human reviewer comments first. They carry
  higher authority than bot suggestions.
- **Context for all comments** — older comments may have been addressed by subsequent
  commits. Check the code at the referenced location to see if the concern still
  applies. Note "addressed" for comments whose concerns have been resolved.
- **Read the actual code** — don't classify based on the comment text alone. Read
  the local file at the referenced path and line to verify.
- **Be specific about fixes** — "Add null check before accessing `result.data`" is
  useful. "Fix the issue" is not.
- **Context matters** — bot reviewers compare against `main`, so they may flag
  intentional changes as issues. Check whether the flagged code is the intended
  new behavior.
- **Group related comments** — if multiple comments point to the same underlying
  issue, group them in the valid issues table.
- **Governance alignment** — note when a comment aligns with or contradicts
  workspace principles or ADRs.
- **No comments posted** — this skill is read-only. It does not post review comments,
  dismiss reviews, or modify the PR in any way.
