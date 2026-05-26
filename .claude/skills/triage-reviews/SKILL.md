---
name: triage-reviews
description: Integrator — evaluate PR review comments (human and bot) together with the prior progress.md review timeline, against local code, principles, and ADRs. Includes CI check status. Classifies each finding as valid or false positive, flags cross-source confirmations, presents a fix plan, and persists a unified Integrated Review entry to progress.md.
---

# Triage Reviews

## Usage

```
/triage-reviews <pr-number>
```

## Overview

**Lifecycle position**: implement → push → review → **triage-reviews** → fix

Evaluate all PR review comments — from human reviewers, Copilot, and other
bots — **together with the prior `progress.md` review timeline** (e.g.
`## Local Review`, `## Local Review (Pre-Push)`), against the local worktree
code, workspace principles, and ADRs. As the **integrator**
([#470](https://github.com/rolker/ros2_agent_workspace/issues/470) phase B),
this skill merges those sources, flags findings raised by more than one as
**cross-source confirmations** (the strongest signal), classifies each as a
valid issue or false positive, presents a structured plan, and persists a
unified `## Integrated Review` entry. Does not auto-fix or post comments.

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

1. Extract the issue number from the PR branch name (patterns:
   `feature/issue-<N>` or `feature/ISSUE-<N>-<description>`).
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

If the result contains no reviews and no conversation comments, **do not stop** —
the local timeline (next paragraph) may still carry findings to integrate. Only
report "No reviews, comments, or prior entries" and stop if *both* the GitHub
side and the local timeline are empty.

**Also read the prior local timeline** (integrator step). The GitHub-side
reviews are one source; the other is the issue's own `progress.md`. Here
`<issue-N>` is the **issue** number (resolved from the PR head branch, as in
step 7) — *not* the PR number the skill was invoked with. Extract it (invoke via
`python3`, matching the repo's `.agent/scripts/*.py` convention — don't rely on
the executable bit):

```bash
# progress.md may not exist yet (a legacy PR with no lifecycle timeline);
# progress_read.py exits non-zero on a missing file, so guard with -f and treat
# an absent timeline as empty — the GitHub-side reviews are then the only source.
PROGRESS=.agent/work-plans/issue-<issue-N>/progress.md
[ -f "$PROGRESS" ] && python3 .agent/scripts/progress_read.py "$PROGRESS" \
    --type "Local Review" --type "Local Review (Pre-Push)" --type "Integrated Review"
```

`progress_read.py` emits JSON with one record per entry, each carrying its entry
type, correlation key (issue# / plan-commit SHA / PR-or-branch head SHA per
ADR-0013), and findings. Select the review entries relevant to this PR:
`## Local Review`, `## Local Review (Pre-Push)`, prior `## Integrated Review`
entries (earlier rounds on this same PR — read them so multi-round triage builds
on, rather than repeats, prior findings), and any historical `## External
Review` (the recognized predecessor; `--type "Integrated Review"` matches those
predecessors too). Match them to the GitHub-side reviews by **head SHA** (the
correlation key for review entries): entries at the current `head_sha` describe
the same code as the live
reviews; older entries are prior rounds.

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
   - **Plan files** (`issue-*/plan.md` or legacy `PLAN_ISSUE-*.md`): If
     the comment targets a file in `.agent/work-plans/`, it is a
     planning artifact. Check whether the concern is addressed in the
     implementation files changed in the same PR. If so, classify as
     "Addressed" and note that the concern is satisfied by the
     implementation. Plan wording does not need to be updated to match
     implementation.
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
g. **Detect cross-source confirmations** (integrator step) — a finding raised by
   two or more sources at the **same entry-type correlation key** is a
   cross-source confirmation: the strongest signal. Per ADR-0013's "consume by
   entry-type filter," the key is that entry type's correlation key — for review
   entries, the PR/branch **head SHA**. Example: a finding in a `## Local Review`
   at head `<sha>` that a Copilot review at the same head also raises. **Keep
   both; do not collapse** — record the finding once with its list of sources.
   Cross-source confirmations are the highest-priority class in the report.

### 6. Classify and present plan

Output a structured report:

```markdown
## PR Review Triage: PR #<N> — <title>

**PR**: <url>
**Head**: `<branch>` at `<short-sha>`
**Sources**: <count> (e.g., Copilot R2, Local Review @ `<sha>`, CI rollup)
**Reviews**: <total> review(s), <total> inline comment(s), <total> conversation comment(s)
**Cross-source confirmations**: <count>

### Cross-Source Confirmations

Findings raised by two or more sources at the same head SHA — highest priority.

| # | Sources | File | Line | Finding |
|---|---------|------|------|---------|
| 1 | Copilot R2 + Local Review @ `<sha>` | `path/to/file` | 42 | Description |

### Human Reviewer Comments

| # | Reviewer | File | Line | Comment | Status |
|---|----------|------|------|---------|--------|
| 1 | `user` | `path/to/file` | 42 | Summary of comment | Valid / Addressed / Likely addressed — verify / Needs discussion |

### Conversation Comments

| # | Author | Type | Comment | Status |
|---|--------|------|---------|--------|
| 1 | `user` | User | Summary of comment | Valid / Addressed / Needs discussion |

### Valid Issues (Bot)

| # | Sources | File | Line | Issue | Suggested Fix |
|---|---------|------|------|-------|---------------|
| 1 | Copilot | `path/to/file` | 42 | Description of the valid issue | Brief fix description |

### False Positives (Bot)

| # | Source | File | Line | Comment | Justification |
|---|--------|------|------|---------|---------------|
| 1 | Copilot | `path/to/file` | 10 | What the bot said | Specific reason the failure mode cannot occur |

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

### 7. Persist triage to progress.md

Resolve the linked issue number from the PR branch name (same extraction
as step 1: `feature/issue-<N>` or `feature/ISSUE-<N>-<description>`).
This intentionally couples progress.md placement to the worktree's
plan.md location — both live under `.agent/work-plans/issue-<N>/` in
the worktree the branch belongs to.

The reviewer's `closingIssuesReferences` is **not** used here, even
though `review-code`'s post-PR mode and `review-plan`'s PR-number form
both prefer it. Rationale: those skills resolve an issue to **look up**
information (the issue body, the plan file); `triage-reviews` resolves
an issue to **co-locate** the timeline entry with plan.md. Branch-name
is the right signal because it identifies the worktree, not the
GitHub-closing semantics — and it works offline / pre-PR, consistent
with the rest of the local-first skill set.

**Fail loud, don't fall back**: if the branch name doesn't match the
`feature/issue-<N>` / `feature/ISSUE-<N>-<desc>` pattern, stop with an
explicit error rather than silently picking a default or attempting a
GitHub-side lookup. Example message:

```
Cannot resolve issue number from branch name '<branch>'. Expected
'feature/issue-<N>' or 'feature/ISSUE-<N>-<description>'. Rename the
branch to follow the convention, or skip the persistence step manually
if the worktree intentionally has no associated issue.
```

Once resolved, determine which repo owns the linked issue and check
`.agent/work-plans/issue-<N>/progress.md` in the owning repo's worktree
first, falling back to the current worktree. If `progress.md` doesn't
exist in either location, create it in the owning repo's worktree (or
the current worktree if no owning worktree exists). Fetch the issue
title from the correct repo via:

```bash
gh issue view <N> --repo <owner/repo> --json title --jq '.title'
```

For new files, create the parent directory first:

```bash
mkdir -p .agent/work-plans/issue-<N>
```

Frontmatter for new files:

```yaml
---
issue: <N>
---

# Issue #<N> — <issue title>
```

Append this step entry:

```markdown

## Integrated Review
**Status**: complete
**When**: <YYYY-MM-DD HH:MM ±HH:MM>
**By**: <agent name> (<model>)

**PR**: #<N> at `<short-sha>`   <!-- or **Branch**: <name> at `<sha>` if no PR yet -->
**Sources**: <count> (e.g., Copilot R2 @ `<sha>`, Local Review @ `<sha>`, CI rollup)
**Cross-source confirmations**: <count>
**CI**: <all-pass | failures-noted>

### Findings
- [ ] (cross-confirmed) <finding raised by 2+ sources> — `<file>`
- [ ] (<severity>, <source>) <single-source finding> — `<file>`

### False positives
- (<source>) <what was claimed> — <specific reason the failure mode cannot occur>
```

Findings carry their source(s) inline in the leading `(...)`; cross-source
confirmations use `(cross-confirmed)` and are listed first. False positives are
plain bullets (not checkboxes) with a justification — they are dismissals, not
action items. This shape matches the worked example in
`.agent/work-plans/issue-468/progress.md`.

Commit `progress.md` after appending. Run `git add` and `git commit` in
the worktree where progress.md was found or created (which may differ
from the current working directory):

```bash
git -C <worktree-path> add .agent/work-plans/issue-<N>/progress.md
git -C <worktree-path> \
    -c user.name="$AGENT_NAME" \
    -c user.email="$AGENT_EMAIL" \
    commit -m "progress: integrated review for #<N>"
```

The per-invocation `-c` overrides are required by
[AGENTS.md § Agent Commit Identity](../../../AGENTS.md#agent-commit-identity);
agents run each bash invocation in a fresh subshell where
`set_git_identity_env.sh`'s env exports may not be in scope, and a
plain `git commit` then trips `check_pr_authors.py` (CI mechanism C,
[#468](https://github.com/rolker/ros2_agent_workspace/issues/468)).
`## Integrated Review` is the current entry type per
[ADR-0013](../../../docs/decisions/0013-progress-md-entry-type-vocabulary.md)
(this skill is [#470](https://github.com/rolker/ros2_agent_workspace/issues/470)
phase B). The predecessor `## External Review` is still *read* (step 3) for
historical entries but is no longer *written*.

(The fail-loud rule above already covers the case where the branch
name doesn't carry an issue number — the skill stops with an error
rather than silently writing to the wrong path or skipping
persistence.)

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
- **Justify every false positive** — every "false positive" classification must
  include a specific reason the failure mode cannot occur in this system. Blanket
  dismissals are not sufficient:
  - "Config is under our control" — explain what prevents misconfiguration in the field
  - "Pathological input" — explain why that input genuinely cannot reach this code path
  - "Nice-to-have" / "low priority" — not valid justifications; if the concern is
    about error handling, stale data, or silent failures, classify as Valid unless
    you can prove the failure mode is impossible
  - If you cannot articulate why it's safe, classify as Valid and suggest the fix
- **No GitHub review actions** — this skill does not post review
  comments, dismiss reviews, or modify the PR on GitHub. The only
  side-effect on disk is appending to `progress.md` and committing it
  (step 7).
- **Plan-first workflow PRs** — In the plan-first workflow, a PR starts with a
  plan commit and later receives implementation commits. When triaging these PRs:
  - Comments on plan files (`.agent/work-plans/issue-*/plan.md` or
    legacy `PLAN_ISSUE-*.md`) are low priority — the plan is a
    pre-implementation artifact and the implementation is the source of
    truth.
  - Reviews submitted against the plan-only commit (`commit_id` differs from
    `head_sha`) are likely stale once implementation lands. Evaluate the
    reviewer's concern against the current implementation code, not the plan text.
  - For bot comments on plan files, classify as false positives when the
    implementation already addresses the concern, even if the plan wording
    doesn't match.
  - For human comments on plan files, use "Addressed" status in the Human
    Reviewer Comments table when the implementation resolves the concern.
