---
name: review-issue
description: Evaluate a GitHub issue against workspace principles and ADRs before work begins. Posts findings as a comment on the issue.
---

# Review Issue

## Usage

```
/review-issue <issue-number>
```

## Overview

Evaluate an issue before work begins. Checks scope, principle alignment, ADR
relevance, repo placement, and dependencies. Posts findings as a comment on
the issue — does not modify the issue body.

**Lifecycle position**: **review-issue** → plan-task → review-plan → implement → review-code

**This skill evaluates the issue itself** (is it well-scoped? in the right
repo? conflicting with ADRs?). For evaluating an implementation plan, use
`plan-task`. For evaluating a completed PR, use `review-code`.

## Steps

### 1. Read the issue

```bash
gh issue view <N> --json title,body,labels,assignees,milestone,comments,url
```

Identify:
- What is being proposed?
- Which repo does this affect? (workspace infra, or a project repo?)
- Is there a linked PR or existing work?

### 2. Load governance context

Read the evaluation criteria:

- `.agent/knowledge/principles_review_guide.md` — principle quick reference,
  ADR applicability, and consequences map

Read the governance docs:

- `docs/PRINCIPLES.md` — workspace principles
- `docs/decisions/*.md` — ADR titles (read full text only for triggered ADRs)

If the issue targets a project repo, also check:
- The project repo's `PRINCIPLES.md`
- `.agent/project_knowledge/` if available (symlink to manifest repo's `.agents/workspace-context/`)

### 3. Assess scope

- **Well-scoped?** Can this be completed in a single PR? If not, should it
  be broken into sub-issues?
- **Right repo?** Does this change belong in the workspace repo or a project
  repo? Check the workspace vs. project separation principle.
- **Dependencies?** Does this depend on other open issues? Will other issues
  need to wait for this one?

### 4. Evaluate principle alignment

For each relevant principle, assess whether the proposed change aligns:

| Principle | Status | Notes |
|---|---|---|
| ... | OK / Watch / Action needed | Specific observation |

**Statuses**:
- **OK** — No concerns
- **Watch** — Worth noting but not blocking
- **Action needed** — Should be addressed before or during implementation

Skip principles that clearly don't apply.

### 5. Check ADR applicability

Using the ADR applicability table from the review guide, identify which ADRs
are triggered by this type of change. For each triggered ADR:

- Does the issue description account for the ADR's requirements?
- If not, note what may be missing.

### 6. Check consequences

Using the consequences map: if this issue changes something in the "If you
change..." column, note the corresponding "Also update..." items. These
should be part of the issue scope or flagged as follow-up work.

### 7. Post findings as a comment

Post the review as a comment on the issue. **Do not modify the issue body.**

```markdown
## Review

### Scope Assessment

**Well-scoped?** Yes/No — [explanation]
**Right repo?** Yes/No — [explanation]
**Dependencies**: [list or "none identified"]

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| ... | ... | ... |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ... | ... | ... |

### Consequences

- [items that should be updated as part of this work]

### Recommendations

- [specific suggestions, if any]

---
**Authored-By**: `$AGENT_NAME`
**Model**: `$AGENT_MODEL`
```

Use `--body-file` for the comment (not `--body`):

```bash
BODY_FILE=$(mktemp /tmp/gh_body.XXXXXX.md)
cat << 'COMMENT_EOF' > "$BODY_FILE"
[review content]
COMMENT_EOF
gh issue comment <N> --body-file "$BODY_FILE"
rm "$BODY_FILE"
```

### 8. Persist to progress.md

After posting the comment, append a `## Issue Review` entry to
`.agent/work-plans/issue-<N>/progress.md` so the timeline reflects
that the issue has been governance-reviewed before plan-task starts.
Per [ADR-0013](../../docs/decisions/0013-progress-md-entry-type-vocabulary.md).

**`review-issue` is typically the *first* skill in the lifecycle, so a
worktree may not exist yet.** Since progress.md commits cannot land on
a protected default branch, this step creates a worktree on demand:

1. Check `$WORKTREE_ISSUE`. If it matches `<N>`, you're already in the
   right worktree — skip to step 8b.
2. If not, check whether a worktree for the issue already exists:
   ```bash
   .agent/scripts/worktree_list.sh | grep -E "issue-(workspace-)?<N>"
   ```
3. If a worktree exists, source `worktree_enter.sh --issue <N>` to
   enter it (or `cd` to the path directly when sourcing isn't
   possible).
4. If neither: determine worktree type from the issue's owning repo
   (workspace for issues in the workspace repo, layer for project-repo
   issues — same logic as `plan-task` step 4). Create the worktree:
   ```bash
   # Workspace issue
   .agent/scripts/worktree_create.sh --issue <N> --type workspace

   # Project repo issue
   .agent/scripts/worktree_create.sh --issue <N> --type layer \
       --layer <layer> --packages <project_repo>
   ```
   `worktree_create.sh` does NOT open a draft PR when invoked without
   `--plan-file`; only the worktree + branch are created. A draft PR
   will follow when `plan-task` runs.

**8b. Append the entry.** Use frontmatter for a new file:

```yaml
---
issue: <N>
---

# Issue #<N> — <issue title>
```

Append:

```markdown

## Issue Review
**Status**: complete
**When**: <YYYY-MM-DD HH:MM>
**By**: <agent name> (<model>)

**Comment**: <URL of the posted issue comment from step 7>
**Scope verdict**: <well-scoped | needs-splitting | needs-more-detail>

### Action items
- [ ] <each "Action needed" principle finding>
- [ ] <each "Recommendation" worth following up on>
```

If the scope verdict is `well-scoped` and there were no Action-needed
principle findings, set `### Action items` to a single line:
`No action items — issue is plan-task-ready.`

Commit:

```bash
git -C <worktree-path> add .agent/work-plans/issue-<N>/progress.md
git -C <worktree-path> commit -m "progress: issue review for #<N>"
```

The branch (`feature/issue-<N>`) now exists with one commit. If the
user decides not to proceed (and so plan-task never runs), they can
remove the worktree + branch via `worktree_remove.sh --issue <N>`.

## Guidelines

- **Comment, don't edit** — the issue body is the authoritative spec. Review
  findings are advisory input posted as a comment with their own timestamp.
- **Be actionable** — "Watch: might be too large for a single PR" is useful.
  "Looks fine" without specifics is not.
- **Don't block unnecessarily** — most issues are fine. Flag genuine concerns,
  not hypothetical risks.
- **Note gaps, don't fill them** — if the issue is missing detail, note what's
  missing. Don't rewrite the issue.
