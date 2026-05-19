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
Per [ADR-0013](../../../docs/decisions/0013-progress-md-entry-type-vocabulary.md).

**`review-issue` is typically the *first* skill in the lifecycle, so a
worktree may not exist yet.** Since progress.md commits cannot land on
a protected default branch, this step creates a worktree on demand:

1. **Resolve the owning repo first.** Issue numbers can collide across
   the workspace repo and project repos (e.g., workspace `#42` is a
   different issue from `unh_echoboats_project11#42`), so the issue
   number alone is not enough to identify a worktree. Determine the
   owning repo via `gh issue view <N> --repo <owner/repo> --json
   repository --jq '.repository.nameWithOwner'`, mirroring `plan-task`
   step 4's resolution.
2. Check `$WORKTREE_ISSUE` **and** `$WORKTREE_REPO`. If `$WORKTREE_ISSUE`
   matches `<N>` *and* `$WORKTREE_REPO` matches the owning repo's short
   slug, you're already in the right worktree — skip to step 8b. If
   only the number matches, treat it as a miss and continue to step 3
   (you're in the wrong-repo worktree for a colliding number).
3. If not, check whether a worktree for the issue already exists. Use
   an anchored, repo-aware pattern so neighbouring issue numbers
   (e.g. `4700` for `<N>=470`) and unrelated repo worktrees don't
   match:
   ```bash
   # Workspace worktree (path ends in `/issue-workspace-<N>` exactly)
   ls -d "/path/to/workspace/.workspace-worktrees/issue-workspace-<N>" 2>/dev/null
   # Layer worktree for a specific project repo
   ls -d "/path/to/workspace/layers/worktrees/issue-<repo-slug>-<N>" 2>/dev/null
   ```
   Or grep `worktree_list.sh` output with a numeric boundary:
   ```bash
   .agent/scripts/worktree_list.sh \
     | grep -E "issue-(workspace|<repo-slug>)-<N>($|[^0-9])"
   ```
4. If a worktree exists, source `worktree_enter.sh --issue <N>` to
   enter it (or `cd` to the path directly when sourcing isn't
   possible). When the issue number collides across repos, pass
   `--repo-slug <slug>` to disambiguate (see `WORKTREE_GUIDE.md`).
5. If neither: determine worktree type from the owning repo (workspace
   for issues in the workspace repo, layer for project-repo issues —
   same logic as `plan-task` step 4). Create the worktree:
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

### Actions
- [ ] <each "Action needed" principle finding>
- [ ] <each "Recommendation" worth following up on>
```

If the scope verdict is `well-scoped` and there were no Action-needed
principle findings *and* no Recommendations, set `### Actions` to a
single line: `No actions — issue is plan-task-ready.` If there are
Recommendations but no Action-needed findings, list the Recommendations
under `### Actions` so the timeline preserves them — they're follow-up
candidates downstream consumers should be able to see.

Commit:

```bash
git -C <worktree-path> add .agent/work-plans/issue-<N>/progress.md
git -C <worktree-path> \
    -c user.name="$AGENT_NAME" \
    -c user.email="$AGENT_EMAIL" \
    commit -m "progress: issue review for #<N>"
```

The per-invocation `-c` overrides are required by
[AGENTS.md § Agent Commit Identity](../../../AGENTS.md#agent-commit-identity)
because the skill instructions run in fresh subshells where the
`set_git_identity_env.sh` exports may not be in scope; without them
`check_pr_authors.py` (CI mechanism C, [#468](https://github.com/rolker/ros2_agent_workspace/issues/468))
will reject the commit.

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
