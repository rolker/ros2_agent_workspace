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

**This skill evaluates the issue itself** (is it well-scoped? in the right
repo? conflicting with ADRs?). For evaluating an implementation plan, use
`plan-task`. For evaluating a completed PR, use `review-pr`.

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
- `.agent/project_knowledge/` if available

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

## Guidelines

- **Comment, don't edit** — the issue body is the authoritative spec. Triage
  findings are advisory input posted as a comment with their own timestamp.
- **Be actionable** — "Watch: might be too large for a single PR" is useful.
  "Looks fine" without specifics is not.
- **Don't block unnecessarily** — most issues are fine. Flag genuine concerns,
  not hypothetical risks.
- **Note gaps, don't fill them** — if the issue is missing detail, note what's
  missing. Don't rewrite the issue.
