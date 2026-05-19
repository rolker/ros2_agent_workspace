---
name: review-plan
description: Independent evaluation of a committed work plan before implementation begins. Checks scope, approach, principle alignment, consequences, and ROS conventions.
---

# Review Plan

## Usage

```
/review-plan <pr-number>
/review-plan <path-to-plan.md>
/review-plan --issue <N> [--repo <owner/repo>]
```

- `<pr-number>` — read the plan from a draft PR (existing behavior)
- `<path-to-plan.md>` — read the plan directly from a local file
- `--issue <N>` — resolve to `.agent/work-plans/issue-<N>/plan.md` in
  the current repo's worktree

`--repo <owner/repo>` is an optional adjunct that affects only `gh`
lookups (issue title, linked PR metadata) — it does **not** change
where the plan file is read from. To review a plan from a different
repo, enter that repo's worktree first; `--repo` is for the case where
the worktree is correct but `gh` would otherwise default to the wrong
remote.

The file path and `--issue` forms enable **PR-less** plan review —
useful before pushing or when working on a plan that won't get a draft
PR. (Not fully offline: step 2 still calls `gh issue view` to fetch the
linked issue's body and review-issue comments. The plan file itself is
read locally.)

## Overview

**Lifecycle position**: review-issue → plan-task → **review-plan** → implement → review-code

Independent evaluation of a committed work plan. The planner should not
grade their own work — this skill provides a second opinion before
implementation begins. Accepts draft PRs created by `plan-task`
(typically prefixed `[PLAN]`), local file paths, or issue numbers.

## Steps

### 1. Read the plan

Determine the input form and locate the plan file. Detection heuristic:
- Argument starts with `--issue` → issue number form
- Argument contains `/` or ends with `.md` → file path form
- Otherwise → PR number form

#### PR number (e.g., `/review-plan 127`)

```bash
# PR metadata and body (plan is often summarised in the PR body).
# Pass `--repo <owner/repo>` when reviewing a PR from a different repo
# than the current worktree.
gh pr view <N> [--repo <owner/repo>] --json title,body,baseRefName,headRefName,files,url

# Primary closing-linked issue (parsed by GitHub from `Closes #N` /
# `Fixes #N` / `Resolves #N` in the PR body). Mirrors review-code's
# resolution — a body grep for `#[0-9]+` mis-resolves PRs that mention
# multiple issues (e.g., #448 closes #247 and #445 but body-grep would
# return #247).
ISSUE_NUM=$(gh pr view <N> [--repo <owner/repo>] --json closingIssuesReferences --jq '.closingIssuesReferences[0].number // empty')
```

Find the plan file in the PR's changed files. It lives at
`.agent/work-plans/issue-<N>/plan.md` (or, for plans authored before the
directory convention landed, the legacy
`.agent/work-plans/PLAN_ISSUE-<N>.md` file — the symlink at the new path
resolves to it). Read it in full.

#### File path (e.g., `/review-plan .agent/work-plans/issue-45/plan.md`)

Read the plan file directly. Extract the issue number from the path:
- New layout: `issue-<N>` directory name
- Legacy layout: `PLAN_ISSUE-<N>.md` filename

#### Issue number (e.g., `/review-plan --issue 45`)

Resolve to `.agent/work-plans/issue-<N>/plan.md` in the current repo. If
the file doesn't exist, also check the legacy
`.agent/work-plans/PLAN_ISSUE-<N>.md`. If neither is present, check for
an active worktree:

```bash
# Workspace worktree
ls .workspace-worktrees/issue-workspace-<N>/.agent/work-plans/issue-<N>/plan.md 2>/dev/null

# Layer worktree (project repo) — plans live inside the project repo,
# not at the worktree root. The path is
# `layers/worktrees/issue-<repo>-<N>/<layer>_ws/src/<project_repo>/.agent/work-plans/...`,
# so glob through the layer-workspace and project-repo tiers:
ls layers/worktrees/issue-*-<N>/*_ws/src/*/.agent/work-plans/issue-<N>/plan.md 2>/dev/null
```

**Cross-repo lookups**: `--repo <owner/repo>` redirects only `gh`
queries (issue title via `gh issue view`, PR metadata via `gh pr view`)
to the named repo. The plan file is always read from the current
repo's worktree — there's no cross-repo plan-file resolution. If you
need to review a plan that lives in a different repo, enter that
repo's worktree first, then run the skill.

If still not found, stop and inform the user.

### 2. Read the issue and any review-issue comments

```bash
# The linked issue (pass --repo to target a non-current-worktree repo)
ISSUE_NUM=<extracted from step 1>
gh issue view "$ISSUE_NUM" [--repo <owner/repo>] --json title,body,labels,comments,url
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
**Plan file**: `.agent/work-plans/issue-<N>/plan.md`

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

**PR-less format** — when reviewing via `--issue` or a file path (no PR
exists), replace the PR header:

```markdown
## Plan Review: #<N> — <title>

**Issue**: #<N> — <issue-title>
**Plan file**: `.agent/work-plans/issue-<N>/plan.md`
**Branch**: `<branch-name>` (if in a worktree, otherwise omit)
```

If no findings, output (PR mode):

```markdown
## Plan Review: PR #<N> — <title>

**PR**: <url>
Plan looks solid. Ready for implementation.
```

PR-less no-findings variant (`--issue` or file path):

```markdown
## Plan Review: #<N> — <title>

**Issue**: #<N> — <issue-title>
**Plan file**: `.agent/work-plans/issue-<N>/plan.md`
Plan looks solid. Ready for implementation.
```

### 6. Persist to progress.md

Append a `## Plan Review` entry to
`.agent/work-plans/issue-<N>/progress.md` in the worktree that owns
the plan. Per [ADR-0013](../../../docs/decisions/0013-progress-md-entry-type-vocabulary.md).
The entry lets implementation skills and downstream consumers (notably
`triage-reviews` as integrator) see what the plan review concluded
without re-reading the entire PR conversation.

**Locate-or-create the owning worktree.** `review-plan` supports
PR-number, file-path, and `--issue` invocations, none of which
guarantee the agent is already inside the owning worktree. Before
appending, resolve the right place to write:

1. **Resolve the owning repo and issue number** — `<N>` already came
   from step 1 (the PR's `closingIssuesReferences`, the issue argument,
   or extracted from the plan path). Determine the owning repo with
   `gh issue view <N> --repo <owner/repo> --json repository --jq
   '.repository.nameWithOwner'` if it isn't already known from the
   step-1 PR metadata.
2. **Check the current worktree** — if `$WORKTREE_ISSUE` matches `<N>`
   *and* `$WORKTREE_REPO` matches the owning repo's short slug, the
   current directory is the right worktree. Skip to "Append" below.
3. **Find an existing worktree** with an anchored, repo-aware match —
   same pattern as `review-issue` step 8a.3:
   ```bash
   .agent/scripts/worktree_list.sh \
     | grep -E "issue-(workspace|<repo-slug>)-<N>($|[^0-9])"
   ```
   If one exists, record its path as `<plan-worktree-path>` (used in
   the commit commands below). When the issue number collides across
   repos, pass `--repo-slug <slug>` if entering via
   `worktree_enter.sh`.
4. **Otherwise create one on demand**, mirroring `review-issue` step
   8a.5: `.agent/scripts/worktree_create.sh --issue <N> --type
   workspace` for workspace issues, or `--type layer --layer <layer>
   --packages <project_repo>` for project-repo issues. No `--plan-file`
   — the plan already exists, we just need a branch to commit
   progress.md on.
5. **Initialise progress.md if absent** — if
   `<plan-worktree-path>/.agent/work-plans/issue-<N>/progress.md`
   doesn't exist, create it with the standard frontmatter:
   ```yaml
   ---
   issue: <N>
   ---

   # Issue #<N> — <issue title>
   ```
   (Fetch the issue title from the owning repo via `gh issue view <N>
   --repo <owner/repo> --json title --jq '.title'`.)

**Independence annotation.** If this review is the *plan author*
re-reading their own work — detect by checking whether `$AGENT_NAME`
matches the `**By**` field of the existing `## Plan Authored` entry
in the same `progress.md` — annotate the entry's `**By**` field with
`(in-context — author self-review)` so downstream consumers can weight
the entry appropriately. Otherwise the review is independent (fresh
context or different agent) and no annotation is needed.

Append:

```markdown

## Plan Review
**Status**: complete
**When**: <YYYY-MM-DD HH:MM>
**By**: <agent name> (<model>)  <!-- append ` (in-context — author self-review)` per the detection above when applicable -->

**Plan**: `.agent/work-plans/issue-<N>/plan.md` at `<short-sha-of-plan-commit>`
**PR**: <plan PR URL, or "PR-less" if --issue / file path mode>
**Verdict**: <approve | approve-with-suggestions | changes-requested>

### Findings
- [ ] (must-fix) <one-line summary> — `plan.md:<line>`
- [ ] (suggestion) <one-line summary> — `plan.md:<line>`
```

If the review surfaced no findings, set `**Verdict**: approve` and
write `Plan looks solid. Ready for implementation.` under `### Findings`.

Commit:

```bash
git -C <plan-worktree-path> add .agent/work-plans/issue-<N>/progress.md
git -C <plan-worktree-path> \
    -c user.name="$AGENT_NAME" \
    -c user.email="$AGENT_EMAIL" \
    commit -m "progress: plan review for #<N>"
```

The per-invocation `-c` overrides are required by
[AGENTS.md § Agent Commit Identity](../../../AGENTS.md#agent-commit-identity)
to keep commits on agent-convention branches. Plain `git commit` in
a fresh subshell can fall back to the human git config and trip the
[`check_pr_authors.py`](../../../.agent/hooks/check_pr_authors.py)
CI check.

If the review found anything must-fix or substantive, the
implementation step (or the plan author) should address them and amend
the plan inline per `plan-task`'s "During implementation" rules before
work continues. The `## Plan Review` entry stays as a historical record.

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
