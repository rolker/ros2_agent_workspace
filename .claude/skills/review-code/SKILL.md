---
name: review-code
description: Lead reviewer that orchestrates specialist sub-reviews (static analysis, governance, plan drift, adversarial) to evaluate a PR or pre-push diff. Scales review depth to change risk. Produces a unified structured report and persists findings to progress.md.
---

# Review Code

## Usage

```
/review-code                                # pre-push: diff vs default branch
/review-code --base <branch>                # pre-push, override default base
/review-code <pr-number>                    # post-PR: diff vs PR base
/review-code <pr-url>                       # post-PR
/review-code <pr-number> [light|standard|deep]   # post-PR with depth override
/review-code <branch-name>                  # pre-push, force a base override
```

The optional depth keyword (`light` / `standard` / `deep`) overrides
automatic classification.

## Overview

**Lifecycle position**: review-issue → plan-task → review-plan →
implement → **review-code** → push / open PR → triage-reviews

Multi-specialist code review system. A lead reviewer gathers context,
classifies review depth based on change risk, dispatches specialist
sub-reviews in parallel, collects findings, deduplicates, applies a
silence filter, produces a unified report, and appends a step entry to
the issue's `progress.md` so findings persist across sessions. Does not
post comments or modify the PR unless the user asks.

**Two modes**:
- **Pre-push** (default, no arg) — diffs against the current repo's
  default branch. Run before `git push` / opening a PR to catch findings
  while still locally fixable. No PR-side context (comments, PR body)
  available.
- **Post-PR** (`<N>` or URL) — diffs against the PR base branch via
  `gh pr view`. Includes PR-side context: existing comments, linked
  issue, plan file referenced in PR body.

**Depth tiers** (see `.agent/knowledge/review_depth_classification.md`):
- **Light** — Static Analysis only (small, low-risk changes)
- **Standard** — Static Analysis + Governance + Plan Drift + Claude
  Adversarial (medium changes or governance-touching files)
- **Deep** — Same as Standard with Adversarial primed for security /
  concurrency / lifecycle (large changes, security, or cross-layer)

**Specialists**:
- **Static Analysis** — runs linters with ament-aligned configs on
  changed files
- **Governance** — evaluates against principles, ADRs, and the
  consequences map
- **Plan Drift** — compares implementation against the work plan (if one
  exists)
- **Adversarial** — fresh-context Claude subagent that re-reads the diff
  cold (Standard + Deep). Cross-model adversarial is **not** wired in
  here; see `inspiration_agent_workspace_digest.md` "Not adopted".

## Steps

### 1. Detect mode and gather diff context

Determine pre-push vs post-PR mode from the arguments. Parse out the
optional depth keyword first (`light` / `standard` / `deep` — positional,
matching the Usage block above), then classify what remains:

- Empty, or only `--base <branch>` → **pre-push mode**
- A number or `https://github.com/.../pull/<N>` → **post-PR mode**

The depth keyword is positional and may appear after the PR number /
URL or after `--base <branch>`. The same syntax applies in both modes.
Examples:

```
/review-code                       # pre-push, auto-classify
/review-code deep                  # pre-push, force Deep
/review-code --base develop        # pre-push, override base
/review-code --base develop deep   # pre-push with override + force Deep
/review-code 42                    # post-PR, auto-classify
/review-code 42 standard           # post-PR, force Standard
```

#### Pre-push mode

```bash
# Repo root and default branch
REPO_ROOT=$(git rev-parse --show-toplevel)
DEFAULT_BRANCH=$(gh repo view --json defaultBranchRef --jq '.defaultBranchRef.name' 2>/dev/null || echo "main")
BASE="${USER_BASE:-$DEFAULT_BRANCH}"

# Make sure base is current
git fetch origin "$BASE" --quiet

# Diff and stats
git diff "origin/$BASE...HEAD" --stat
git diff "origin/$BASE...HEAD"
git diff "origin/$BASE...HEAD" --numstat   # per-file +/- counts

# Linked issue from branch name (feature/issue-<N> or feature/ISSUE-<N>-...)
BRANCH=$(git branch --show-current)
ISSUE_NUM=$(echo "$BRANCH" | grep -oE 'issue-[0-9]+|ISSUE-[0-9]+' | grep -oE '[0-9]+' | head -1)
```

If `gh repo view` fails (no GitHub remote, offline), fall back to `main`
and proceed. If no issue number can be extracted from the branch name,
note "no linked issue" in the report header — the review still runs but
plan-drift and progress.md persistence are skipped.

#### Post-PR mode

```bash
# PR metadata
gh pr view <N> --json title,body,baseRefName,headRefName,headRefOid,files,additions,deletions,url,comments,reviews

# Full diff
gh pr diff <N>

# Linked issue (from "Closes #<N>" in PR body)
gh pr view <N> --json body --jq '.body' | grep -oE '#[0-9]+' | head -1
```

In both modes, identify:
- What repo the diff lives in (workspace or project repo?)
- What files changed and in which directories
- The linked issue and its requirements (when resolvable)
- Whether a work plan exists at `.agent/work-plans/issue-<N>/plan.md` in
  the repo that owns the issue

Read the **full content** of each changed file (not just the diff hunks)
to understand surrounding context.

### 2. Classify review depth

Load `.agent/knowledge/review_depth_classification.md` and apply the risk
signals from step 1:

1. Count total lines changed (additions + deletions).
2. Count files changed.
3. Check file paths against the workspace-repo and project-repo override
   trigger lists.
4. Check Deep promotion triggers (security-relevant, cross-layer, ADR add
   or substantive ADR rewrite).
5. Apply tier promotion logic — highest tier wins.

**User override**: If the `/review-code` invocation includes a depth
keyword (`light`, `standard`, or `deep`), use that tier instead of the
automatic classification.

Record the tier and the primary signal that determined it for the report
header.

### 3. Load project context

For project repo PRs:
- Read `.agents/README.md` for architecture overview, key files,
  cross-layer dependencies, and pitfalls.
- Check for `.agents/review-context.yaml` — if present, use it for the
  compact relevance map (packages, topics, dependencies).
- **Staleness check**: If `review-context.yaml` exists, compare its
  `context_generated_from_sha` field against the current HEAD of the
  project repo. If they differ, include a warning in the report header:

  > ⚠ Review context is stale (generated from `<sha>`; repo HEAD is `<sha>`).
  > Consider running `/gather-project-knowledge` to refresh.

  If `review-context.yaml` does not exist, note this in the report
  header:

  > ℹ No review-context.yaml found. Review proceeds with .agents/README.md only.

- Read project `PRINCIPLES.md` if it exists.
- Check `.agent/project_knowledge/` symlink for workspace-level project
  summaries.

### 4. Classify changed files for static analysis

Determine the linter profile for each changed file:

| File location | Language detection | Linter config profile |
|---|---|---|
| `layers/*/src/**/*.py` | Python | ament (max-line-length=99, ament ignores) |
| `layers/*/src/**/*.cpp`, `*.hpp` | C++ | ament (cpplint, cppcheck) |
| `.agent/scripts/*.py` | Python | workspace (max-line-length=100, Black compat) |
| `.agent/scripts/*.sh` | Shell | workspace (shellcheck --severity=warning) |
| `*.yaml`, `*.yml` | YAML | yamllint (max-line-length=120) |
| `*.xml`, `*.launch.xml` | XML | xmllint |
| `*.md` | Markdown | (no linter — content review only) |

See `.agent/knowledge/review_static_analysis.md` for full tool configs.

### 5. Dispatch specialists

Dispatch specialists based on the depth tier from step 2. Run independent
specialists in parallel — use the `Agent` tool with subagents when
available so each runs in its own context window; otherwise evaluate
sequentially.

#### Light tier

Run only:
- **5a. Static Analysis Specialist**

#### Standard tier

Run all of:
- **5a. Static Analysis Specialist**
- **5b. Governance Specialist**
- **5c. Plan Drift Specialist** (if a plan exists)
- **5d. Adversarial Specialist** (Standard prompt)

#### Deep tier

Same as Standard, but **5d. Adversarial Specialist** runs with the Deep
prompt (broader file horizon plus an explicit security / concurrency /
lifecycle checklist).

---

#### 5a. Static Analysis Specialist

Run linters on **changed files only**, using the config profile from
step 4. See `.agent/knowledge/review_static_analysis.md` for exact
commands and flags.

If **no linter profile matches any changed file**, report this
explicitly: "No static analysis profile configured for these file types
(`.ext1`, `.ext2`)." Don't silently produce an empty findings section —
the reviewer and user need to know that absence of findings means "not
checked", not "code is clean."

Report each finding as:
- File, line number, tool name, message
- Skip findings on lines not touched by this PR (context-only lines)

#### 5b. Governance Specialist

Load governance context:
- `.agent/knowledge/principles_review_guide.md` — evaluation criteria
- `docs/PRINCIPLES.md` — workspace principles
- `docs/decisions/*.md` — ADRs (scan titles, read those triggered by this
  change)
- Project-level governance (if applicable)

**Principle evaluation**: For each relevant principle, assess the PR:

| Verdict | Meaning |
|---|---|
| **Pass** | PR clearly adheres |
| **Watch** | Not a violation, but worth noting |
| **Concern** | Potential violation that should be addressed |
| **N/A** | Principle doesn't apply |

Skip principles that clearly don't apply.

**ADR compliance**: Using the ADR applicability table, identify triggered
ADRs. For each: does the PR comply with the key requirement?

**Consequence check**: Using the consequences map, check if this PR
changes something in the "If you change..." column. Are the corresponding
"Also update..." items addressed? Mark each as Done or Missing.

**Existing review comments** (post-PR mode only): Check for unresolved
human and bot comments:

```bash
.agent/scripts/fetch_pr_reviews.sh --pr <N>
```

Note unresolved human comments (high priority), valid bot findings, and
false positives.

#### 5c. Plan Drift Specialist

If a work plan exists at `.agent/work-plans/issue-<N>/plan.md`:
- Read the plan's "Approach" and "Files to Change" sections.
- Compare against the actual diff:
  - Files listed in plan but not changed? (incomplete)
  - Files changed but not in plan? (scope creep or oversight)
  - Approach deviations? (different from what was planned)
- Report deviations as suggestions (not must-fix — plans are guides, not
  contracts).

If no work plan exists, skip this specialist.

#### 5d. Adversarial Specialist

**Activates at**: Standard, Deep.

Launch as a **fresh Claude subagent** via the `Agent` tool with no
context from the other specialists. The adversarial reviewer reads the
diff and full changed files independently — that fresh-context dispatch
is the whole point. An independent reviewer that agrees with the
governance specialist is a stronger signal than one told what to look
for.

**Cross-repo limitation**: the Adversarial Specialist only sees the diff
and the files it explicitly opens. In a layered workspace, cross-repo
consequences (a workspace ADR change that affects project repos, a shared
message-package edit that breaks downstream nodes) won't surface from
fresh-context reading alone. The Governance Specialist carries that load
via the consequences map; Adversarial is not a substitute for it.

Standard-tier prompt focus areas:
- Missed edge cases and boundary conditions
- Assumption violations (what does the code assume that might not hold?)
- Subtle bugs (off-by-one, race conditions, resource leaks)
- Logic errors (does the code actually do what the PR claims?)

Deep-tier prompt adds:
- Security implications (injection, auth bypass, data exposure)
- Concurrency / lifecycle (lock ordering, init/destroy ordering, signal
  handling, shutdown paths)
- Cross-cutting effects (does this change interact with caching,
  retries, error propagation, or other system-wide behaviour?)

Report findings in the same format as other specialists (file, line,
severity, description). The silence filter (step 6) deduplicates overlap
with other specialists' findings.

> **Cross-model adversarial is intentionally not wired here.** See
> `inspiration_agent_workspace_digest.md` "Not adopted". When you want
> a second model's read on a Deep PR, run that agent's review-code
> manually.

### 6. Apply silence filter

Collect all findings from all dispatched specialists and filter:

1. **Deduplicate** — if multiple specialists flag the same issue (common
   between adversarial and governance), keep the more specific one.
2. **Drop linter-enforced nits** — if pre-commit or CI already catches
   it, don't report it again (the author will see it on commit/push).
3. **Merge related findings** — group findings about the same logical
   issue.
4. **Classify severity**:
   - **Must-fix** — bugs, security issues, principle violations, missing
     consequences
   - **Suggestion** — improvements worth the author's time
   - Drop anything below suggestion threshold.
5. **Silence check** — if no findings survive the filter, report "No
   issues found." Don't invent feedback to fill the report. Target: ≥85%
   of reported findings should be actionable.

### 7. Produce the report

```markdown
## Code Review: <#N or branch> — <title>

**PR**: <url>          (post-PR mode)
**Branch**: `<branch>` (pre-push mode)
**Issue**: #<issue> — <issue-title>
**Repo**: workspace | <project-repo>
**Files changed**: <count> (+<additions> -<deletions>)
**Review depth**: <Light|Standard|Deep> (reason: <primary signal>)
**Context**: <status of review-context.yaml — fresh / stale / not found / N/A>

### Must-Fix

| # | Source | File | Line | Finding |
|---|--------|------|------|---------|
| 1 | <specialist> | `path` | 42 | Description |

### Suggestions

| # | Source | File | Line | Finding |
|---|--------|------|------|---------|
| 1 | <specialist> | `path` | 10 | Description |

### Governance

| Principle | Verdict | Notes |
|---|---|---|
| ... | ... | ... |

| ADR | Triggered | Compliant | Notes |
|---|---|---|---|
| ... | ... | ... | ... |

| Changed | Required update | Status |
|---|---|---|
| ... | ... | Done / Missing |

### Plan Adherence

<comparison summary, or "No work plan found">

### Existing Review Comments

<post-PR mode only — summary of unresolved comments, if any>

### Summary

<1-3 sentence overall assessment>

### Recommended Actions

- [ ] <specific action items, if any>
```

**Light tier condensed format** — skip Governance, Plan Adherence, and
Existing Review Comments sections. Use:

```markdown
## Code Review: <#N or branch> — <title>

**PR / Branch**: ...
**Review depth**: Light (reason: <primary signal>)

### Static Analysis

| # | File | Line | Finding |
|---|------|------|---------|
| 1 | `path` | 42 | Description |

No governance concerns for a change of this scope.
```

**No findings format** — if no findings exist across all sections:

```markdown
## Code Review: <#N or branch> — <title>

**PR / Branch**: ...
**Review depth**: <tier> (reason: <signal>)
No issues found. LGTM.
```

### 8. Persist review summary to progress.md

After outputting the report to the conversation, append a step entry to
`progress.md` so findings survive across sessions.

**Locate or create progress.md**: Use the issue number resolved in step 1.
Determine which repo owns the linked issue (workspace repo for workspace
issues, project repo for project issues). Check
`.agent/work-plans/issue-<N>/progress.md` in the owning repo's worktree
first. If it doesn't exist there, check the current worktree. If neither
exists, create it in the owning repo's worktree (or current worktree if
no owning worktree exists). Fetch the issue title via:

```bash
gh issue view <N> --repo <owner/repo> --json title --jq '.title'
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

## Local Review
**Status**: complete
**When**: <YYYY-MM-DD HH:MM>
**By**: <agent name> (<model>)
**Verdict**: <approved|changes-requested>

**PR / Branch**: <#N or branch> at `<short-sha>`
**Mode**: <pre-push | post-PR>
**Depth**: <tier> (reason: <signal>)
**Must-fix**: <count> | **Suggestions**: <count>

### Findings
- [ ] (must-fix) <one-line summary> — `file:line`
- [ ] (suggestion) <one-line summary> — `file:line`
```

If no findings survived the silence filter, set `**Verdict**: approved`,
`**Must-fix**: 0 | **Suggestions**: 0`, and write `No issues found.
LGTM.` under Findings.

Key points:
- Use `- [ ]` checkboxes so findings can be checked off as addressed.
- Include only the one-line summary and location, not the full
  description.
- Commit progress.md after appending. Run `git add` and `git commit` in
  the worktree where progress.md was found or created (which may differ
  from the current working directory):
  ```bash
  git -C <worktree-path> add .agent/work-plans/issue-<N>/progress.md
  git -C <worktree-path> commit -m "progress: local review for #<N>"
  ```
- If no issue number was resolved in step 1, skip persistence and note
  this in the report Summary ("Findings not persisted: no linked issue").

## Guidelines

- **Report first, then persist** — output the review in the conversation,
  append a step to `progress.md`, and commit it (step 8). The user
  decides whether to post it as a PR comment, request changes, or act on
  findings.
- **Be specific** — "Must-fix: null check missing before `result.data`
  access at line 42" is useful. "Watch: could add more error handling"
  is not.
- **Read the code** — don't just check file names. Read full files and
  the diff to evaluate correctness and principle adherence.
- **Silence is a feature** — saying nothing when there's nothing to say
  is better than generating low-value comments. If the code is fine, say
  so briefly.
- **Project governance** — for project repo PRs, apply both workspace and
  project governance. Note conflicts between them if any.
- **Severity matters** — every finding must be classified as must-fix or
  suggestion. Unclassified findings are noise.
- **Context-aware linting** — use ament configs for ROS package code,
  pre-commit configs for workspace infrastructure code. Never mix them.
- **Depth is transparent** — always show the tier and reason in the
  report header. If the user disagrees with the classification, they can
  re-run with an explicit depth keyword.
- **Pre-push mode caveats** — no PR comments, no PR body to extract a
  plan reference from, no `Closes #` link unless it's already in the
  branch name. Plan-drift still works (plan is a file in the repo);
  Existing-Review-Comments doesn't.
