---
name: review-code
description: Lead reviewer that orchestrates specialist sub-reviews (static analysis, governance, plan drift) to evaluate a PR. Produces a unified structured report.
---

# Review Code

## Usage

```
/review-code <pr-number-or-url>
```

## Overview

**Lifecycle position**: review-issue → plan-task → review-plan → implement → **review-code**

Multi-specialist code review system. A lead reviewer gathers context, dispatches
specialist sub-reviews in parallel, collects findings, deduplicates, applies a
silence filter, and produces a unified report. Does not post comments or modify
the PR unless the user asks.

**Specialists** (Phase 1):
- **Static Analysis** — runs linters with ament-aligned configs on changed files
- **Governance** — evaluates against principles, ADRs, and consequences
- **Plan Drift** — compares implementation against the work plan (if one exists)

## Steps

### 1. Gather PR context

```bash
# PR metadata
gh pr view <N> --json title,body,baseRefName,headRefName,files,additions,deletions,url,comments,reviews

# Full diff
gh pr diff <N>

# Linked issue
gh pr view <N> --json body --jq '.body' | grep -o '#[0-9]*'
```

Identify:
- What repo the PR targets (workspace or project repo?)
- What files changed and in which directories
- The linked issue and its requirements
- Whether a work plan exists (`.agent/work-plans/PLAN_ISSUE-*.md` in the PR's target repo)

Read the **full content** of each changed file (not just the diff hunks) to
understand surrounding context.

### 2. Load project context

For project repo PRs:
- Read `.agents/README.md` for architecture overview, key files, cross-layer
  dependencies, and pitfalls
- Check for `.agents/review-context.yaml` — if present, use it for the compact
  relevance map (packages, topics, dependencies)
- **Staleness check**: If `review-context.yaml` exists, compare its
  `context_generated_from_sha` field against the current HEAD of the project
  repo. If they differ, include a warning in the report header:

  > ⚠ Review context is stale (generated from `<sha>`; repo HEAD is `<sha>`).
  > Consider running `/gather-project-knowledge` to refresh.

  If `review-context.yaml` does not exist, note this in the report header:

  > ℹ No review-context.yaml found. Review proceeds with .agents/README.md only.

- Read project `PRINCIPLES.md` if it exists
- Check `.agent/project_knowledge/` symlink for workspace-level project summaries

### 3. Classify changed files

Determine the review profile for each changed file:

| File location | Language detection | Linter config profile |
|---|---|---|
| `layers/*/src/**/*.py` | Python | ament (max-line-length=99, ament ignores) |
| `layers/*/src/**/*.cpp`, `*.hpp` | C++ | ament (cpplint, cppcheck) |
| `.agent/scripts/*.py` | Python | workspace (max-line-length=100, Black compat) |
| `.agent/scripts/*.sh` | Shell | workspace (shellcheck --severity=warning) |
| `*.yaml`, `*.yml` | YAML | yamllint (max-line-length=120) |
| `*.xml`, `*.launch.xml` | XML | xmllint |

See `.agent/knowledge/review_static_analysis.md` for full tool configs.

### 4. Dispatch specialists

Run specialists in parallel (use Task tool with subagents when available,
otherwise evaluate sequentially):

#### 4a. Static Analysis Specialist

Run linters on **changed files only**, using the config profile from step 3.
See `.agent/knowledge/review_static_analysis.md` for exact commands and flags.

Report each finding as:
- File, line number, tool name, message
- Skip findings on lines not touched by this PR (context-only lines)

#### 4b. Governance Specialist

Load governance context:
- `.agent/knowledge/principles_review_guide.md` — evaluation criteria
- `docs/PRINCIPLES.md` — workspace principles
- `docs/decisions/*.md` — ADRs (scan titles, read those triggered by this change)
- Project-level governance (if applicable)

**Principle evaluation**: For each relevant principle, assess the PR:

| Verdict | Meaning |
|---|---|
| **Pass** | PR clearly adheres |
| **Watch** | Not a violation, but worth noting |
| **Concern** | Potential violation that should be addressed |
| **N/A** | Principle doesn't apply |

Skip principles that clearly don't apply.

**ADR compliance**: Using the ADR applicability table, identify triggered ADRs.
For each: does the PR comply with the key requirement?

**Consequence check**: Using the consequences map, check if this PR changes
something in the "If you change..." column. Are the corresponding "Also update..."
items addressed? Mark each as Done or Missing.

**Existing review comments**: Check for unresolved human and bot comments:

```bash
.agent/scripts/fetch_pr_reviews.sh --pr <N>
```

Note unresolved human comments (high priority), valid bot findings, and false
positives.

#### 4c. Plan Drift Specialist

If a work plan exists (`.agent/work-plans/PLAN_ISSUE-*.md`):
- Read the plan's "Approach" and "Files to Change" sections
- Compare against the actual diff:
  - Files listed in plan but not changed? (incomplete)
  - Files changed but not in plan? (scope creep or oversight)
  - Approach deviations? (different from what was planned)
- Report deviations as suggestions (not must-fix — plans are guides, not contracts)

If no work plan exists, skip this specialist.

### 5. Apply silence filter

Collect all findings from specialists and filter:

1. **Deduplicate** — if static analysis and governance flag the same issue,
   keep the more specific one
2. **Drop linter-enforced nits** — if pre-commit or CI already catches it,
   don't report it again (the author will see it when they commit/push)
3. **Merge related findings** — group findings about the same logical issue
4. **Classify severity**:
   - **Must-fix** — bugs, security issues, principle violations, missing
     consequences
   - **Suggestion** — improvements worth the author's time
   - Drop anything below suggestion threshold
5. **Silence check** — if no findings survive the filter, report "No issues
   found." Do not invent feedback to fill the report. Target: >=85% of
   reported findings should be actionable.

### 6. Produce the report

```markdown
## Code Review: #<N> — <title>

**PR**: <url>
**Issue**: #<issue> — <issue-title>
**Repo**: workspace | <project-repo>
**Files changed**: <count> (+<additions> -<deletions>)
**Context**: <status of review-context.yaml — fresh / stale / not found>

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

- <summary of unresolved comments, if any>

### Summary

<1-3 sentence overall assessment>

### Recommended Actions

- [ ] <specific action items, if any>
```

If no findings exist across all sections, output:

```markdown
## Code Review: #<N> — <title>

**PR**: <url>
No issues found. LGTM.
```

## Guidelines

- **Report, don't act** — output the review in the conversation. The user
  decides whether to post it as a PR comment, request changes, or act on it.
- **Be specific** — "Must-fix: null check missing before `result.data` access
  at line 42" is useful. "Watch: could add more error handling" is not.
- **Read the code** — don't just check file names. Read full files and the diff
  to evaluate correctness and principle adherence.
- **Silence is a feature** — saying nothing when there's nothing to say is
  better than generating low-value comments. If the code is fine, say so briefly.
- **Project governance** — for project repo PRs, apply both workspace and project
  governance. Note conflicts between them if any.
- **Severity matters** — every finding must be classified as must-fix or
  suggestion. Unclassified findings are noise.
- **Context-aware linting** — use ament configs for ROS package code, pre-commit
  configs for workspace infrastructure code. Never mix them.
