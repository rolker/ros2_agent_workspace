# Skill Workflows

## Per-Issue Lifecycle

The governance skills follow a sequence from idea exploration through pre-merge PR review:

```
brainstorm → review-issue → plan-task → implement → review-pr
```

Each step is optional — simple issues can skip straight to `plan-task` or implementation.

## Skill Index

### Lifecycle skills (per-issue sequence)

| Skill | Position | Purpose |
|-------|----------|---------|
| `brainstorm` | Before review-issue | Explore possibilities using research digests and project knowledge |
| `review-issue` | Before plan-task | Evaluate issue scope, principle alignment, and ADR applicability |
| `plan-task` | Before implementation | Generate a principles-aware work plan, commit to branch |
| `review-pr` | After implementation | Evaluate PR against principles, ADRs, and consequences |

### Utility skills (on-demand / periodic)

| Skill | When to use | Purpose |
|-------|-------------|---------|
| `research` | Any time | Survey external sources, maintain living research digests |
| `gather-project-knowledge` | After repo changes | Scan repos and generate project knowledge summaries |
| `audit-workspace` | Periodically | Check workspace governance health: enforcement, drift, staleness |
| `audit-project` | Before or after repo work | Check a project repo against workspace and project conventions |

### Makefile skills

`make_*` skills are auto-generated wrappers around Makefile targets (e.g.,
`/make_build`, `/make_test`). Run `make generate-skills` after adding or
removing `.PHONY` targets to keep them current.
