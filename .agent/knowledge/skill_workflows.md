# Skill Workflows

## Per-Issue Lifecycle

The governance skills follow a sequence from idea exploration through pre-merge PR review:

```
brainstorm → review-issue → plan-task → implement → review-code
```

Each step is optional — simple issues can skip straight to `plan-task` or implementation.

## Skill Index

### Lifecycle skills (per-issue sequence)

| Skill | Position | Purpose |
|-------|----------|---------|
| `brainstorm` | Before review-issue | Explore possibilities using research digests and project knowledge |
| `review-issue` | Before plan-task | Evaluate issue scope, principle alignment, and ADR applicability |
| `plan-task` | Before implementation | Generate a principles-aware work plan, commit to branch |
| `review-code` | After implementation | Evaluate PR against principles, ADRs, and consequences |

### Utility skills (on-demand / periodic)

| Skill | When to use | Purpose |
|-------|-------------|---------|
| `research` | Any time | Survey external sources, maintain living research digests |
| `gather-project-knowledge` | After repo changes | Scan repos and generate project knowledge summaries |
| `audit-workspace` | Periodically | Check workspace governance health: enforcement, drift, staleness |
| `audit-project` | Before or after repo work | Check a project repo against workspace and project conventions |
| `onboard-project` | When onboarding a repo | Interactive audit + fix: add CI, pre-commit, agent guide, GitHub settings |
| `skill-importer` | When importing external skills | Evaluate, adapt, and import skills from external sources |
| `document-package` | After audit-project flags doc gaps | Generate or update ROS 2 package README and API docs from source |
| `issue-triage` | Periodically | Cross-repo issue scanning, categorization, and stale issue detection |
| `test-engineering` | After audit-project flags test gaps | Test scaffolding, debugging, and coverage analysis for ROS 2 packages |

### Makefile skills

`make_*` skills are auto-generated wrappers around Makefile targets (e.g.,
`/make_build`, `/make_test`). Run `make generate-skills` after adding or
removing `.PHONY` targets to keep them current.
