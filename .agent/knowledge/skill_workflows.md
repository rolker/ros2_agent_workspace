# Skill Workflows

## Per-Issue Lifecycle

The governance skills follow a sequence from idea exploration through
post-PR review triage:

```
brainstorm → review-issue → plan-task → review-plan → implement
          → review-code → push / open PR → triage-reviews
```

Most steps are optional — simple issues can skip straight to
`plan-task` or implementation. **`review-code` is the exception**:
AGENTS.md Post-Task Verification step 5 makes a pre-push `review-code`
pass an expected check before opening a PR (the framework adapters in
`.github/copilot-instructions.md`, `.agent/instructions/gemini-cli.instructions.md`,
and `.agent/AGENT_ONBOARDING.md` mirror this for non-Claude runtimes
with the Adversarial-Claude-only caveat). Pre-push mode (no
arguments) catches static-analysis, governance, plan-drift, and
Adversarial findings while still cheap to fix locally; `review-code`
also accepts a PR number / URL for post-PR review of someone else's
work. `triage-reviews` runs after a PR has accumulated review comments
(human + bot) and classifies each against the current code.

## Skill Index

### Lifecycle skills (per-issue sequence)

| Skill | Position | Purpose |
|-------|----------|---------|
| `brainstorm` | Before review-issue | Explore possibilities using research digests and project knowledge |
| `review-issue` | Before plan-task | Evaluate issue scope, principle alignment, and ADR applicability |
| `plan-task` | Before implementation | Generate a principles-aware work plan, commit to branch |
| `review-plan` | After plan-task, before implementation | Independent evaluation of a committed work plan (accepts PR / file path / `--issue <N>`) |
| `review-code` | Between implement and push / open PR (also post-PR for someone else's diff) | Lead reviewer orchestrating specialist sub-reviews (static analysis, governance, plan drift, adversarial). Pre-push mode is the default; depth tiers (Light / Standard / Deep) scale specialist count to change risk |
| `triage-reviews` | After PR review comments arrive | Classifies each comment (human + bot) against current code, persists triage to `progress.md` |

### Utility skills (on-demand / periodic)

| Skill | When to use | Purpose |
|-------|-------------|---------|
| `research` | Any time | Survey external sources, maintain living research digests |
| `gather-project-knowledge` | After repo changes | Scan repos and generate project knowledge summaries |
| `audit-workspace` | Periodically | Check workspace governance health: enforcement, drift, staleness |
| `audit-project` | Before or after repo work | Check a project repo against workspace and project conventions |
| `onboard-project` | When onboarding a repo | Interactive audit + fix: add CI, pre-commit, agent guide, GitHub settings |
| `skill-importer` | When importing external skills | Evaluate, adapt, and import skills from external sources |
| `inspiration-tracker` | Periodically | Track external projects for portable enhancements and interesting patterns |
| `document-package` | After audit-project flags doc gaps | Generate or update ROS 2 package README and API docs from source |
| `issue-triage` | Periodically | Cross-repo issue scanning, categorization, and stale issue detection |
| `test-engineering` | After audit-project flags test gaps | Test scaffolding, debugging, and coverage analysis for ROS 2 packages |

### Makefile skills

`make_*` skills are auto-generated wrappers around Makefile targets (e.g.,
`/make_build`, `/make_test`). Run `make generate-skills` after adding or
removing `.PHONY` targets to keep them current.
